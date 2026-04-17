// Embedded Python 3.11 runtime.
//
// Matches the original binary's approach: Py_Initialize() once at startup,
// one module object per active Python pipeline, and `runPipeline(image,
// llrobot)` invoked with GIL held on each frame.
//
// To avoid leaking PyObject* through the public header we use a pimpl:
// PythonRuntime::Impl holds the interpreter thread state and a map of
// pipeline-index -> loaded module.

#include "visionserver/python_runtime.h"

#include <spdlog/spdlog.h>

#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <opencv2/core.hpp>

#include <filesystem>
#include <map>
#include <mutex>

namespace limelight {

namespace {

// Convert a cv::Mat (BGR uint8) into a flat python `bytes` object. We do
// NOT build a numpy array directly — numpy's C API requires linking against
// numpy internals, and the user scripts we ship expect an OpenCV-style Mat
// object. Instead we expose a small shim module that converts bytes back
// into numpy before calling `runPipeline()`.
//
// The shim is injected into each sub-interpreter as `_ll_shim`.
static const char* kShim = R"PY(
import numpy as np
import cv2

def make_image(data, h, w, c):
    arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, c)
    return arr

def call_run(mod, data, h, w, c, llrobot):
    img = make_image(data, h, w, c)
    res = mod.runPipeline(img, list(llrobot))
    # res must be (contour, image, llpython)
    contour, image, llpython = res
    # Make a copy so the caller doesn't depend on python-managed memory.
    if image is None:
        out_bytes = b""
        oh, ow, oc = 0, 0, 0
    else:
        oh, ow = image.shape[:2]
        oc = image.shape[2] if image.ndim == 3 else 1
        out_bytes = image.tobytes()
    # Normalize llpython to exactly 8 floats.
    llp = list(llpython) if llpython is not None else []
    llp = (llp + [0.0] * 8)[:8]
    # Normalize contour to a flat list of [x, y] pairs.
    contour_flat = []
    if contour is not None:
        try:
            for p in contour:
                if hasattr(p, "__len__") and len(p) >= 2:
                    contour_flat.append([float(p[0]), float(p[1])])
        except Exception:
            pass
    return out_bytes, oh, ow, oc, llp, contour_flat
)PY";

}  // namespace

struct PythonRuntime::Impl {
    PyThreadState* main_ts = nullptr;
    PyObject*      shim    = nullptr;  // the _ll_shim module
    std::map<int, PyObject*> modules;  // pipeline_index -> loaded user module
    std::string    last_error;

    void initPy() {
        if (Py_IsInitialized()) return;
        PyConfig config;
        PyConfig_InitPythonConfig(&config);
        Py_InitializeFromConfig(&config);
        PyConfig_Clear(&config);

        // Release the GIL so other threads can acquire it.
        main_ts = PyEval_SaveThread();
    }

    void finalizePy() {
        if (!Py_IsInitialized()) return;
        PyEval_RestoreThread(main_ts);
        for (auto& [_, m] : modules) Py_XDECREF(m);
        modules.clear();
        Py_XDECREF(shim);
        shim = nullptr;
        Py_Finalize();
    }

    // Caller must hold GIL.
    void ensureShim() {
        if (shim) return;
        PyObject* code = Py_CompileString(kShim, "<ll_shim>", Py_file_input);
        if (!code) {
            PyErr_Print();
            return;
        }
        shim = PyImport_ExecCodeModule("_ll_shim", code);
        Py_DECREF(code);
        if (!shim) {
            PyErr_Print();
        }
    }
};

PythonRuntime& PythonRuntime::instance() {
    static PythonRuntime rt;
    return rt;
}

PythonRuntime::PythonRuntime() : impl_(std::make_unique<Impl>()) {
    impl_->initPy();
    spdlog::info("Python 3.11 runtime initialized");
}

PythonRuntime::~PythonRuntime() {
    if (impl_) impl_->finalizePy();
}

bool PythonRuntime::loadScript(int pipeline_index, const std::string& path) {
    std::lock_guard<std::mutex> lk(mu_);

    PyGILState_STATE gil = PyGILState_Ensure();
    impl_->ensureShim();

    // Read the source file manually so we can pass a friendly module name.
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) {
        impl_->last_error = "Unable to open " + path;
        PyGILState_Release(gil);
        return false;
    }
    if (std::fseek(f, 0, SEEK_END) != 0) {
        std::fclose(f);
        impl_->last_error = "seek failed: " + path;
        PyGILState_Release(gil);
        return false;
    }
    long sz = std::ftell(f);
    if (sz < 0 || sz > (64L * 1024L * 1024L)) {
        std::fclose(f);
        impl_->last_error = "bad size for " + path;
        PyGILState_Release(gil);
        return false;
    }
    std::fseek(f, 0, SEEK_SET);
    std::string src(static_cast<size_t>(sz), '\0');
    size_t got = std::fread(src.data(), 1, static_cast<size_t>(sz), f);
    std::fclose(f);
    if (got != static_cast<size_t>(sz)) {
        impl_->last_error = "short read from " + path;
        PyGILState_Release(gil);
        return false;
    }

    PyObject* code = Py_CompileString(src.c_str(), path.c_str(), Py_file_input);
    if (!code) {
        PyErr_Print();
        impl_->last_error = "Compile error in " + path;
        PyGILState_Release(gil);
        return false;
    }
    const std::string mod_name = "ll_pipeline_" + std::to_string(pipeline_index);
    PyObject* new_mod = PyImport_ExecCodeModule(mod_name.c_str(), code);
    Py_DECREF(code);
    if (!new_mod) {
        PyErr_Print();
        impl_->last_error = "Module load failed: " + path;
        PyGILState_Release(gil);
        return false;
    }
    auto it = impl_->modules.find(pipeline_index);
    if (it != impl_->modules.end()) {
        Py_DECREF(it->second);
    }
    impl_->modules[pipeline_index] = new_mod;
    impl_->last_error.clear();
    PyGILState_Release(gil);
    spdlog::info("Loaded Python pipeline {} from {}", pipeline_index, path);
    return true;
}

void PythonRuntime::unloadScript(int pipeline_index) {
    std::lock_guard<std::mutex> lk(mu_);
    PyGILState_STATE gil = PyGILState_Ensure();
    auto it = impl_->modules.find(pipeline_index);
    if (it != impl_->modules.end()) {
        Py_DECREF(it->second);
        impl_->modules.erase(it);
    }
    PyGILState_Release(gil);
}

PythonCallResult PythonRuntime::runPipeline(int pipeline_index,
                                            const cv::Mat& image,
                                            const std::array<double, 8>& llrobot) {
    PythonCallResult out;
    if (image.empty()) return out;

    std::lock_guard<std::mutex> lk(mu_);
    auto it = impl_->modules.find(pipeline_index);
    if (it == impl_->modules.end()) {
        out.error = "No module loaded";
        return out;
    }

    PyGILState_STATE gil = PyGILState_Ensure();

    // Build the bytes -> call_run(mod, data, h, w, c, llrobot) invocation.
    const int h = image.rows;
    const int w = image.cols;
    const int c = image.channels();
    const size_t total_bytes = image.total() * image.elemSize();

    PyObject* py_bytes = PyBytes_FromStringAndSize(
        reinterpret_cast<const char*>(image.data),
        static_cast<Py_ssize_t>(total_bytes));
    PyObject* py_llrobot = PyList_New(llrobot.size());
    for (size_t i = 0; i < llrobot.size(); ++i) {
        PyList_SetItem(py_llrobot, i, PyFloat_FromDouble(llrobot[i]));
    }

    PyObject* call_run = PyObject_GetAttrString(impl_->shim, "call_run");
    if (!call_run) {
        PyErr_Print();
        out.error = "_ll_shim.call_run missing";
        Py_DECREF(py_bytes);
        Py_DECREF(py_llrobot);
        PyGILState_Release(gil);
        return out;
    }

    PyObject* py_h = PyLong_FromLong(h);
    PyObject* py_w = PyLong_FromLong(w);
    PyObject* py_c = PyLong_FromLong(c);
    PyObject* args = PyTuple_Pack(6,
        it->second,
        py_bytes,
        py_h,
        py_w,
        py_c,
        py_llrobot);
    PyObject* res = PyObject_CallObject(call_run, args);
    Py_DECREF(call_run);
    Py_DECREF(args);
    Py_DECREF(py_bytes);
    Py_DECREF(py_llrobot);
    Py_XDECREF(py_h);
    Py_XDECREF(py_w);
    Py_XDECREF(py_c);

    if (!res) {
        PyErr_Print();
        out.error = "runPipeline raised";
        PyGILState_Release(gil);
        return out;
    }

    // res = (out_bytes, oh, ow, oc, llp, contour_flat)
    PyObject* py_out = PyTuple_GetItem(res, 0);
    const int oh = static_cast<int>(PyLong_AsLong(PyTuple_GetItem(res, 1)));
    const int ow = static_cast<int>(PyLong_AsLong(PyTuple_GetItem(res, 2)));
    const int oc = static_cast<int>(PyLong_AsLong(PyTuple_GetItem(res, 3)));
    PyObject* py_llp     = PyTuple_GetItem(res, 4);
    PyObject* py_contour = PyTuple_GetItem(res, 5);

    if (oh > 0 && ow > 0 && oc > 0 && py_out && PyBytes_Check(py_out)) {
        const int type = (oc == 1) ? CV_8UC1 : (oc == 3 ? CV_8UC3 : CV_8UC4);
        out.annotated = cv::Mat(oh, ow, type).clone();
        std::memcpy(out.annotated.data, PyBytes_AsString(py_out),
                    static_cast<size_t>(PyBytes_Size(py_out)));
    }

    if (py_llp && PyList_Check(py_llp)) {
        const Py_ssize_t n = PyList_Size(py_llp);
        for (Py_ssize_t i = 0; i < n && i < 8; ++i) {
            out.llpython[i] = PyFloat_AsDouble(PyList_GetItem(py_llp, i));
        }
    }
    if (py_contour && PyList_Check(py_contour)) {
        const Py_ssize_t n = PyList_Size(py_contour);
        for (Py_ssize_t i = 0; i < n; ++i) {
            PyObject* pt = PyList_GetItem(py_contour, i);
            if (!pt || !PyList_Check(pt) || PyList_Size(pt) < 2) continue;
            out.contour.push_back({
                PyFloat_AsDouble(PyList_GetItem(pt, 0)),
                PyFloat_AsDouble(PyList_GetItem(pt, 1)),
            });
        }
    }

    Py_DECREF(res);
    PyGILState_Release(gil);
    out.ok = true;
    return out;
}

std::string PythonRuntime::lastError() const {
    std::lock_guard<std::mutex> lk(mu_);
    return impl_->last_error;
}

} // namespace limelight
