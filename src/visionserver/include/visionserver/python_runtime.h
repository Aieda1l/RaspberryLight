#pragma once

// Embedded Python 3.11 runtime for the Python pipeline(s).
//
// In the original binary, `Py_Initialize` is called exactly once from
// visionserver_main_boot (phase 7 "PYTHON") and the interpreter lives for
// the entire process lifetime. Each Python pipeline gets its own module
// object loaded from `extern/pipeN/python/*.py`, and `runPipeline(image,
// llrobot)` is invoked once per frame.
//
// We mirror that: PythonRuntime::instance() initializes CPython on first
// access, and each PythonRuntimeSession owns a loaded module + a GIL-safe
// call path.

#include <opencv2/core.hpp>
#include <array>
#include <memory>
#include <mutex>
#include <string>

namespace limelight {

struct PythonCallResult {
    bool ok = false;
    std::array<double, 8> llpython{};
    cv::Mat annotated;
    std::vector<std::vector<double>> contour; // optional polyline
    std::string error;
};

class PythonRuntime {
public:
    // Global singleton — initializes CPython on first call and deinitializes
    // it at process exit.
    static PythonRuntime& instance();

    PythonRuntime(const PythonRuntime&) = delete;
    PythonRuntime& operator=(const PythonRuntime&) = delete;

    // Load (or reload) a script from disk. Takes the full path to the .py
    // file. Returns false on error; use lastError() for details.
    bool loadScript(int pipeline_index, const std::string& path);
    void unloadScript(int pipeline_index);

    // Invoke `runPipeline(image, llrobot)` on the module previously loaded
    // into the given slot. Safe to call concurrently from one thread per
    // slot; internally acquires the GIL.
    PythonCallResult runPipeline(int pipeline_index,
                                 const cv::Mat& image,
                                 const std::array<double, 8>& llrobot);

    std::string lastError() const;

private:
    PythonRuntime();
    ~PythonRuntime();

    struct Impl;
    std::unique_ptr<Impl> impl_;
    mutable std::mutex mu_;
};

} // namespace limelight
