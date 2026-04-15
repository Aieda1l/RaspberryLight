// Python pipeline — per-slot script runner.
//
// Loads extern/pipeN/python/pythonScript.py (or whatever `python_snapscript_name`
// in the config points at) and invokes its `runPipeline(image, llrobot)` on
// every frame. The llpython array and optional annotated image are copied
// back into PipelineResult.

#include "visionserver/python_pipeline.h"
#include "visionserver/python_runtime.h"

#include <spdlog/spdlog.h>

#include <filesystem>

namespace limelight {

PythonPipeline::PythonPipeline() {
    // Ensure the global runtime is up. The ctor is cheap after the first call.
    (void)PythonRuntime::instance();
    spdlog::info("Python pipeline constructed");
}

PythonPipeline::~PythonPipeline() {
    PythonRuntime::instance().unloadScript(pipeline_index_);
}

std::string PythonPipeline::getType() const { return "pipe_python"; }

void PythonPipeline::configure(const PipelineConfig& config) {
    cfg_ = config;
    namespace fs = std::filesystem;
    fs::path dir = fs::path("extern") / ("pipe" + std::to_string(pipeline_index_)) / "python";

    // Default filename matches what the firmware ships (pythonScript1.py etc).
    std::string name = cfg_.python_snapscript_name;
    if (name.empty()) name = "pythonScript" + std::to_string(pipeline_index_) + ".py";
    fs::path script = dir / name;

    if (!fs::exists(script)) {
        // Fall back to the first .py in the directory.
        if (fs::exists(dir)) {
            for (const auto& e : fs::directory_iterator(dir)) {
                if (e.path().extension() == ".py") {
                    script = e.path();
                    break;
                }
            }
        }
    }
    if (!fs::exists(script)) {
        spdlog::warn("PythonPipeline: no script found for pipe{}", pipeline_index_);
        return;
    }

    if (!PythonRuntime::instance().loadScript(pipeline_index_, script.string())) {
        spdlog::error("PythonPipeline: {}", PythonRuntime::instance().lastError());
    }
}

void PythonPipeline::setRobotInputs(const double robot[8]) {
    for (int i = 0; i < 8; ++i) robot_inputs_[i] = robot[i];
}

PipelineResult PythonPipeline::process(const cv::Mat& frame) {
    PipelineResult result;
    if (frame.empty()) return result;

    std::array<double, 8> llrobot{};
    for (int i = 0; i < 8; ++i) llrobot[i] = robot_inputs_[i];

    auto py = PythonRuntime::instance().runPipeline(pipeline_index_, frame, llrobot);
    if (!py.ok) {
        // Fall back: return the raw frame so the stream server still has
        // something to push.
        result.annotated_frame = frame.clone();
        return result;
    }

    result.annotated_frame = py.annotated.empty() ? frame.clone() : py.annotated;
    for (int i = 0; i < 8; ++i) result.llpython[i] = py.llpython[i];
    result.corners = std::move(py.contour);
    // "Has target" is implied by the first llpython slot being non-zero,
    // matching what the original firmware's web UI inspects.
    result.has_target = std::abs(result.llpython[0]) > 1e-9;
    return result;
}

} // namespace limelight
