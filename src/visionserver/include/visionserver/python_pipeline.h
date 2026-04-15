#pragma once

// Python pipeline — runs a user-authored Python script per frame.
//
// The original firmware embeds CPython 3.11 (`Py_Initialize`, `runPipeline`
// are both referenced in raw_strings.txt). Each active Python pipeline loads
// `extern/pipeN/python/pythonScript.py` and calls `runPipeline(image, llrobot)`
// which returns `(contour, image, llpython[8])`.
//
// The embedded runtime lives in python_runtime.cpp; this class is a thin
// wrapper that owns one interpreter sub-thread state.

#include "visionserver/pipeline.h"

#include <memory>
#include <string>

namespace limelight {

class PythonRuntime;  // python_runtime.h

class PythonPipeline final : public Pipeline {
public:
    PythonPipeline();
    ~PythonPipeline() override;

    PythonPipeline(const PythonPipeline&) = delete;
    PythonPipeline& operator=(const PythonPipeline&) = delete;

    std::string getType() const override;
    void configure(const PipelineConfig& config) override;
    PipelineResult process(const cv::Mat& frame) override;

    void setSlotIndex(int idx) override { pipeline_index_ = idx; }
    void setRobotInputs(const double robot[8]);

private:
    PipelineConfig cfg_{};
    int pipeline_index_ = 0;
    std::unique_ptr<PythonRuntime> rt_;
    double robot_inputs_[8] = {};
};

} // namespace limelight
