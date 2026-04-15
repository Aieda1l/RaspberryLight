#pragma once

// Neural detector pipeline — generic object detector.
//
// Wraps hal::InferenceEngine so the underlying backend (TFLite CPU, RKNN,
// Hailo, EdgeTPU) is interchangeable. Model file and label file are
// resolved from extern/pipeN/ at configure time.

#include "visionserver/pipeline.h"
#include "hal/npu.h"

#include <memory>
#include <string>
#include <vector>

namespace limelight {

class DetectorPipeline final : public Pipeline {
public:
    DetectorPipeline();

    std::string getType() const override;
    void configure(const PipelineConfig& config) override;
    PipelineResult process(const cv::Mat& frame) override;

    // PipelineManager calls this to set the pipeline index so we know
    // where to look for extern/pipeN/*.tflite.
    void setSlotIndex(int idx) override { pipeline_index_ = idx; }

private:
    PipelineConfig cfg_{};
    int pipeline_index_ = 0;
    std::unique_ptr<hal::InferenceEngine> engine_;
    std::vector<std::string> labels_;
};

} // namespace limelight
