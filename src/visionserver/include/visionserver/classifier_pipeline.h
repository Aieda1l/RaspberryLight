#pragma once

// Neural classifier pipeline — image classification using the same
// InferenceEngine HAL as DetectorPipeline.

#include "visionserver/pipeline.h"
#include "hal/npu.h"

#include <memory>
#include <string>
#include <vector>

namespace limelight {

class ClassifierPipeline final : public Pipeline {
public:
    ClassifierPipeline();

    std::string getType() const override;
    void configure(const PipelineConfig& config) override;
    PipelineResult process(const cv::Mat& frame) override;

    void setSlotIndex(int idx) override { pipeline_index_ = idx; }

private:
    PipelineConfig cfg_{};
    int pipeline_index_ = 0;
    std::unique_ptr<hal::InferenceEngine> engine_;
    std::vector<std::string> labels_;
};

} // namespace limelight
