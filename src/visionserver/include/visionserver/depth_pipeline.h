#pragma once

// Neural depth pipeline — monocular depth estimation.
//
// The original firmware ships a Hailo HEF model at
// `defaults/depth/depth_hailo.hef`. We wrap it via InferenceEngine so the
// same pipeline works on TFLite CPU or RKNN if the user substitutes a
// different model file at extern/pipeN/.

#include "visionserver/pipeline.h"
#include "hal/npu.h"

#include <memory>

namespace limelight {

class DepthPipeline final : public Pipeline {
public:
    DepthPipeline();

    std::string getType() const override;
    void configure(const PipelineConfig& config) override;
    PipelineResult process(const cv::Mat& frame) override;

    void setSlotIndex(int idx) override { pipeline_index_ = idx; }

private:
    PipelineConfig cfg_{};
    int pipeline_index_ = 0;
    std::unique_ptr<hal::InferenceEngine> engine_;
};

} // namespace limelight
