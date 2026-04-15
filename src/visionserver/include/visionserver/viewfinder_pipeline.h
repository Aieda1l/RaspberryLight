#pragma once

// Viewfinder pipeline — raw camera passthrough used for aiming the camera
// without any vision processing. The original firmware also reuses this
// class (under a different name) for the "focus" pipeline type; the only
// difference is that focus draws a focus-assist overlay on the annotated
// frame. Log strings confirmed in .ghidra_work/output/visionserver/
// raw_strings.txt:
//   "Viewfinder setup complete", "Viewfinder size chosen is "

#include "visionserver/pipeline.h"

namespace limelight {

class ViewfinderPipeline final : public Pipeline {
public:
    explicit ViewfinderPipeline(bool focus_mode = false);

    std::string getType() const override;
    void configure(const PipelineConfig& config) override;
    PipelineResult process(const cv::Mat& frame) override;

private:
    bool focus_mode_ = false;
    PipelineConfig cfg_{};
};

} // namespace limelight
