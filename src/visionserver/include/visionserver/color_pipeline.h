#pragma once

// Color pipeline — HSV threshold + contour detection.
//
// Closest match to the original:
//   - cv::cvtColor(BGR -> HSV)
//   - cv::inRange using (hue_min..hue_max, sat_min..sat_max, val_min..val_max)
//   - morphology (erode/dilate, configurable steps from PipelineConfig)
//   - cv::findContours + filter by area / aspect / convexity
//   - compute tx/ty/ta using the configured crosshair as reference
//
// The binary exports `hue_min`, `sat_min`, `val_min`, `area_min`, etc. as
// strings in .rodata — see `visionserver_strings.txt`.

#include "visionserver/pipeline.h"

namespace limelight {

class ColorPipeline final : public Pipeline {
public:
    ColorPipeline();

    std::string getType() const override;
    void configure(const PipelineConfig& config) override;
    PipelineResult process(const cv::Mat& frame) override;

private:
    PipelineConfig cfg_{};
};

} // namespace limelight
