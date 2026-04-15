#pragma once

// Barcode pipeline.
//
// The original firmware uses BOTH OpenCV's cv::QRCodeDetector / cv::barcode::
// BarcodeDetector AND ZXing::ReadBarcodes (all three constructors are
// exported in the visionserver binary; confirmed in raw_strings.txt).
// We prefer ZXing because it supports more symbologies and is faster on
// ARM, and fall back to the OpenCV detectors only if ZXing isn't available.

#include "visionserver/pipeline.h"

namespace limelight {

class BarcodePipeline final : public Pipeline {
public:
    BarcodePipeline();

    std::string getType() const override;
    void configure(const PipelineConfig& config) override;
    PipelineResult process(const cv::Mat& frame) override;

private:
    PipelineConfig cfg_{};
};

} // namespace limelight
