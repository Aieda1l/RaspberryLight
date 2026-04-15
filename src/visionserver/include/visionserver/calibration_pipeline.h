#pragma once

// Calibration pipeline — camera calibration using a ChArUco board.
//
// The original firmware calls:
//   cv::aruco::CharucoDetector::detectBoard()  (confirmed by mangled name
//   _ZNK2cv5aruco15CharucoDetector11detectBoard...)
//   cv::calibrateCamera()
// It also has a separate "focus" viewfinder mode; we handle that via
// ViewfinderPipeline.
//
// This pipeline accumulates detected ChArUco corners in memory and writes
// the calibration result to /var/www/html/calibration/ on completion.

#include "visionserver/pipeline.h"

#include <opencv2/core.hpp>
#include <memory>
#include <mutex>
#include <vector>

namespace cv { namespace aruco {
    class CharucoBoard;
    class CharucoDetector;
}}

namespace limelight {

class CalibrationPipeline final : public Pipeline {
public:
    CalibrationPipeline();
    ~CalibrationPipeline() override;

    std::string getType() const override;
    void configure(const PipelineConfig& config) override;
    PipelineResult process(const cv::Mat& frame) override;

    // Web UI action hooks (REST: /cal-default, /cal-file, ...).
    void startCalibration();
    void resetCalibration();
    bool saveCalibration(const std::string& out_path);

private:
    PipelineConfig cfg_{};

    // Accumulated corner observations.
    std::vector<std::vector<cv::Point2f>> all_corners_;
    std::vector<std::vector<int>>         all_ids_;
    cv::Size image_size_{0, 0};

    std::shared_ptr<cv::aruco::CharucoBoard>    board_;
    std::shared_ptr<cv::aruco::CharucoDetector> detector_;

    std::mutex mu_;
    bool running_ = false;
    int  total_captured_ = 0;
};

} // namespace limelight
