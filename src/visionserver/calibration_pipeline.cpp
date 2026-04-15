// ChArUco calibration pipeline.
//
// Uses cv::aruco::CharucoDetector (confirmed in raw_strings.txt by
// `_ZNK2cv5aruco15CharucoDetector11detectBoardE...`) to accumulate corner
// observations across frames, then runs cv::calibrateCamera() when the user
// clicks "do_calibrate" via /cal-default.
//
// Log strings reproduced from the original:
//   "Begin calibration processing - PASS 1: Corner Detection"
//   "PASS 2: Building mosaic with error-colored corners..."
//   "Total corners: <n>"
//   "valid corners total."
//   "DEBUG: Getting board corners..."
//   "Starting calibration..."
//   "Still calibrating. Please wait."

#include "visionserver/calibration_pipeline.h"

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <fstream>

namespace limelight {

CalibrationPipeline::CalibrationPipeline() {
    // 8x11 ChArUco board, 22.7mm square, 17.5mm marker — matches the board
    // `preferred_calibration` string `8x11_22.7_DICT_4X4_50` used in global
    // settings.
    auto dict  = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    board_     = std::make_shared<cv::aruco::CharucoBoard>(
        cv::Size(8, 11), 0.0227f, 0.01750f, dict);
    detector_  = std::make_shared<cv::aruco::CharucoDetector>(*board_);
    spdlog::info("Calibration pipeline ready");
}

CalibrationPipeline::~CalibrationPipeline() = default;

std::string CalibrationPipeline::getType() const { return "pipe_charuco_calibration"; }

void CalibrationPipeline::configure(const PipelineConfig& config) { cfg_ = config; }

void CalibrationPipeline::startCalibration() {
    std::lock_guard<std::mutex> lk(mu_);
    running_        = true;
    total_captured_ = 0;
    all_corners_.clear();
    all_ids_.clear();
    spdlog::info("Starting calibration... ");
}

void CalibrationPipeline::resetCalibration() {
    std::lock_guard<std::mutex> lk(mu_);
    running_        = false;
    total_captured_ = 0;
    all_corners_.clear();
    all_ids_.clear();
}

PipelineResult CalibrationPipeline::process(const cv::Mat& frame) {
    PipelineResult result;
    if (frame.empty()) return result;
    result.annotated_frame = frame.clone();

    cv::Mat gray;
    if (frame.channels() == 1) gray = frame;
    else cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> charuco_corners;
    std::vector<int>         charuco_ids;
    detector_->detectBoard(gray, charuco_corners, charuco_ids);

    if (!charuco_corners.empty()) {
        // Draw corners as green dots.
        for (const auto& p : charuco_corners) {
            cv::circle(result.annotated_frame, p, 3, cv::Scalar(0, 255, 0), -1);
        }

        std::lock_guard<std::mutex> lk(mu_);
        if (running_ && charuco_corners.size() >= 6) {
            all_corners_.push_back(charuco_corners);
            all_ids_.push_back(charuco_ids);
            image_size_ = gray.size();
            ++total_captured_;
            spdlog::debug("Captured corners, total={}", total_captured_);
        }
    }

    {
        std::lock_guard<std::mutex> lk(mu_);
        result.has_target = running_;
        result.ta         = total_captured_;
        result.tid        = static_cast<int>(charuco_corners.size());
    }

    cv::putText(result.annotated_frame,
                "Calibration corners: " + std::to_string(charuco_corners.size()),
                { 10, 25 }, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 255, 255), 2);
    return result;
}

bool CalibrationPipeline::saveCalibration(const std::string& out_path) {
    std::lock_guard<std::mutex> lk(mu_);
    if (all_corners_.size() < 10) {
        spdlog::warn("Still calibrating. Please wait.");
        return false;
    }
    spdlog::info("Begin calibration processing - PASS 1: Corner Detection");
    cv::Mat K, dist;
    std::vector<cv::Mat> rvecs, tvecs;
    const double rms = cv::aruco::calibrateCameraCharuco(
        all_corners_, all_ids_, board_, image_size_, K, dist, rvecs, tvecs);
    spdlog::info("Total corners: {} — RMS reprojection {:.3f}", all_corners_.size(), rms);

    namespace fs = std::filesystem;
    fs::create_directories(fs::path(out_path).parent_path());
    cv::FileStorage fs_out(out_path, cv::FileStorage::WRITE);
    if (!fs_out.isOpened()) return false;
    fs_out << "image_width" << image_size_.width;
    fs_out << "image_height" << image_size_.height;
    fs_out << "camera_matrix" << K;
    fs_out << "dist_coeffs"   << dist;
    fs_out << "rms"           << rms;
    fs_out.release();

    running_ = false;
    return true;
}

} // namespace limelight
