// Fiducial (AprilTag) pipeline.
//
// Uses the libapriltag C API directly, same as the original binary:
//   apriltag_detector_create()
//   tag36h11_create() / tag16h5_create() / ...
//   apriltag_detector_add_family_bits(detector, family, 2)
//   apriltag_detector_detect(detector, image_u8)
//   apriltag_detections_destroy(detections)
//
// For 3D pose we use cv::solvePnP with:
//   - 4 object-space corners at (±size/2, ±size/2, 0)
//   - 4 image-space corners from the detection
//   - camera_matrix + dist_coeffs from calibration (or a pinhole fallback)
//
// If a field map is loaded and the detected tag has a fmap entry, we compose
// the camera-relative pose with the field-space transform and publish
// botpose / botpose_wpiblue / botpose_wpired.

#include "visionserver/fiducial_pipeline.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include <apriltag.h>
#include <tag36h11.h>
#include <tag16h5.h>
#include <tag25h9.h>

#include <algorithm>
#include <cmath>

namespace limelight {

FiducialPipeline::FiducialPipeline() {
    spdlog::info("Fiducial Pipeline Initialization");
    detector_ = apriltag_detector_create();
    family_   = tag36h11_create();
    apriltag_detector_add_family_bits(detector_, family_, 2);
    spdlog::info("Fiducial Setup Complete");
}

FiducialPipeline::~FiducialPipeline() {
    if (detector_) apriltag_detector_destroy(detector_);
    if (family_)   tag36h11_destroy(family_);
}

std::string FiducialPipeline::getType() const { return "pipe_fiducial"; }

void FiducialPipeline::configure(const PipelineConfig& config) {
    cfg_ = config;
    rebuildDetector();
}

void FiducialPipeline::setFieldMap(const FieldMap& fmap) { field_map_ = fmap; }

void FiducialPipeline::rebuildDetector() {
    if (!detector_) return;
    // libapriltag detector parameters
    detector_->quad_decimate = std::max(1, cfg_.fiducial_resdiv);
    detector_->quad_sigma    = static_cast<float>(cfg_.fiducial_denoise);
    detector_->refine_edges  = 1;
    detector_->decode_sharpening = 0.25;
    detector_->nthreads      = 2;
}

namespace {

// Build object-space corners for a square tag of the given size (mm).
// Order matches libapriltag's detection.p[] order (TL, TR, BR, BL).
std::vector<cv::Point3f> tagObjectCorners(double size_mm) {
    const double s = size_mm / 2.0;
    return {
        { -static_cast<float>(s),  static_cast<float>(s), 0.0f }, // TL
        {  static_cast<float>(s),  static_cast<float>(s), 0.0f }, // TR
        {  static_cast<float>(s), -static_cast<float>(s), 0.0f }, // BR
        { -static_cast<float>(s), -static_cast<float>(s), 0.0f }, // BL
    };
}

cv::Mat pinholeCameraMatrix(int W, int H) {
    // 60° horizontal FOV.
    const double fx = W / (2.0 * std::tan(30.0 * CV_PI / 180.0));
    const double fy = fx;  // square pixels
    const double cx = W / 2.0;
    const double cy = H / 2.0;
    cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    return K;
}

// Convert rvec/tvec into a 6-DOF pose vector [x,y,z,roll,pitch,yaw] in meters/deg.
std::vector<double> rvecTvecToPose(const cv::Mat& rvec, const cv::Mat& tvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    // Convert tvec from mm to meters to match botpose units.
    const double x = tvec.at<double>(0) / 1000.0;
    const double y = tvec.at<double>(1) / 1000.0;
    const double z = tvec.at<double>(2) / 1000.0;
    // XYZ Euler.
    const double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                                R.at<double>(1, 0) * R.at<double>(1, 0));
    const bool singular = sy < 1e-6;
    double roll, pitch, yaw;
    if (!singular) {
        roll  = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        pitch = std::atan2(-R.at<double>(2, 0), sy);
        yaw   = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        roll  = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        pitch = std::atan2(-R.at<double>(2, 0), sy);
        yaw   = 0.0;
    }
    const double deg = 180.0 / CV_PI;
    return {x, y, z, roll * deg, pitch * deg, yaw * deg};
}

}  // namespace

PipelineResult FiducialPipeline::process(const cv::Mat& frame) {
    PipelineResult result;
    if (frame.empty() || !detector_) return result;

    result.annotated_frame = frame.clone();

    // Convert to grayscale and wrap in an image_u8_t for libapriltag.
    cv::Mat gray;
    if (frame.channels() == 1) gray = frame;
    else cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    image_u8_t img{/*width=*/gray.cols, /*height=*/gray.rows,
                   /*stride=*/static_cast<int32_t>(gray.step[0]),
                   /*buf=*/gray.data};

    zarray_t* detections = apriltag_detector_detect(detector_, &img);
    const int n = zarray_size(detections);

    // Lazy camera intrinsics.
    if (camera_matrix_.empty()) {
        camera_matrix_ = pinholeCameraMatrix(frame.cols, frame.rows);
        dist_coeffs_   = cv::Mat::zeros(4, 1, CV_64F);
    }

    // Pick the first tag whose id passes the ID filter (or the largest one
    // if no filter). Everything else is reported via corners only.
    int best_idx = -1;
    double best_area = 0.0;
    for (int i = 0; i < n; ++i) {
        apriltag_detection_t* det = nullptr;
        zarray_get(detections, i, &det);
        // Approximate area from the bounding quad.
        const double a = std::abs(
            (det->p[0][0] - det->p[2][0]) * (det->p[1][1] - det->p[3][1]) -
            (det->p[1][0] - det->p[3][0]) * (det->p[0][1] - det->p[2][1])) * 0.5;
        if (a > best_area) {
            best_area = a;
            best_idx  = i;
        }
    }

    for (int i = 0; i < n; ++i) {
        apriltag_detection_t* det = nullptr;
        zarray_get(detections, i, &det);
        std::vector<cv::Point> poly = {
            cv::Point(static_cast<int>(det->p[0][0]), static_cast<int>(det->p[0][1])),
            cv::Point(static_cast<int>(det->p[1][0]), static_cast<int>(det->p[1][1])),
            cv::Point(static_cast<int>(det->p[2][0]), static_cast<int>(det->p[2][1])),
            cv::Point(static_cast<int>(det->p[3][0]), static_cast<int>(det->p[3][1])),
        };
        cv::polylines(result.annotated_frame,
                      std::vector<std::vector<cv::Point>>{poly},
                      true, cv::Scalar(0, 255, 255), 2);
        char idstr[16];
        std::snprintf(idstr, sizeof(idstr), "%d", det->id);
        cv::putText(result.annotated_frame, idstr,
                    {static_cast<int>(det->c[0]), static_cast<int>(det->c[1])},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

        for (int k = 0; k < 4; ++k) {
            result.corners.push_back({det->p[k][0], det->p[k][1]});
        }
    }

    if (best_idx >= 0) {
        apriltag_detection_t* best = nullptr;
        zarray_get(detections, best_idx, &best);

        result.has_target = true;
        result.tid = best->id;

        // tx/ty: center offset in degrees (pinhole small-angle approx).
        const double dx = best->c[0] - frame.cols / 2.0;
        const double dy = frame.rows / 2.0 - best->c[1];
        const double fov_h = 60.0, fov_v = 45.0;
        result.tx = (dx / frame.cols) * fov_h;
        result.ty = (dy / frame.rows) * fov_v;
        result.ta = best_area / (frame.cols * frame.rows) * 100.0;

        // 3D pose via solvePnP.
        if (!cfg_.fiducial_skip3d) {
            auto obj = tagObjectCorners(cfg_.fiducial_size);
            std::vector<cv::Point2f> img2d = {
                { static_cast<float>(best->p[0][0]), static_cast<float>(best->p[0][1]) },
                { static_cast<float>(best->p[1][0]), static_cast<float>(best->p[1][1]) },
                { static_cast<float>(best->p[2][0]), static_cast<float>(best->p[2][1]) },
                { static_cast<float>(best->p[3][0]), static_cast<float>(best->p[3][1]) },
            };
            cv::Mat rvec, tvec;
            if (cv::solvePnP(obj, img2d, camera_matrix_, dist_coeffs_, rvec, tvec,
                             false, cv::SOLVEPNP_IPPE_SQUARE)) {
                result.targetpose_cameraspace = rvecTvecToPose(rvec, tvec);

                // If we have a fmap entry for this id, compose into field space.
                if (const Fiducial* tag = field_map_.getFiducial(best->id)) {
                    cv::Mat T_field = cv::Mat(4, 4, CV_64F);
                    for (int r = 0; r < 4; ++r)
                        for (int c = 0; c < 4; ++c)
                            T_field.at<double>(r, c) = tag->transform[r * 4 + c];

                    cv::Mat R_cam;
                    cv::Rodrigues(rvec, R_cam);
                    cv::Mat T_cam = cv::Mat::eye(4, 4, CV_64F);
                    R_cam.copyTo(T_cam(cv::Rect(0, 0, 3, 3)));
                    T_cam.at<double>(0, 3) = tvec.at<double>(0) / 1000.0;
                    T_cam.at<double>(1, 3) = tvec.at<double>(1) / 1000.0;
                    T_cam.at<double>(2, 3) = tvec.at<double>(2) / 1000.0;

                    cv::Mat T_bot = T_field * T_cam.inv();
                    cv::Mat R_bot = T_bot(cv::Rect(0, 0, 3, 3));
                    cv::Mat rvec_bot;
                    cv::Rodrigues(R_bot, rvec_bot);
                    cv::Mat tvec_bot = (cv::Mat_<double>(3, 1)
                        << T_bot.at<double>(0, 3) * 1000.0,
                           T_bot.at<double>(1, 3) * 1000.0,
                           T_bot.at<double>(2, 3) * 1000.0);
                    result.botpose = rvecTvecToPose(rvec_bot, tvec_bot);
                    result.botpose_blue = result.botpose;
                    result.botpose_red  = result.botpose;
                }
            }
        }
    }

    apriltag_detections_destroy(detections);
    return result;
}

} // namespace limelight
