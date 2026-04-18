#pragma once

// Fiducial pipeline — AprilTag detection and 3D pose estimation.
//
// The original firmware links libapriltag.so.3 (confirmed in raw_strings.txt)
// and uses these C entry points:
//   apriltag_detector_create / destroy
//   apriltag_detector_detect
//   apriltag_detector_add_family_bits
//   tag36h11_create / destroy
// For 3D pose it uses cv::solvePnP (multiple solver method variants are
// referenced by mangled name in .rodata).
//
// This recreation uses the same libapriltag C API. The field map lookup
// (to report botpose in field space) goes through limelight::FieldMap
// which parses pipe*/fmap.fmap.

#include "visionserver/pipeline.h"
#include "field_map.h"

#include <array>
#include <atomic>

// Forward-decls for libapriltag so we don't drag apriltag.h into the header.
struct apriltag_detector;
struct apriltag_family;

namespace limelight {

// IMU fusion modes for MegaTag2.  Semantics match the Limelight docs:
//   0 -- external only (robot_orientation from NT, ignore on-board IMU)
//   1 -- internal only (on-board IMU)
//   2 -- external with internal fallback
//   3 -- internal seeded by external on init / on tare
//   4 -- external blended with internal (alpha = imu_assist_alpha)
constexpr int kImuModeExternal        = 0;
constexpr int kImuModeInternal        = 1;
constexpr int kImuModeExternalFallback = 2;
constexpr int kImuModeInternalSeeded  = 3;
constexpr int kImuModeAssistBlend     = 4;

class FiducialPipeline final : public Pipeline {
public:
    FiducialPipeline();
    ~FiducialPipeline() override;

    FiducialPipeline(const FiducialPipeline&) = delete;
    FiducialPipeline& operator=(const FiducialPipeline&) = delete;

    std::string getType() const override;
    void configure(const PipelineConfig& config) override;
    PipelineResult process(const cv::Mat& frame) override;

    // Inject the active field map. PipelineManager calls this whenever the
    // pipeline index becomes active.
    void setFieldMap(const FieldMap& fmap) override;

    // MegaTag2 inputs.
    void setImu(const hal::Imu* imu) override;
    void setRobotOrientation(const std::array<double, 6>& orient) override;
    void setImuMode(int mode) override;
    void setImuAssistAlpha(double alpha) override;

private:
    void rebuildDetector();

    // Returns the fused yaw in radians for the current frame based on
    // imu_mode_.  out_valid is set to false when no yaw source is available
    // (in which case the caller should skip the tier-A solve).
    double fusedYawRad(bool& out_valid) const;

    apriltag_detector* detector_ = nullptr;
    apriltag_family*   family_   = nullptr;

    PipelineConfig cfg_{};
    FieldMap field_map_{};

    // Cached camera intrinsics for solvePnP. These come from calibration
    // files; when none are loaded we fall back to a pinhole model derived
    // from the frame resolution.
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // MegaTag2 state.  Written from NT callback threads / startup, read from
    // the vision thread in process().  Using atomics keeps this lockfree.
    const hal::Imu*      imu_ = nullptr;  // owned elsewhere, nullable
    std::atomic<double>  robot_yaw_rad_{0.0};
    std::atomic<bool>    robot_yaw_valid_{false};
    std::atomic<int>     imu_mode_{kImuModeExternal};
    std::atomic<double>  imu_assist_alpha_{1.0};
};

} // namespace limelight
