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

// Forward-decls for libapriltag so we don't drag apriltag.h into the header.
struct apriltag_detector;
struct apriltag_family;

namespace limelight {

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

private:
    void rebuildDetector();

    apriltag_detector* detector_ = nullptr;
    apriltag_family*   family_   = nullptr;

    PipelineConfig cfg_{};
    FieldMap field_map_{};

    // Cached camera intrinsics for solvePnP. These come from calibration
    // files; when none are loaded we fall back to a pinhole model derived
    // from the frame resolution.
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};

} // namespace limelight
