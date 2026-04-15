#pragma once

#include "pipeline_config.h"
#include "field_map.h"
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include <memory>

namespace limelight {

// Per-frame result from a pipeline
struct PipelineResult {
    // Target data
    bool has_target = false;
    double tx = 0.0;   // Horizontal offset from crosshair (degrees)
    double ty = 0.0;   // Vertical offset from crosshair (degrees)
    double ta = 0.0;   // Target area (% of image)
    double ts = 0.0;   // Target skew (-90 to 0 degrees)
    int tid = -1;       // Tag/class ID

    // AprilTag 3D pose
    std::vector<double> botpose;       // Robot pose in field space [x,y,z,rx,ry,rz]
    std::vector<double> botpose_blue;  // Blue alliance origin
    std::vector<double> botpose_red;   // Red alliance origin
    std::vector<double> targetpose_robotspace;
    std::vector<double> targetpose_cameraspace;

    // Detector/classifier
    std::string tclass;
    std::vector<std::vector<double>> corners;

    // Python pipeline custom output
    double llpython[8] = {};

    // Annotated frame for streaming
    cv::Mat annotated_frame;

    // JSON representation for NT/Modbus
    std::string to_json() const;
};

// Base pipeline interface
class Pipeline {
public:
    virtual ~Pipeline() = default;

    virtual std::string getType() const = 0;
    virtual void configure(const PipelineConfig& config) = 0;
    virtual PipelineResult process(const cv::Mat& frame) = 0;

    // Called when pipeline becomes active/inactive
    virtual void onActivate() {}
    virtual void onDeactivate() {}

    // Context injection. PipelineManager calls this before configure() so
    // the implementation knows which slot it lives in (for extern/pipeN/
    // resource lookup) and which field map to use (for 3D pose).
    virtual void setSlotIndex(int /*index*/) {}
    virtual void setFieldMap(const FieldMap& /*fmap*/) {}

    // Factory
    static std::unique_ptr<Pipeline> create(const std::string& pipeline_type);
};

} // namespace limelight
