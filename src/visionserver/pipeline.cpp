// Pipeline base + factory.
//
// The JSON envelope emitted by PipelineResult::to_json() matches the shape
// the web UI expects over WebSocket / REST. Keys derive from the NetworkTables
// topic list discovered in .ghidra_work/output/visionserver/raw_strings.txt
// (tv, tx, ty, ta, ts, tid, tclass, botpose, botpose_wpiblue, botpose_wpired,
// targetpose_robotspace, targetpose_cameraspace, tcornxy, pythonOut).

#include "visionserver/pipeline.h"

#include "visionserver/barcode_pipeline.h"
#include "visionserver/calibration_pipeline.h"
#include "visionserver/classifier_pipeline.h"
#include "visionserver/color_pipeline.h"
#include "visionserver/depth_pipeline.h"
#include "visionserver/detector_pipeline.h"
#include "visionserver/fiducial_pipeline.h"
#include "visionserver/python_pipeline.h"
#include "visionserver/viewfinder_pipeline.h"

#include <nlohmann/json.hpp>
#include <stdexcept>

namespace limelight {

std::string PipelineResult::to_json() const {
    nlohmann::json j;
    j["tv"] = has_target ? 1 : 0;
    j["tx"] = tx;
    j["ty"] = ty;
    j["ta"] = ta;
    j["ts"] = ts;
    j["tid"] = tid;
    j["tclass"] = tclass;
    j["botpose"] = botpose;
    j["botpose_wpiblue"] = botpose_blue;
    j["botpose_wpired"] = botpose_red;
    j["targetpose_robotspace"] = targetpose_robotspace;
    j["targetpose_cameraspace"] = targetpose_cameraspace;

    // Flatten corners into a single [x0,y0,x1,y1,...] array like the
    // NT "tcornxy" topic.
    std::vector<double> flat;
    flat.reserve(corners.size() * 2);
    for (const auto& c : corners) {
        for (double v : c) flat.push_back(v);
    }
    j["tcornxy"] = flat;

    nlohmann::json py = nlohmann::json::array();
    for (double v : llpython) py.push_back(v);
    j["pythonOut"] = py;

    return j.dump();
}

std::unique_ptr<Pipeline> Pipeline::create(const std::string& pipeline_type) {
    if (pipeline_type == "pipe_fiducial") {
        return std::make_unique<FiducialPipeline>();
    }
    if (pipeline_type == "pipe_color" || pipeline_type == "pipe_colorsensor") {
        return std::make_unique<ColorPipeline>();
    }
    if (pipeline_type == "pipe_neuraldetector") {
        return std::make_unique<DetectorPipeline>();
    }
    if (pipeline_type == "pipe_neuralclassifier") {
        return std::make_unique<ClassifierPipeline>();
    }
    if (pipeline_type == "pipe_neuraldepth") {
        return std::make_unique<DepthPipeline>();
    }
    if (pipeline_type == "pipe_python" || pipeline_type == "pipe_pythonpro") {
        return std::make_unique<PythonPipeline>();
    }
    if (pipeline_type == "pipe_barcode") {
        return std::make_unique<BarcodePipeline>();
    }
    if (pipeline_type == "pipe_viewfinder" || pipeline_type == "pipe_focus") {
        return std::make_unique<ViewfinderPipeline>(pipeline_type == "pipe_focus");
    }
    if (pipeline_type == "pipe_charuco_calibration") {
        return std::make_unique<CalibrationPipeline>();
    }
    throw std::runtime_error("Pipeline type not supported: " + pipeline_type);
}

} // namespace limelight
