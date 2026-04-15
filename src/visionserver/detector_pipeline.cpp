// Neural detector pipeline.
//
// Thin wrapper over hal::InferenceEngine::detectObjects. The detector model
// and label file live under extern/pipeN/ (names picked up by whatever
// upload-nn / upload-nnlabels handler stored them). The runtime backend is
// chosen by `detector_runtime` in the .vpr config:
//   "cpu"     -> TFLITE_CPU
//   "hailo"   -> HAILO
//   "rknn"    -> RKNN
//   "edgetpu" -> EDGETPU
//
// For each detection above detector_conf, we push a bounding-box corner
// quad into result.corners and report the highest-score box via
// tx/ty/ta/tid/tclass.

#include "visionserver/detector_pipeline.h"

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <filesystem>
#include <fstream>

namespace limelight {

namespace {

hal::InferenceBackend parseBackend(const std::string& s) {
    if (s == "hailo")   return hal::InferenceBackend::HAILO;
    if (s == "rknn")    return hal::InferenceBackend::RKNN;
    if (s == "edgetpu") return hal::InferenceBackend::EDGETPU;
    return hal::InferenceBackend::TFLITE_CPU;
}

std::string findModelFile(int idx, const char* ext) {
    namespace fs = std::filesystem;
    const fs::path dir = fs::path("extern") / ("pipe" + std::to_string(idx));
    if (!fs::exists(dir)) return "";
    for (const auto& e : fs::directory_iterator(dir)) {
        if (e.is_regular_file() && e.path().extension() == ext) {
            return e.path().string();
        }
    }
    return "";
}

std::vector<std::string> loadLabels(int idx) {
    namespace fs = std::filesystem;
    const fs::path dir = fs::path("extern") / ("pipe" + std::to_string(idx));
    for (const auto& e : fs::directory_iterator(dir)) {
        const auto name = e.path().filename().string();
        if (name.find("labels") != std::string::npos) {
            std::ifstream in(e.path());
            std::vector<std::string> out;
            std::string line;
            while (std::getline(in, line)) {
                if (!line.empty() && line.back() == '\r') line.pop_back();
                if (!line.empty()) out.push_back(line);
            }
            return out;
        }
    }
    return {};
}

}  // namespace

DetectorPipeline::DetectorPipeline() {
    spdlog::info("Detector pipeline constructed");
}

std::string DetectorPipeline::getType() const { return "pipe_neuraldetector"; }

void DetectorPipeline::configure(const PipelineConfig& config) {
    cfg_ = config;
    auto backend = parseBackend(cfg_.detector_runtime);
    engine_ = hal::InferenceEngine::create(backend);
    if (!engine_) {
        spdlog::warn("DetectorPipeline: no backend available for {}", cfg_.detector_runtime);
        return;
    }

    const std::string tfl = findModelFile(pipeline_index_, ".tflite");
    const std::string hef = findModelFile(pipeline_index_, ".hef");
    const std::string rknn = findModelFile(pipeline_index_, ".rknn");
    std::string model_path = tfl;
    if (backend == hal::InferenceBackend::HAILO && !hef.empty()) model_path = hef;
    if (backend == hal::InferenceBackend::RKNN  && !rknn.empty()) model_path = rknn;
    if (model_path.empty()) {
        spdlog::warn("DetectorPipeline: no model file found under extern/pipe{}", pipeline_index_);
        return;
    }
    if (!engine_->loadModel(model_path)) {
        spdlog::error("DetectorPipeline: failed to load {}", model_path);
        engine_.reset();
        return;
    }
    labels_ = loadLabels(pipeline_index_);
    spdlog::info("DetectorPipeline: loaded {} ({} labels) backend={}",
                 model_path, labels_.size(), engine_->getBackendName());
}

PipelineResult DetectorPipeline::process(const cv::Mat& frame) {
    PipelineResult result;
    if (frame.empty()) return result;
    result.annotated_frame = frame.clone();
    if (!engine_) return result;

    auto detections = engine_->detectObjects(frame, static_cast<float>(cfg_.detector_conf), 0.45f);
    if (detections.empty()) return result;

    auto best_it = std::max_element(
        detections.begin(), detections.end(),
        [](const hal::BoundingBox& a, const hal::BoundingBox& b) {
            return a.confidence < b.confidence;
        });
    const auto& best = *best_it;

    result.has_target = true;
    result.tid = best.class_id;
    if (best.class_id >= 0 && static_cast<size_t>(best.class_id) < labels_.size()) {
        result.tclass = labels_[best.class_id];
    }

    // tx/ty from bbox center.
    const double cx = best.x + best.w / 2.0;
    const double cy = best.y + best.h / 2.0;
    result.tx = ((cx - frame.cols / 2.0) / frame.cols) * 60.0;
    result.ty = ((frame.rows / 2.0 - cy) / frame.rows) * 45.0;
    result.ta = (best.w * best.h) / (frame.cols * frame.rows) * 100.0;

    for (const auto& d : detections) {
        result.corners.push_back({ static_cast<double>(d.x),         static_cast<double>(d.y) });
        result.corners.push_back({ static_cast<double>(d.x + d.w),   static_cast<double>(d.y) });
        result.corners.push_back({ static_cast<double>(d.x + d.w),   static_cast<double>(d.y + d.h) });
        result.corners.push_back({ static_cast<double>(d.x),         static_cast<double>(d.y + d.h) });

        cv::rectangle(result.annotated_frame,
                      cv::Rect(cv::Point(static_cast<int>(d.x), static_cast<int>(d.y)),
                               cv::Size(static_cast<int>(d.w), static_cast<int>(d.h))),
                      cv::Scalar(0, 255, 0), 2);
        std::string label = (d.class_id >= 0 &&
                             static_cast<size_t>(d.class_id) < labels_.size())
                                ? labels_[d.class_id]
                                : std::to_string(d.class_id);
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s %.2f", label.c_str(), d.confidence);
        cv::putText(result.annotated_frame, buf,
                    { static_cast<int>(d.x), static_cast<int>(d.y) - 6 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    return result;
}

} // namespace limelight
