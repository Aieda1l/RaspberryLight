// Neural classifier pipeline.
//
// Same shape as DetectorPipeline but routes through
// InferenceEngine::classify(). Top-k results are emitted via result.corners
// (no geometric corners, just the scores as y-values) and the best one
// goes to tid/tclass/ta.

#include "visionserver/classifier_pipeline.h"

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

std::string findModel(int idx, const char* ext) {
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
    if (!fs::exists(dir)) return {};
    for (const auto& e : fs::directory_iterator(dir)) {
        if (e.path().filename().string().find("labels") != std::string::npos) {
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

ClassifierPipeline::ClassifierPipeline() {
    spdlog::info("Classifier pipeline constructed");
}

std::string ClassifierPipeline::getType() const { return "pipe_neuralclassifier"; }

void ClassifierPipeline::configure(const PipelineConfig& config) {
    cfg_ = config;
    auto backend = parseBackend(cfg_.classifier_runtime);
    engine_ = hal::InferenceEngine::create(backend);
    if (!engine_) {
        spdlog::warn("ClassifierPipeline: no backend for {}", cfg_.classifier_runtime);
        return;
    }

    const std::string tfl  = findModel(pipeline_index_, ".tflite");
    const std::string hef  = findModel(pipeline_index_, ".hef");
    const std::string rknn = findModel(pipeline_index_, ".rknn");
    std::string model_path = tfl;
    if (backend == hal::InferenceBackend::HAILO && !hef.empty()) model_path = hef;
    if (backend == hal::InferenceBackend::RKNN  && !rknn.empty()) model_path = rknn;
    if (model_path.empty()) {
        spdlog::warn("ClassifierPipeline: no model file under extern/pipe{}", pipeline_index_);
        return;
    }
    if (!engine_->loadModel(model_path)) {
        spdlog::error("ClassifierPipeline: failed to load {}", model_path);
        engine_.reset();
        return;
    }
    labels_ = loadLabels(pipeline_index_);
    spdlog::info("ClassifierPipeline: loaded {} ({} labels) backend={}",
                 model_path, labels_.size(), engine_->getBackendName());
}

PipelineResult ClassifierPipeline::process(const cv::Mat& frame) {
    PipelineResult result;
    if (frame.empty()) return result;
    result.annotated_frame = frame.clone();
    if (!engine_) return result;

    auto results = engine_->classify(frame, /*top_k=*/5);
    if (results.empty()) return result;

    auto& best = results.front();
    if (best.confidence < cfg_.classifier_conf) return result;

    result.has_target = true;
    result.tid = best.class_id;
    if (best.class_id >= 0 && static_cast<size_t>(best.class_id) < labels_.size()) {
        result.tclass = labels_[best.class_id];
    } else if (!best.class_name.empty()) {
        result.tclass = best.class_name;
    }
    result.ta = best.confidence * 100.0;

    // Publish the full top-k through llpython so the web UI can render it.
    for (size_t i = 0; i < results.size() && i < 8; ++i) {
        result.llpython[i] = results[i].confidence;
    }

    char buf[128];
    std::snprintf(buf, sizeof(buf), "%s %.2f", result.tclass.c_str(), best.confidence);
    cv::putText(result.annotated_frame, buf, { 10, 30 },
                cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 255), 2);
    return result;
}

} // namespace limelight
