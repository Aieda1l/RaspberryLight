// Neural depth pipeline.
//
// Runs a monocular depth estimator (original ships depth_hailo.hef) through
// hal::InferenceEngine::estimateDepth() and renders a color-mapped overlay
// into the annotated frame. Reports the median depth of the center ROI as
// "ta" and the centroid of the closest region as "tx/ty".

#include "visionserver/depth_pipeline.h"

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include <filesystem>

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
}  // namespace

DepthPipeline::DepthPipeline() { spdlog::info("Depth pipeline constructed"); }

std::string DepthPipeline::getType() const { return "pipe_neuraldepth"; }

void DepthPipeline::configure(const PipelineConfig& config) {
    cfg_ = config;
    auto backend = parseBackend(cfg_.detector_runtime);
    engine_ = hal::InferenceEngine::create(backend);
    if (!engine_) return;

    const std::string hef  = findModel(pipeline_index_, ".hef");
    const std::string rknn = findModel(pipeline_index_, ".rknn");
    const std::string tfl  = findModel(pipeline_index_, ".tflite");
    std::string model_path = tfl;
    if (!hef.empty())  model_path = hef;
    if (!rknn.empty()) model_path = rknn;
    if (model_path.empty()) {
        spdlog::warn("DepthPipeline: no model in extern/pipe{}", pipeline_index_);
        engine_.reset();
        return;
    }
    if (!engine_->loadModel(model_path)) {
        spdlog::error("DepthPipeline: failed to load {}", model_path);
        engine_.reset();
    }
}

PipelineResult DepthPipeline::process(const cv::Mat& frame) {
    PipelineResult result;
    if (frame.empty()) return result;
    result.annotated_frame = frame.clone();
    if (!engine_) return result;

    cv::Mat depth = engine_->estimateDepth(frame);
    if (depth.empty()) return result;

    // Normalize to 0..255 for display.
    cv::Mat depth_u8;
    cv::normalize(depth, depth_u8, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::Mat color;
    cv::applyColorMap(depth_u8, color, cv::COLORMAP_TURBO);
    if (color.size() != frame.size()) {
        cv::resize(color, color, frame.size());
    }
    cv::addWeighted(color, 0.4, result.annotated_frame, 0.6, 0.0, result.annotated_frame);

    // Median depth of the center crop -> "ta".
    const int cw = frame.cols / 3;
    const int ch = frame.rows / 3;
    cv::Rect roi((frame.cols - cw) / 2, (frame.rows - ch) / 2, cw, ch);
    cv::Mat center = depth(roi);
    std::vector<float> vals;
    vals.reserve(center.total());
    for (int y = 0; y < center.rows; ++y) {
        for (int x = 0; x < center.cols; ++x) {
            vals.push_back(center.at<float>(y, x));
        }
    }
    if (!vals.empty()) {
        std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
        result.has_target = true;
        result.ta = vals[vals.size() / 2];
    }

    return result;
}

} // namespace limelight
