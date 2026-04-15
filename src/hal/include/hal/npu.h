#pragma once

#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>

namespace limelight::hal {

enum class InferenceBackend {
    TFLITE_CPU,
    HAILO,
    RKNN,
    EDGETPU
};

struct InferenceResult {
    std::vector<float> output_data;
    std::vector<int> output_shape;
    int output_count = 0;
};

struct BoundingBox {
    float x, y, w, h;
    float confidence;
    int class_id;
    std::string class_name;
};

struct ClassificationResult {
    int class_id;
    std::string class_name;
    float confidence;
};

class InferenceEngine {
public:
    virtual ~InferenceEngine() = default;

    virtual bool loadModel(const std::string& model_path) = 0;
    virtual void unloadModel() = 0;
    virtual bool isLoaded() const = 0;

    virtual InferenceBackend getBackend() const = 0;
    virtual std::string getBackendName() const = 0;

    // Run raw inference
    virtual InferenceResult run(const cv::Mat& input) = 0;

    // High-level: object detection (postprocessed with NMS)
    virtual std::vector<BoundingBox> detectObjects(
        const cv::Mat& frame, float conf_threshold = 0.5f,
        float nms_threshold = 0.4f) = 0;

    // High-level: classification (top-k)
    virtual std::vector<ClassificationResult> classify(
        const cv::Mat& frame, int top_k = 5) = 0;

    // High-level: depth estimation
    virtual cv::Mat estimateDepth(const cv::Mat& frame) = 0;

    // Factory: creates best available backend
    static std::unique_ptr<InferenceEngine> create(InferenceBackend preferred);
    static std::vector<InferenceBackend> availableBackends();
};

} // namespace limelight::hal
