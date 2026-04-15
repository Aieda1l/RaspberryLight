// InferenceEngine factory — dispatches to the best available backend.
//
// The original Limelight binary links against both libtensorflow-lite and
// libhailort (with libedgetpu as an optional delegate). Symbols confirmed:
//
//   tflite::Interpreter, tflite::FlatBufferModel, tflite::InterpreterBuilder
//   tflite::ops::builtin::BuiltinOpResolver
//   edgetpu_list_devices, edgetpu_free_devices
//   libhailort.so.4.23.0 linked (symbols not decompiled in detail)
//
// For the Orange Pi 5 port we add RKNN as the primary accelerator. The
// other three backends are compile-time toggleable via HAS_TFLITE, HAS_HAILO,
// HAS_EDGETPU so we can build on any target.

#include "hal/npu.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

#ifdef HAS_TFLITE
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#endif

namespace limelight::hal {

namespace {

#ifdef HAS_TFLITE
// TFLite CPU backend. Loads a .tflite model and runs it on CPU using the
// default BuiltinOpResolver. detectObjects supports both YOLO single-head
// and TFOD SSD 4-output heads with NMS. classify does a top-k partial
// sort supporting float and uint8 quantized outputs. estimateDepth
// returns the raw output tensor as a float matrix.
class TfLiteCpuEngine final : public InferenceEngine {
public:
    bool loadModel(const std::string& path) override {
        model_ = tflite::FlatBufferModel::BuildFromFile(path.c_str());
        if (!model_) {
            spdlog::error("TFLite: failed to load model {}", path);
            return false;
        }
        tflite::ops::builtin::BuiltinOpResolver resolver;
        tflite::InterpreterBuilder builder(*model_, resolver);
        if (builder(&interp_) != kTfLiteOk || !interp_) {
            spdlog::error("TFLite: failed to build interpreter");
            model_.reset();
            return false;
        }
        interp_->SetNumThreads(2);
        if (interp_->AllocateTensors() != kTfLiteOk) {
            spdlog::error("TFLite: AllocateTensors failed");
            model_.reset();
            interp_.reset();
            return false;
        }
        model_path_ = path;
        return true;
    }

    void unloadModel() override {
        interp_.reset();
        model_.reset();
        model_path_.clear();
    }

    bool isLoaded() const override { return static_cast<bool>(interp_); }
    InferenceBackend getBackend() const override { return InferenceBackend::TFLITE_CPU; }
    std::string getBackendName() const override { return "tflite-cpu"; }

    bool setInputAndInvoke(const cv::Mat& frame) {
        if (!interp_) return false;
        const TfLiteIntArray* in_dims = interp_->input_tensor(0)->dims;
        if (in_dims->size < 4) return false;
        const int in_h = in_dims->data[1];
        const int in_w = in_dims->data[2];
        cv::Mat resized;
        cv::resize(frame, resized, {in_w, in_h});

        if (interp_->input_tensor(0)->type == kTfLiteFloat32) {
            cv::Mat f;
            resized.convertTo(f, CV_32F, 1.0 / 255.0);
            std::memcpy(interp_->typed_input_tensor<float>(0), f.data,
                        f.total() * f.elemSize());
        } else {
            std::memcpy(interp_->typed_input_tensor<uint8_t>(0), resized.data,
                        resized.total() * resized.elemSize());
        }
        return interp_->Invoke() == kTfLiteOk;
    }

    InferenceResult run(const cv::Mat& input) override {
        InferenceResult out;
        if (!interp_) return out;

        // Assumes single float32 input tensor with shape [1, H, W, C].
        auto* in_tensor = interp_->typed_input_tensor<float>(0);
        if (!in_tensor) return out;

        // Copy bytes — caller is responsible for resize/normalize.
        std::memcpy(in_tensor, input.data, input.total() * input.elemSize());

        if (interp_->Invoke() != kTfLiteOk) {
            spdlog::error("Failed to invoke tflite!");
            return out;
        }

        auto* out_tensor = interp_->typed_output_tensor<float>(0);
        if (!out_tensor) return out;
        size_t n = 1;
        const TfLiteIntArray* dims = interp_->output_tensor(0)->dims;
        for (int i = 0; i < dims->size; ++i) {
            n *= static_cast<size_t>(dims->data[i]);
            out.output_shape.push_back(dims->data[i]);
        }
        out.output_data.assign(out_tensor, out_tensor + n);
        out.output_count = static_cast<int>(n);
        return out;
    }

    // ---- Object detection --------------------------------------------------
    //
    // Supports the two formats the original firmware ships: TFLite TFODAPI
    // (SSD-style, 4 outputs: boxes, classes, scores, num) and YOLO-style
    // (1 output: [N, 85]). We detect which by the number of output tensors.
    std::vector<BoundingBox> detectObjects(const cv::Mat& frame,
                                           float conf_threshold,
                                           float nms_threshold) override {
        std::vector<BoundingBox> out;
        if (!interp_) return out;

        // Resize + normalize into the input tensor.
        if (!setInputAndInvoke(frame)) return out;

        const size_t num_outputs = interp_->outputs().size();
        const int W = frame.cols, H = frame.rows;

        if (num_outputs >= 4) {
            // TFODAPI SSD: locations[N,4], classes[N], scores[N], num_det.
            const float* boxes   = interp_->typed_output_tensor<float>(0);
            const float* classes = interp_->typed_output_tensor<float>(1);
            const float* scores  = interp_->typed_output_tensor<float>(2);
            const float* num_f   = interp_->typed_output_tensor<float>(3);
            const int num = static_cast<int>(num_f ? num_f[0] : 0);
            for (int i = 0; i < num; ++i) {
                if (scores[i] < conf_threshold) continue;
                const float y1 = boxes[i * 4 + 0] * H;
                const float x1 = boxes[i * 4 + 1] * W;
                const float y2 = boxes[i * 4 + 2] * H;
                const float x2 = boxes[i * 4 + 3] * W;
                BoundingBox b;
                b.x = x1;
                b.y = y1;
                b.w = x2 - x1;
                b.h = y2 - y1;
                b.confidence = scores[i];
                b.class_id = static_cast<int>(classes[i]);
                out.push_back(b);
            }
        } else if (num_outputs == 1) {
            // YOLOv5/8-style single output [1, N, 85] or [1, 85, N].
            const TfLiteTensor* t = interp_->output_tensor(0);
            const float* data = interp_->typed_output_tensor<float>(0);
            if (t->dims->size < 3) return out;
            const int d1 = t->dims->data[1];
            const int d2 = t->dims->data[2];
            // Heuristic: if d1 < d2, layout is [1, rows, cols] => rows=N.
            int rows, cols;
            bool row_major = (d1 > d2);
            if (row_major) {
                rows = d1; cols = d2;
            } else {
                rows = d2; cols = d1;
            }
            
            cv::Mat transposed;
            const float* fast_data = data;
            if (!row_major) {
                cv::Mat tensorMat(d1, d2, CV_32F, const_cast<float*>(data));
                cv::transpose(tensorMat, transposed);
                fast_data = reinterpret_cast<float*>(transposed.data);
                std::swap(rows, cols);
                row_major = true;
            }
            
            const int num_classes = cols - 5;
            if (num_classes <= 0) return out;
            for (int i = 0; i < rows; ++i) {
                auto val = [&](int j) -> float {
                    return fast_data[i * cols + j];
                };
                const float obj = val(4);
                if (obj < conf_threshold) continue;
                int best_c = 0;
                float best_s = 0.0f;
                for (int c = 0; c < num_classes; ++c) {
                    const float s = val(5 + c) * obj;
                    if (s > best_s) { best_s = s; best_c = c; }
                }
                if (best_s < conf_threshold) continue;
                const float cx = val(0) * W;
                const float cy = val(1) * H;
                const float bw = val(2) * W;
                const float bh = val(3) * H;
                BoundingBox b;
                b.x = cx - bw / 2;
                b.y = cy - bh / 2;
                b.w = bw;
                b.h = bh;
                b.confidence = best_s;
                b.class_id = best_c;
                out.push_back(b);
            }
        }

        // NMS via OpenCV.
        if (out.empty()) return out;
        std::vector<cv::Rect> rects;
        std::vector<float> scores;
        rects.reserve(out.size());
        scores.reserve(out.size());
        for (const auto& b : out) {
            rects.emplace_back(static_cast<int>(b.x), static_cast<int>(b.y),
                               static_cast<int>(b.w), static_cast<int>(b.h));
            scores.push_back(b.confidence);
        }
        std::vector<int> indices;
        cv::dnn::NMSBoxes(rects, scores, conf_threshold, nms_threshold, indices);
        std::vector<BoundingBox> filtered;
        filtered.reserve(indices.size());
        for (int i : indices) filtered.push_back(out[i]);
        return filtered;
    }

    // ---- Classification ----------------------------------------------------
    std::vector<ClassificationResult> classify(const cv::Mat& frame,
                                               int top_k) override {
        std::vector<ClassificationResult> out;
        if (!interp_) return out;

        if (!setInputAndInvoke(frame)) return out;

        const TfLiteTensor* t = interp_->output_tensor(0);
        size_t n = 1;
        for (int i = 0; i < t->dims->size; ++i) n *= static_cast<size_t>(t->dims->data[i]);

        std::vector<std::pair<float, int>> scored;
        scored.reserve(n);
        if (t->type == kTfLiteFloat32) {
            const float* d = interp_->typed_output_tensor<float>(0);
            for (size_t i = 0; i < n; ++i) scored.emplace_back(d[i], static_cast<int>(i));
        } else if (t->type == kTfLiteUInt8) {
            const uint8_t* d = interp_->typed_output_tensor<uint8_t>(0);
            const float scale = t->params.scale;
            const int zp = t->params.zero_point;
            for (size_t i = 0; i < n; ++i)
                scored.emplace_back(scale * (d[i] - zp), static_cast<int>(i));
        } else {
            return out;
        }
        if (static_cast<int>(scored.size()) > top_k) {
            std::partial_sort(scored.begin(), scored.begin() + top_k, scored.end(),
                              [](auto& a, auto& b) { return a.first > b.first; });
            scored.resize(top_k);
        } else {
            std::sort(scored.begin(), scored.end(),
                      [](auto& a, auto& b) { return a.first > b.first; });
        }
        for (auto& p : scored) {
            ClassificationResult r;
            r.class_id   = p.second;
            r.confidence = p.first;
            out.push_back(r);
        }
        return out;
    }

    // ---- Depth -------------------------------------------------------------
    cv::Mat estimateDepth(const cv::Mat& frame) override {
        if (!setInputAndInvoke(frame)) return {};

        const TfLiteTensor* t = interp_->output_tensor(0);
        if (t->dims->size < 3) return {};
        // Assume [1, H, W] or [1, H, W, 1].
        const int oh = t->dims->data[1];
        const int ow = t->dims->data[2];
        cv::Mat depth(oh, ow, CV_32F,
                      const_cast<float*>(interp_->typed_output_tensor<float>(0)));
        return depth.clone();
    }

private:
    std::unique_ptr<tflite::FlatBufferModel> model_;
    std::unique_ptr<tflite::Interpreter>     interp_;
    std::string model_path_;
};
#endif  // HAS_TFLITE

// Placeholder backends. Each throws at loadModel() until we port the real
// code over — keeping them compile-able avoids link errors in configurations
// where a backend is enabled at compile time but not yet implemented.
class StubEngine final : public InferenceEngine {
public:
    explicit StubEngine(InferenceBackend b, const char* name)
        : backend_(b), name_(name) {}

    bool loadModel(const std::string&) override {
        spdlog::warn("InferenceEngine {} is a stub — loadModel rejected", name_);
        return false;
    }
    void unloadModel() override {}
    bool isLoaded() const override { return false; }
    InferenceBackend getBackend() const override { return backend_; }
    std::string getBackendName() const override { return name_; }
    InferenceResult run(const cv::Mat&) override { return {}; }
    std::vector<BoundingBox> detectObjects(const cv::Mat&, float, float) override { return {}; }
    std::vector<ClassificationResult> classify(const cv::Mat&, int) override { return {}; }
    cv::Mat estimateDepth(const cv::Mat&) override { return {}; }

private:
    InferenceBackend backend_;
    const char* name_;
};

}  // namespace

// Per-backend factory functions. Each backend may live in a separate
// translation unit (RKNN is in src/hal/platforms/opi5/opi5_npu.cpp) so we
// declare the factory functions here and let the linker pick up the one
// enabled by the HAS_* define.
#ifdef HAS_RKNN
std::unique_ptr<InferenceEngine> makeRknnEngine();
#endif
#ifdef HAS_HAILO
std::unique_ptr<InferenceEngine> makeHailoEngine();
#endif
#ifdef HAS_EDGETPU
std::unique_ptr<InferenceEngine> makeEdgeTpuEngine();
#endif

std::unique_ptr<InferenceEngine> InferenceEngine::create(InferenceBackend preferred) {
    auto backends = availableBackends();
    auto has = [&](InferenceBackend b) {
        return std::find(backends.begin(), backends.end(), b) != backends.end();
    };

    // Try preferred first, then fall back.
    auto try_make = [&](InferenceBackend b) -> std::unique_ptr<InferenceEngine> {
        switch (b) {
#ifdef HAS_TFLITE
            case InferenceBackend::TFLITE_CPU: return std::make_unique<TfLiteCpuEngine>();
#endif
#ifdef HAS_RKNN
            case InferenceBackend::RKNN: {
                auto e = makeRknnEngine();
                if (e) return e;
                return std::make_unique<StubEngine>(b, "rknn");
            }
#endif
#ifdef HAS_HAILO
            case InferenceBackend::HAILO: {
                auto e = makeHailoEngine();
                if (e) return e;
                return std::make_unique<StubEngine>(b, "hailo");
            }
#endif
#ifdef HAS_EDGETPU
            case InferenceBackend::EDGETPU: {
                auto e = makeEdgeTpuEngine();
                if (e) return e;
                return std::make_unique<StubEngine>(b, "edgetpu");
            }
#endif
            default: return nullptr;
        }
    };

    if (has(preferred)) {
        if (auto e = try_make(preferred)) return e;
    }
    // Fallback order: RKNN -> HAILO -> EDGETPU -> TFLITE_CPU (most to least
    // hardware-accelerated, with TFLite as the universal fallback).
    for (auto b : {InferenceBackend::RKNN, InferenceBackend::HAILO,
                   InferenceBackend::EDGETPU, InferenceBackend::TFLITE_CPU}) {
        if (!has(b)) continue;
        if (auto e = try_make(b)) return e;
    }
    return nullptr;
}

}  // namespace limelight::hal
