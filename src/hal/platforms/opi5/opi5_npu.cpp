// Orange Pi 5 NPU HAL — RK3588 RKNN Runtime backend.
//
// The Rockchip NPU is wrapped by librknnrt.so (installed from Rockchip's
// rknpu2 repo). This implementation targets RKNN API v2 which is what ships
// in the Rockchip BSP 5.10 and 6.1 kernels.
//
// Model flow:
//   1. rknn_init(&ctx, model_bytes, model_size, flags, NULL)
//   2. rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM) + RKNN_QUERY_INPUT_ATTR +
//      RKNN_QUERY_OUTPUT_ATTR to pick up the tensor shapes
//   3. rknn_inputs_set(ctx, n_inputs, rknn_input[])
//   4. rknn_run(ctx, NULL)
//   5. rknn_outputs_get(ctx, n_outputs, rknn_output[], NULL)
//   6. rknn_outputs_release(ctx, n_outputs, rknn_output[])
//   7. rknn_destroy(ctx)
//
// We register the factory in hal_factory.cpp through InferenceEngine::create();
// this file only provides the class definition, conditional on HAS_RKNN.

#include "hal/npu.h"

#ifdef HAS_RKNN

#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <spdlog/spdlog.h>

#include <rknn_api.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

namespace limelight::hal {

namespace {

struct IoTensor {
    rknn_tensor_attr attr;
};

std::vector<uint8_t> slurpBytes(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return {};
    in.seekg(0, std::ios::end);
    const auto sz = static_cast<size_t>(in.tellg());
    in.seekg(0, std::ios::beg);
    std::vector<uint8_t> buf(sz);
    in.read(reinterpret_cast<char*>(buf.data()), sz);
    return buf;
}

}  // namespace

class RknnEngine final : public InferenceEngine {
public:
    ~RknnEngine() override { unloadModel(); }

    bool loadModel(const std::string& path) override {
        unloadModel();

        auto bytes = slurpBytes(path);
        if (bytes.empty()) {
            spdlog::error("RKNN: unable to read {}", path);
            return false;
        }
        int ret = rknn_init(&ctx_, bytes.data(), static_cast<uint32_t>(bytes.size()),
                            0, nullptr);
        if (ret != RKNN_SUCC) {
            spdlog::error("RKNN: rknn_init failed {}", ret);
            ctx_ = 0;
            return false;
        }

        rknn_input_output_num io_num{};
        ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
        if (ret != RKNN_SUCC) {
            spdlog::error("RKNN: query io num failed {}", ret);
            unloadModel();
            return false;
        }

        inputs_.resize(io_num.n_input);
        for (uint32_t i = 0; i < io_num.n_input; ++i) {
            inputs_[i].attr.index = i;
            ret = rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &inputs_[i].attr,
                             sizeof(rknn_tensor_attr));
            if (ret != RKNN_SUCC) {
                spdlog::error("RKNN: query input attr {} failed {}", i, ret);
                unloadModel();
                return false;
            }
        }
        outputs_.resize(io_num.n_output);
        for (uint32_t i = 0; i < io_num.n_output; ++i) {
            outputs_[i].attr.index = i;
            ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &outputs_[i].attr,
                             sizeof(rknn_tensor_attr));
            if (ret != RKNN_SUCC) {
                spdlog::error("RKNN: query output attr {} failed {}", i, ret);
                unloadModel();
                return false;
            }
        }

        model_path_ = path;
        spdlog::info("RKNN: loaded {} ({} inputs, {} outputs)", path,
                     inputs_.size(), outputs_.size());
        return true;
    }

    void unloadModel() override {
        if (ctx_ != 0) {
            rknn_destroy(ctx_);
            ctx_ = 0;
        }
        inputs_.clear();
        outputs_.clear();
        model_path_.clear();
    }

    bool isLoaded() const override { return ctx_ != 0; }
    InferenceBackend getBackend() const override { return InferenceBackend::RKNN; }
    std::string getBackendName() const override { return "rknn"; }

    InferenceResult run(const cv::Mat& input) override {
        InferenceResult out;
        if (ctx_ == 0 || inputs_.empty()) return out;

        // Build one rknn_input descriptor for the primary tensor. Other
        // inputs are left as-is (assumed optional auxiliary inputs).
        rknn_input in{};
        in.index = 0;
        in.type  = RKNN_TENSOR_UINT8;
        in.fmt   = RKNN_TENSOR_NHWC;
        in.size  = static_cast<uint32_t>(input.total() * input.elemSize());
        in.buf   = input.data;
        in.pass_through = 0;

        int ret = rknn_inputs_set(ctx_, 1, &in);
        if (ret != RKNN_SUCC) return out;
        ret = rknn_run(ctx_, nullptr);
        if (ret != RKNN_SUCC) return out;

        std::vector<rknn_output> router(outputs_.size());
        for (size_t i = 0; i < outputs_.size(); ++i) {
            router[i].want_float = 1;
            router[i].is_prealloc = 0;
            router[i].index = static_cast<uint32_t>(i);
        }
        ret = rknn_outputs_get(ctx_, static_cast<uint32_t>(router.size()),
                               router.data(), nullptr);
        if (ret != RKNN_SUCC) return out;

        // Flatten everything into a single output_data vector (matches the
        // TFLite backend's behavior).
        if (!router.empty()) {
            const auto* p = static_cast<const float*>(router[0].buf);
            out.output_count = static_cast<int>(router[0].size / sizeof(float));
            out.output_data.assign(p, p + out.output_count);
            for (int d = 0; d < outputs_[0].attr.n_dims; ++d) {
                out.output_shape.push_back(static_cast<int>(outputs_[0].attr.dims[d]));
            }
        }
        rknn_outputs_release(ctx_, static_cast<uint32_t>(router.size()), router.data());
        return out;
    }

    // For detection/classification we reuse the TFLite-style postprocess:
    // run() above returns the raw floats so callers can interpret them.
    // Here we implement a simple YOLO parser for detection and a top-k
    // softmax reader for classification.
    std::vector<BoundingBox> detectObjects(const cv::Mat& frame,
                                           float conf_threshold,
                                           float nms_threshold) override {
        std::vector<BoundingBox> out;
        if (ctx_ == 0 || inputs_.empty()) return out;
        const auto& in_attr = inputs_[0].attr;
        const int in_h = static_cast<int>(in_attr.dims[1]);
        const int in_w = static_cast<int>(in_attr.dims[2]);
        cv::Mat resized;
        cv::resize(frame, resized, {in_w, in_h});

        InferenceResult r = run(resized);
        if (r.output_data.empty() || r.output_shape.size() < 3) return out;

        const int rows = r.output_shape[1];
        const int cols = r.output_shape[2];
        if (rows <= 0 || cols <= 6) return out;
        const int num_classes = cols - 5;
        const int W = frame.cols, H = frame.rows;

        for (int i = 0; i < rows; ++i) {
            const float* row = r.output_data.data() + i * cols;
            const float obj = row[4];
            if (obj < conf_threshold) continue;
            int best_c = 0;
            float best_s = 0.0f;
            for (int c = 0; c < num_classes; ++c) {
                const float s = row[5 + c] * obj;
                if (s > best_s) { best_s = s; best_c = c; }
            }
            if (best_s < conf_threshold) continue;
            const float cx = row[0] * W;
            const float cy = row[1] * H;
            const float bw = row[2] * W;
            const float bh = row[3] * H;
            BoundingBox b;
            b.x = cx - bw / 2;
            b.y = cy - bh / 2;
            b.w = bw;
            b.h = bh;
            b.confidence = best_s;
            b.class_id = best_c;
            out.push_back(b);
        }

        std::vector<cv::Rect> rects;
        std::vector<float> scores;
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

    std::vector<ClassificationResult> classify(const cv::Mat& frame, int top_k) override {
        std::vector<ClassificationResult> out;
        if (ctx_ == 0 || inputs_.empty()) return out;
        const auto& in_attr = inputs_[0].attr;
        const int in_h = static_cast<int>(in_attr.dims[1]);
        const int in_w = static_cast<int>(in_attr.dims[2]);
        cv::Mat resized;
        cv::resize(frame, resized, {in_w, in_h});

        InferenceResult r = run(resized);
        if (r.output_data.empty()) return out;

        std::vector<std::pair<float, int>> scored;
        scored.reserve(r.output_data.size());
        for (size_t i = 0; i < r.output_data.size(); ++i) {
            scored.emplace_back(r.output_data[i], static_cast<int>(i));
        }
        const int k = std::min(top_k, static_cast<int>(scored.size()));
        std::partial_sort(scored.begin(), scored.begin() + k, scored.end(),
                          [](auto& a, auto& b) { return a.first > b.first; });
        scored.resize(k);
        for (auto& p : scored) {
            ClassificationResult c;
            c.class_id   = p.second;
            c.confidence = p.first;
            out.push_back(c);
        }
        return out;
    }

    cv::Mat estimateDepth(const cv::Mat& frame) override {
        if (ctx_ == 0 || inputs_.empty()) return {};
        const auto& in_attr = inputs_[0].attr;
        const int in_h = static_cast<int>(in_attr.dims[1]);
        const int in_w = static_cast<int>(in_attr.dims[2]);
        cv::Mat resized;
        cv::resize(frame, resized, {in_w, in_h});

        InferenceResult r = run(resized);
        if (r.output_data.empty() || r.output_shape.size() < 3) return {};
        const int oh = r.output_shape[1];
        const int ow = r.output_shape[2];
        return cv::Mat(oh, ow, CV_32F, r.output_data.data()).clone();
    }

private:
    rknn_context ctx_ = 0;
    std::vector<IoTensor> inputs_;
    std::vector<IoTensor> outputs_;
    std::string model_path_;
};

std::unique_ptr<InferenceEngine> makeRknnEngine() {
    return std::make_unique<RknnEngine>();
}

} // namespace limelight::hal

#endif  // HAS_RKNN
