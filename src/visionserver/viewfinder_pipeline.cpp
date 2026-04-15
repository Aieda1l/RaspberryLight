// Viewfinder / focus pipeline.
//
// Phases from the log strings in visionserver_strings.txt:
//   "Viewfinder setup complete"
//   "Viewfinder size chosen is <WxH>"
// The original draws a centered crosshair and (in focus mode) an in-image
// Laplacian variance indicator. Reproduced below with plain OpenCV ops.

#include "visionserver/viewfinder_pipeline.h"

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace limelight {

ViewfinderPipeline::ViewfinderPipeline(bool focus_mode) : focus_mode_(focus_mode) {
    spdlog::info("Viewfinder setup complete");
}

std::string ViewfinderPipeline::getType() const {
    return focus_mode_ ? "pipe_focus" : "pipe_viewfinder";
}

void ViewfinderPipeline::configure(const PipelineConfig& config) {
    cfg_ = config;
    spdlog::info("Viewfinder size chosen is {}", cfg_.pipeline_res);
}

PipelineResult ViewfinderPipeline::process(const cv::Mat& frame) {
    PipelineResult result;
    if (frame.empty()) return result;

    // Clone so the annotated overlay doesn't scribble on the raw frame.
    result.annotated_frame = frame.clone();

    // Crosshair — drawn at (cross_a_x, cross_a_y) which is in [-1..1] NDC.
    const int W = result.annotated_frame.cols;
    const int H = result.annotated_frame.rows;
    const int cx = static_cast<int>((cfg_.cross_a_x + 1.0) * 0.5 * W);
    const int cy = static_cast<int>((cfg_.cross_a_y + 1.0) * 0.5 * H);
    const cv::Scalar crosshair_color(0, 255, 0);  // green
    cv::line(result.annotated_frame, {cx - 10, cy}, {cx + 10, cy}, crosshair_color, 2);
    cv::line(result.annotated_frame, {cx, cy - 10}, {cx, cy + 10}, crosshair_color, 2);

    if (focus_mode_) {
        // Laplacian variance over the center crop — higher means sharper.
        const int crop_w = W / 3;
        const int crop_h = H / 3;
        cv::Rect roi{(W - crop_w) / 2, (H - crop_h) / 2, crop_w, crop_h};
        cv::Mat gray, lap;
        cv::cvtColor(frame(roi), gray, cv::COLOR_BGR2GRAY);
        cv::Laplacian(gray, lap, CV_64F);
        cv::Scalar mean, stddev;
        cv::meanStdDev(lap, mean, stddev);
        const double variance = stddev[0] * stddev[0];

        cv::rectangle(result.annotated_frame, roi, cv::Scalar(255, 255, 0), 1);
        char buf[64];
        std::snprintf(buf, sizeof(buf), "focus=%.1f", variance);
        cv::putText(result.annotated_frame, buf, {roi.x, roi.y - 8},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

        // Publish focus variance through llpython[0] so the web UI can plot it.
        result.llpython[0] = variance;
    }

    return result;
}

} // namespace limelight
