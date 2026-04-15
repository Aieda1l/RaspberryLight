// Color pipeline — HSV threshold + contour detection.
//
// Behavior mirrors the original `color` vision mode: convert to HSV, threshold
// against the configured per-channel ranges, morph, find contours, filter by
// area/aspect/convexity, then compute tx/ty/ta/ts relative to the crosshair.
//
// HSV-range and contour-filter field names all come from PipelineConfig
// (which is derived from the .vpr JSON schema in pipe0-9.vpr).

#include "visionserver/color_pipeline.h"

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace limelight {

ColorPipeline::ColorPipeline() {
    spdlog::info("Color pipeline constructed");
}

std::string ColorPipeline::getType() const { return "pipe_color"; }

void ColorPipeline::configure(const PipelineConfig& config) { cfg_ = config; }

namespace {

// Convert NDC [-1..1] to pixel coordinates.
inline cv::Point ndcToPixel(double x, double y, int W, int H) {
    return {static_cast<int>((x + 1.0) * 0.5 * W),
            static_cast<int>((y + 1.0) * 0.5 * H)};
}

// Convert a pixel delta to degrees using a fixed field-of-view fallback;
// calibrated intrinsics go through the fiducial pipeline instead.
inline double pixelDeltaToDegrees(double dpx, int total_px, double fov_deg = 60.0) {
    if (total_px == 0) return 0.0;
    return (dpx / total_px) * fov_deg;
}

}  // namespace

PipelineResult ColorPipeline::process(const cv::Mat& frame) {
    PipelineResult result;
    if (frame.empty()) return result;

    const int W = frame.cols;
    const int H = frame.rows;
    const cv::Point crosshair = ndcToPixel(cfg_.cross_a_x, cfg_.cross_a_y, W, H);
    const double frame_area = static_cast<double>(W) * H;

    // --- 1. HSV threshold ------------------------------------------------
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    const cv::Scalar lower(cfg_.hue_min, cfg_.sat_min, cfg_.val_min);
    const cv::Scalar upper(cfg_.hue_max, cfg_.sat_max, cfg_.val_max);
    if (cfg_.invert_hue) {
        // Wrap-around hue range: union of [0..min] and [max..179].
        cv::Mat m1, m2;
        cv::inRange(hsv, cv::Scalar(0, cfg_.sat_min, cfg_.val_min),
                    cv::Scalar(cfg_.hue_min, cfg_.sat_max, cfg_.val_max), m1);
        cv::inRange(hsv, cv::Scalar(cfg_.hue_max, cfg_.sat_min, cfg_.val_min),
                    cv::Scalar(179, cfg_.sat_max, cfg_.val_max), m2);
        cv::bitwise_or(m1, m2, mask);
    } else {
        cv::inRange(hsv, lower, upper, mask);
    }

    // --- 2. Morphology ---------------------------------------------------
    if (cfg_.erosion_steps > 0) {
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), cfg_.erosion_steps);
    }
    if (cfg_.dilation_steps > 0) {
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), cfg_.dilation_steps);
    }

    // --- 3. Contour detection --------------------------------------------
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    struct Candidate {
        std::vector<cv::Point> points;
        double area;
        double aspect;
        cv::Rect rect;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(contours.size());

    for (auto& c : contours) {
        const double area = cv::contourArea(c);
        const double area_pct = area / frame_area * 100.0;
        if (area_pct < cfg_.area_min || area_pct > cfg_.area_max) continue;

        cv::Rect rect = cv::boundingRect(c);
        if (rect.height == 0) continue;
        const double aspect = static_cast<double>(rect.width) / rect.height;
        if (aspect < cfg_.aspect_min || aspect > cfg_.aspect_max) continue;

        // Convexity = contour area / convex hull area * 100.
        std::vector<cv::Point> hull;
        cv::convexHull(c, hull);
        const double hull_area = cv::contourArea(hull);
        double convexity = 100.0;
        if (hull_area > 0) convexity = area / hull_area * 100.0;
        if (convexity < cfg_.convexity_min || convexity > cfg_.convexity_max) continue;

        Candidate cand;
        cand.points = cfg_.force_convex ? std::move(hull) : std::move(c);
        cand.area   = area;
        cand.aspect = aspect;
        cand.rect   = rect;
        candidates.push_back(std::move(cand));
    }

    // --- 4. Pick the best candidate (largest area by default) -----------
    result.annotated_frame = frame.clone();
    cv::line(result.annotated_frame,
             {crosshair.x - 10, crosshair.y}, {crosshair.x + 10, crosshair.y},
             cv::Scalar(0, 255, 0), 1);
    cv::line(result.annotated_frame,
             {crosshair.x, crosshair.y - 10}, {crosshair.x, crosshair.y + 10},
             cv::Scalar(0, 255, 0), 1);

    if (candidates.empty()) return result;

    auto best_it = std::max_element(
        candidates.begin(), candidates.end(),
        [](const Candidate& a, const Candidate& b) { return a.area < b.area; });
    const auto& best = *best_it;

    cv::Moments m = cv::moments(best.points);
    if (m.m00 <= 0.0) return result;

    const double centroid_x = m.m10 / m.m00;
    const double centroid_y = m.m01 / m.m00;

    result.has_target = true;
    result.tx = pixelDeltaToDegrees(centroid_x - crosshair.x, W);
    // Y is negated so that "up" is positive, matching the original.
    result.ty = pixelDeltaToDegrees(crosshair.y - centroid_y, H, 45.0);
    result.ta = best.area / frame_area * 100.0;

    // Minimum-area rect gives us skew (-90..0).
    cv::RotatedRect rrect = cv::minAreaRect(best.points);
    result.ts = rrect.angle;

    // Corners of the selected contour (bounding box) — populated if
    // send_corners is enabled.
    if (cfg_.send_corners) {
        cv::Point2f pts[4];
        rrect.points(pts);
        for (int i = 0; i < 4; ++i) {
            result.corners.push_back({pts[i].x, pts[i].y});
        }
    }

    // --- 5. Annotate ----------------------------------------------------
    cv::drawContours(result.annotated_frame,
                     std::vector<std::vector<cv::Point>>{best.points}, 0,
                     cv::Scalar(0, 255, 255), 2);
    cv::rectangle(result.annotated_frame, best.rect, cv::Scalar(0, 255, 0), 2);

    return result;
}

} // namespace limelight
