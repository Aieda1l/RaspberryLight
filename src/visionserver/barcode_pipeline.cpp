// Barcode pipeline — ZXing ReadBarcodes().
//
// The original binary exports both cv::QRCodeDetector and ZXing::ReadBarcodes
// (confirmed in raw_strings.txt). We go with ZXing exclusively because it
// matches the mangled entry point `_ZN5ZXing12ReadBarcodesE...` and supports
// the full set of symbologies the web UI advertises in the `barcode_type`
// combo box (QR, DataMatrix, EAN, Code128, Code39, PDF417, Aztec).
//
// ZXing headers: <ZXing/ReadBarcode.h>, <ZXing/ImageView.h>

#include "visionserver/barcode_pipeline.h"

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include <ZXing/ReadBarcode.h>
#include <ZXing/ImageView.h>
#include <ZXing/BarcodeFormat.h>

namespace limelight {

BarcodePipeline::BarcodePipeline() {
    spdlog::info("Barcode pipeline constructed");
}

std::string BarcodePipeline::getType() const { return "pipe_barcode"; }

void BarcodePipeline::configure(const PipelineConfig& config) { cfg_ = config; }

namespace {

ZXing::BarcodeFormats parseFormats(const std::string& t) {
    using F = ZXing::BarcodeFormat;
    // Matches the tokens used in the .vpr JSON files.
    if (t == "qrzx")     return F::QRCode;
    if (t == "qr")       return F::QRCode;
    if (t == "datamatrix") return F::DataMatrix;
    if (t == "ean13")    return F::EAN13;
    if (t == "ean8")     return F::EAN8;
    if (t == "code128")  return F::Code128;
    if (t == "code39")   return F::Code39;
    if (t == "pdf417")   return F::PDF417;
    if (t == "aztec")    return F::Aztec;
    if (t == "all")      return F::Any;
    return F::QRCode;
}

}  // namespace

PipelineResult BarcodePipeline::process(const cv::Mat& frame) {
    PipelineResult result;
    if (frame.empty()) return result;
    result.annotated_frame = frame.clone();

    cv::Mat gray;
    if (frame.channels() == 1) gray = frame;
    else cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    ZXing::ImageView view(gray.data, gray.cols, gray.rows, ZXing::ImageFormat::Lum);
    ZXing::ReaderOptions opts;
    opts.setFormats(parseFormats(cfg_.barcode_type));
    opts.setTryHarder(true);
    opts.setTryRotate(true);

    auto results = ZXing::ReadBarcodes(view, opts);
    if (results.empty()) return result;

    const auto& best = results.front();  // ZXing returns results ordered by size
    result.has_target = true;
    result.tclass = ToString(best.format());
    // Target id encoded as the barcode payload hash — matches the original's
    // "numeric tid" behavior for non-fiducial pipelines.
    int id = 0;
    for (unsigned char c : best.text()) id = (id * 131 + c) & 0x7fffffff;
    result.tid = id;

    // Corners: position TL, TR, BR, BL.
    auto pos = best.position();
    result.corners.push_back({ static_cast<double>(pos.topLeft().x),     static_cast<double>(pos.topLeft().y) });
    result.corners.push_back({ static_cast<double>(pos.topRight().x),    static_cast<double>(pos.topRight().y) });
    result.corners.push_back({ static_cast<double>(pos.bottomRight().x), static_cast<double>(pos.bottomRight().y) });
    result.corners.push_back({ static_cast<double>(pos.bottomLeft().x),  static_cast<double>(pos.bottomLeft().y) });

    // tx/ty relative to image center.
    const double cx = (pos.topLeft().x + pos.topRight().x +
                       pos.bottomLeft().x + pos.bottomRight().x) / 4.0;
    const double cy = (pos.topLeft().y + pos.topRight().y +
                       pos.bottomLeft().y + pos.bottomRight().y) / 4.0;
    result.tx = ((cx - frame.cols / 2.0) / frame.cols) * 60.0;
    result.ty = ((frame.rows / 2.0 - cy) / frame.rows) * 45.0;

    // Approximate area via the bounding quad.
    const double w = std::abs(pos.topRight().x - pos.topLeft().x);
    const double h = std::abs(pos.bottomLeft().y - pos.topLeft().y);
    result.ta = (w * h) / (frame.cols * frame.rows) * 100.0;

    std::vector<cv::Point> poly = {
        { pos.topLeft().x,     pos.topLeft().y },
        { pos.topRight().x,    pos.topRight().y },
        { pos.bottomRight().x, pos.bottomRight().y },
        { pos.bottomLeft().x,  pos.bottomLeft().y },
    };
    cv::polylines(result.annotated_frame,
                  std::vector<std::vector<cv::Point>>{poly},
                  true, cv::Scalar(0, 255, 255), 2);
    cv::putText(result.annotated_frame, best.text(),
                { static_cast<int>(cx), static_cast<int>(cy) },
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    return result;
}

} // namespace limelight
