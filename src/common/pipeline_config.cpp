#include "pipeline_config.h"
#include <fstream>
#include <stdexcept>

namespace limelight {

// Macro to reduce repetitive JSON field extraction
#define JSON_GET_OPT(j, field, target) \
    if ((j).contains(#field)) (j).at(#field).get_to((target).field)

void from_json(const nlohmann::json& j, PipelineConfig& c) {
    JSON_GET_OPT(j, desc, c);
    JSON_GET_OPT(j, pipeline_type, c);
    JSON_GET_OPT(j, hue_min, c);
    JSON_GET_OPT(j, hue_max, c);
    JSON_GET_OPT(j, sat_min, c);
    JSON_GET_OPT(j, sat_max, c);
    JSON_GET_OPT(j, val_min, c);
    JSON_GET_OPT(j, val_max, c);
    JSON_GET_OPT(j, invert_hue, c);
    JSON_GET_OPT(j, area_min, c);
    JSON_GET_OPT(j, area_max, c);
    JSON_GET_OPT(j, area_similarity, c);
    JSON_GET_OPT(j, aspect_min, c);
    JSON_GET_OPT(j, aspect_max, c);
    JSON_GET_OPT(j, convexity_min, c);
    JSON_GET_OPT(j, convexity_max, c);
    JSON_GET_OPT(j, corner_approx, c);
    JSON_GET_OPT(j, contour_grouping, c);
    JSON_GET_OPT(j, contour_sort_final, c);
    JSON_GET_OPT(j, force_convex, c);
    JSON_GET_OPT(j, desired_contour_region, c);
    JSON_GET_OPT(j, direction_filter, c);
    JSON_GET_OPT(j, dual_close_sort_origin, c);
    JSON_GET_OPT(j, intersection_filter, c);
    JSON_GET_OPT(j, multigroup_min, c);
    JSON_GET_OPT(j, multigroup_max, c);
    JSON_GET_OPT(j, multigroup_rejector, c);
    JSON_GET_OPT(j, erosion_steps, c);
    JSON_GET_OPT(j, dilation_steps, c);
    JSON_GET_OPT(j, reverse_morpho, c);
    JSON_GET_OPT(j, exposure, c);
    JSON_GET_OPT(j, lcgain, c);
    JSON_GET_OPT(j, red_balance, c);
    JSON_GET_OPT(j, blue_balance, c);
    JSON_GET_OPT(j, black_level, c);
    JSON_GET_OPT(j, flicker, c);
    JSON_GET_OPT(j, image_flip, c);
    JSON_GET_OPT(j, image_source, c);
    JSON_GET_OPT(j, pipeline_res, c);
    JSON_GET_OPT(j, fiducial_type, c);
    JSON_GET_OPT(j, fiducial_backend, c);
    JSON_GET_OPT(j, fiducial_size, c);
    JSON_GET_OPT(j, fiducial_denoise, c);
    JSON_GET_OPT(j, fiducial_qualitythreshold, c);
    JSON_GET_OPT(j, fiducial_resdiv, c);
    JSON_GET_OPT(j, fiducial_skip3d, c);
    JSON_GET_OPT(j, fiducial_idfilters, c);
    JSON_GET_OPT(j, fiducial_locfilters, c);
    JSON_GET_OPT(j, fiducial_vis_mode, c);
    JSON_GET_OPT(j, detector_conf, c);
    JSON_GET_OPT(j, classifier_conf, c);
    JSON_GET_OPT(j, detector_runtime, c);
    JSON_GET_OPT(j, classifier_runtime, c);
    JSON_GET_OPT(j, detector_idfilters, c);
    JSON_GET_OPT(j, nnp_rotate, c);
    JSON_GET_OPT(j, barcode_type, c);
    JSON_GET_OPT(j, pipeline_led_enabled, c);
    JSON_GET_OPT(j, pipeline_led_power, c);
    JSON_GET_OPT(j, img_to_show, c);
    JSON_GET_OPT(j, debugpipe, c);
    JSON_GET_OPT(j, send_corners, c);
    JSON_GET_OPT(j, send_json, c);
    JSON_GET_OPT(j, crop_x_min, c);
    JSON_GET_OPT(j, crop_x_max, c);
    JSON_GET_OPT(j, crop_y_min, c);
    JSON_GET_OPT(j, crop_y_max, c);
    JSON_GET_OPT(j, crop_focus, c);
    JSON_GET_OPT(j, roi_x, c);
    JSON_GET_OPT(j, roi_y, c);
    JSON_GET_OPT(j, quality_focus, c);
    JSON_GET_OPT(j, cross_a_x, c);
    JSON_GET_OPT(j, cross_a_y, c);
    JSON_GET_OPT(j, cross_a_a, c);
    JSON_GET_OPT(j, cross_b_x, c);
    JSON_GET_OPT(j, cross_b_y, c);
    JSON_GET_OPT(j, cross_b_a, c);
    JSON_GET_OPT(j, rsf, c);
    JSON_GET_OPT(j, rss, c);
    JSON_GET_OPT(j, rsu, c);
    JSON_GET_OPT(j, rspitch, c);
    JSON_GET_OPT(j, rsroll, c);
    JSON_GET_OPT(j, rsyaw, c);
    JSON_GET_OPT(j, tsf, c);
    JSON_GET_OPT(j, tss, c);
    JSON_GET_OPT(j, tsu, c);
    JSON_GET_OPT(j, botwidth, c);
    JSON_GET_OPT(j, botlength, c);
    JSON_GET_OPT(j, bottype, c);
    JSON_GET_OPT(j, botfloorsnap, c);
    JSON_GET_OPT(j, calibration_type, c);
    JSON_GET_OPT(j, hfov_deg, c);
    JSON_GET_OPT(j, vfov_deg, c);
    JSON_GET_OPT(j, x_outlier_miqr, c);
    JSON_GET_OPT(j, y_outlier_miqr, c);
    JSON_GET_OPT(j, python_snapscript_name, c);
}

#undef JSON_GET_OPT

void to_json(nlohmann::json& j, const PipelineConfig& c) {
    j = nlohmann::json{
        {"desc", c.desc}, {"pipeline_type", c.pipeline_type},
        {"hue_min", c.hue_min}, {"hue_max", c.hue_max},
        {"sat_min", c.sat_min}, {"sat_max", c.sat_max},
        {"val_min", c.val_min}, {"val_max", c.val_max},
        {"invert_hue", c.invert_hue},
        {"area_min", c.area_min}, {"area_max", c.area_max},
        {"area_similarity", c.area_similarity},
        {"aspect_min", c.aspect_min}, {"aspect_max", c.aspect_max},
        {"convexity_min", c.convexity_min}, {"convexity_max", c.convexity_max},
        {"corner_approx", c.corner_approx},
        {"contour_grouping", c.contour_grouping}, {"contour_sort_final", c.contour_sort_final},
        {"force_convex", c.force_convex},
        {"desired_contour_region", c.desired_contour_region},
        {"direction_filter", c.direction_filter},
        {"dual_close_sort_origin", c.dual_close_sort_origin},
        {"intersection_filter", c.intersection_filter},
        {"multigroup_min", c.multigroup_min}, {"multigroup_max", c.multigroup_max},
        {"multigroup_rejector", c.multigroup_rejector},
        {"erosion_steps", c.erosion_steps}, {"dilation_steps", c.dilation_steps},
        {"reverse_morpho", c.reverse_morpho},
        {"exposure", c.exposure}, {"lcgain", c.lcgain},
        {"red_balance", c.red_balance}, {"blue_balance", c.blue_balance},
        {"black_level", c.black_level}, {"flicker", c.flicker},
        {"image_flip", c.image_flip}, {"image_source", c.image_source},
        {"pipeline_res", c.pipeline_res},
        {"fiducial_type", c.fiducial_type}, {"fiducial_backend", c.fiducial_backend},
        {"fiducial_size", c.fiducial_size}, {"fiducial_denoise", c.fiducial_denoise},
        {"fiducial_qualitythreshold", c.fiducial_qualitythreshold},
        {"fiducial_resdiv", c.fiducial_resdiv}, {"fiducial_skip3d", c.fiducial_skip3d},
        {"fiducial_idfilters", c.fiducial_idfilters},
        {"fiducial_locfilters", c.fiducial_locfilters},
        {"fiducial_vis_mode", c.fiducial_vis_mode},
        {"detector_conf", c.detector_conf}, {"classifier_conf", c.classifier_conf},
        {"detector_runtime", c.detector_runtime}, {"classifier_runtime", c.classifier_runtime},
        {"detector_idfilters", c.detector_idfilters}, {"nnp_rotate", c.nnp_rotate},
        {"barcode_type", c.barcode_type},
        {"pipeline_led_enabled", c.pipeline_led_enabled},
        {"pipeline_led_power", c.pipeline_led_power},
        {"img_to_show", c.img_to_show}, {"debugpipe", c.debugpipe},
        {"send_corners", c.send_corners}, {"send_json", c.send_json},
        {"crop_x_min", c.crop_x_min}, {"crop_x_max", c.crop_x_max},
        {"crop_y_min", c.crop_y_min}, {"crop_y_max", c.crop_y_max},
        {"crop_focus", c.crop_focus}, {"roi_x", c.roi_x}, {"roi_y", c.roi_y},
        {"quality_focus", c.quality_focus},
        {"cross_a_x", c.cross_a_x}, {"cross_a_y", c.cross_a_y}, {"cross_a_a", c.cross_a_a},
        {"cross_b_x", c.cross_b_x}, {"cross_b_y", c.cross_b_y}, {"cross_b_a", c.cross_b_a},
        {"rsf", c.rsf}, {"rss", c.rss}, {"rsu", c.rsu},
        {"rspitch", c.rspitch}, {"rsroll", c.rsroll}, {"rsyaw", c.rsyaw},
        {"tsf", c.tsf}, {"tss", c.tss}, {"tsu", c.tsu},
        {"botwidth", c.botwidth}, {"botlength", c.botlength}, {"bottype", c.bottype},
        {"botfloorsnap", c.botfloorsnap},
        {"calibration_type", c.calibration_type},
        {"hfov_deg", c.hfov_deg}, {"vfov_deg", c.vfov_deg},
        {"x_outlier_miqr", c.x_outlier_miqr}, {"y_outlier_miqr", c.y_outlier_miqr},
        {"python_snapscript_name", c.python_snapscript_name}
    };
}

PipelineConfig PipelineConfig::load(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot open pipeline config: " + path);
    }
    auto j = nlohmann::json::parse(f);
    return j.get<PipelineConfig>();
}

void PipelineConfig::save(const std::string& path) const {
    std::ofstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot write pipeline config: " + path);
    }
    nlohmann::json j = *this;
    f << j.dump();
}

} // namespace limelight
