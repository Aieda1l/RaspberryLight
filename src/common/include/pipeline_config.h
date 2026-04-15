#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace limelight {

struct PipelineConfig {
    // Identity
    std::string desc = "Pipeline_Name";
    std::string pipeline_type = "pipe_fiducial";

    // HSV thresholds
    int hue_min = 55;
    int hue_max = 85;
    int sat_min = 70;
    int sat_max = 255;
    int val_min = 70;
    int val_max = 255;
    int invert_hue = 0;

    // Contour filtering
    double area_min = 0.001;
    double area_max = 100.0;
    double area_similarity = 0.0;
    double aspect_min = 0.0;
    double aspect_max = 20.0;
    double convexity_min = 10.0;
    double convexity_max = 100.0;
    double corner_approx = 5.0;
    int contour_grouping = 0;
    int contour_sort_final = 0;
    int force_convex = 1;
    int desired_contour_region = 0;
    int direction_filter = 0;
    int dual_close_sort_origin = 0;
    int intersection_filter = 0;
    int multigroup_min = 1;
    int multigroup_max = 7;
    int multigroup_rejector = 0;

    // Morphology
    int erosion_steps = 0;
    int dilation_steps = 0;
    int reverse_morpho = 0;

    // Camera controls
    double exposure = 120.0;
    double lcgain = 6.0;
    double red_balance = 1200.0;
    double blue_balance = 1975.0;
    int black_level = 0;
    int flicker = 0;
    int image_flip = 0;
    int image_source = 0;
    int pipeline_res = 0;

    // Fiducial (AprilTag) settings
    std::string fiducial_type = "aprilClassic36h11";
    std::string fiducial_backend = "umich";
    double fiducial_size = 165.1;
    double fiducial_denoise = 0.0;
    double fiducial_qualitythreshold = 2.0;
    int fiducial_resdiv = 2;
    int fiducial_skip3d = 1;
    std::string fiducial_idfilters;
    std::string fiducial_locfilters;
    std::string fiducial_vis_mode = "3dtargposebotspace";

    // ML settings
    double detector_conf = 0.8;
    double classifier_conf = 0.1;
    std::string detector_runtime = "cpu";
    std::string classifier_runtime = "cpu";
    std::string detector_idfilters;
    int nnp_rotate = 0;

    // Barcode
    std::string barcode_type = "qrzx";

    // LED
    int pipeline_led_enabled = 1;
    int pipeline_led_power = 100;

    // Streaming
    int img_to_show = 0;
    int debugpipe = 0;
    int send_corners = 0;
    int send_json = 0;

    // Crop/ROI
    double crop_x_min = -1.0;
    double crop_x_max = 1.0;
    double crop_y_min = -1.0;
    double crop_y_max = 1.0;
    double crop_focus = 0.0;
    double roi_x = 0.0;
    double roi_y = 0.0;
    double quality_focus = 0.3;

    // Crosshair
    double cross_a_x = 0.0;
    double cross_a_y = 0.0;
    double cross_a_a = 1.0;
    double cross_b_x = 0.0;
    double cross_b_y = 0.0;
    double cross_b_a = 1.0;

    // Robot pose offsets
    double rsf = 0.0;
    double rss = 0.0;
    double rsu = 0.0;
    double rspitch = 0.0;
    double rsroll = 0.0;
    double rsyaw = 0.0;
    double tsf = 0.0;
    double tss = 0.0;
    double tsu = 0.0;

    // Robot config
    double botwidth = 0.7112;
    double botlength = 0.7112;
    std::string bottype = "swerve";
    int botfloorsnap = 0;

    // Calibration
    int calibration_type = 0;

    // Outlier rejection
    double x_outlier_miqr = 1.5;
    double y_outlier_miqr = 1.5;

    // Python
    std::string python_snapscript_name;

    static PipelineConfig load(const std::string& path);
    void save(const std::string& path) const;
};

void from_json(const nlohmann::json& j, PipelineConfig& c);
void to_json(nlohmann::json& j, const PipelineConfig& c);

} // namespace limelight
