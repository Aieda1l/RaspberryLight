#pragma once

#include "pipeline.h"
#include "pipeline_config.h"
#include "config.h"
#include "field_map.h"
#include <array>
#include <memory>
#include <mutex>
#include <string>

namespace limelight {

constexpr int NUM_PIPELINES = 10;

class PipelineManager {
public:
    PipelineManager();

    // Load all pipeline configs from directory
    void loadConfigs(const std::string& config_dir);
    void loadFieldMaps(const std::string& config_dir);

    // Pipeline switching
    void setActivePipeline(int index);
    int getActivePipelineIndex() const;
    Pipeline* getActivePipeline();
    const PipelineConfig& getActiveConfig() const;

    // Config access
    PipelineConfig& getConfig(int index);
    void saveConfig(int index, const std::string& config_dir);

    // Process frame through active pipeline
    PipelineResult processFrame(const cv::Mat& frame);

private:
    void rebuildPipeline(int index);

    std::array<PipelineConfig, NUM_PIPELINES> configs_;
    std::array<std::unique_ptr<Pipeline>, NUM_PIPELINES> pipelines_;
    std::array<FieldMap, NUM_PIPELINES> field_maps_;
    int active_index_ = 0;
    mutable std::mutex mutex_;
};

} // namespace limelight
