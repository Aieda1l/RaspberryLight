#include "visionserver/pipeline_manager.h"
#include <spdlog/spdlog.h>
#include <filesystem>

namespace limelight {

PipelineManager::PipelineManager() = default;

void PipelineManager::loadConfigs(const std::string& config_dir) {
    for (int i = 0; i < NUM_PIPELINES; i++) {
        std::string path = config_dir + "/" + std::to_string(i) + ".vpr";
        try {
            configs_[i] = PipelineConfig::load(path);
            spdlog::debug("Loaded pipeline {}: type={}, desc={}",
                         i, configs_[i].pipeline_type, configs_[i].desc);
        } catch (const std::exception& e) {
            spdlog::warn("Could not load pipeline {}: {}", i, e.what());
            // Keep default config
        }
    }

    // Build pipeline instances
    for (int i = 0; i < NUM_PIPELINES; i++) {
        rebuildPipeline(i);
    }
}

void PipelineManager::loadFieldMaps(const std::string& config_dir) {
    for (int i = 0; i < NUM_PIPELINES; i++) {
        std::string path = config_dir + "/extern/pipe" + std::to_string(i) + "/fmap.fmap";
        try {
            field_maps_[i] = FieldMap::load(path);
            spdlog::debug("Loaded field map for pipe{}: {} fiducials",
                         i, field_maps_[i].fiducials.size());
        } catch (const std::exception& e) {
            spdlog::debug("No field map for pipe{}: {}", i, e.what());
        }
    }
}

void PipelineManager::setActivePipeline(int index) {
    if (index < 0 || index >= NUM_PIPELINES) return;

    std::lock_guard<std::mutex> lock(mutex_);
    if (pipelines_[active_index_]) {
        pipelines_[active_index_]->onDeactivate();
    }
    active_index_ = index;
    if (pipelines_[active_index_]) {
        pipelines_[active_index_]->onActivate();
    }
    spdlog::info("Switched to pipeline {}: {}", index, configs_[index].pipeline_type);
}

int PipelineManager::getActivePipelineIndex() const {
    return active_index_;
}

Pipeline* PipelineManager::getActivePipeline() {
    return pipelines_[active_index_].get();
}

const PipelineConfig& PipelineManager::getActiveConfig() const {
    return configs_[active_index_];
}

PipelineConfig& PipelineManager::getConfig(int index) {
    return configs_[index];
}

void PipelineManager::saveConfig(int index, const std::string& config_dir) {
    std::string path = config_dir + "/" + std::to_string(index) + ".vpr";
    configs_[index].save(path);
}

PipelineResult PipelineManager::processFrame(const cv::Mat& frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pipelines_[active_index_]) {
        return PipelineResult{};
    }
    return pipelines_[active_index_]->process(frame);
}

void PipelineManager::rebuildPipeline(int index) {
    try {
        pipelines_[index] = Pipeline::create(configs_[index].pipeline_type);
        pipelines_[index]->setSlotIndex(index);
        pipelines_[index]->setFieldMap(field_maps_[index]);
        pipelines_[index]->configure(configs_[index]);
    } catch (const std::exception& e) {
        spdlog::warn("Cannot create pipeline {}: {}", index, e.what());
        pipelines_[index] = nullptr;
    }
}

} // namespace limelight
