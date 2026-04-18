#pragma once

#include "pipeline.h"
#include "pipeline_config.h"
#include "config.h"
#include "field_map.h"
#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace limelight::hal { class Imu; }

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

    // DEPRECATED: returns a reference to an array element that the caller
    // *must not* use across a pipeline switch or a concurrent mutateConfig
    // call. Prefer getConfigSnapshot(index) for new code.
    [[deprecated("races with REST/NT mutation; use getConfigSnapshot() instead")]]
    const PipelineConfig& getActiveConfig() const;

    // Thread-safe snapshot of the config at `index`. Returns a copy so the
    // caller isn't exposed to concurrent mutation from REST/NT callbacks.
    PipelineConfig getConfigSnapshot(int index) const;

    // DEPRECATED: returns a mutable reference without holding the mutex. New
    // code must route all mutation through mutateConfig so that reads in the
    // vision thread don't race with writes from REST/NT callbacks.
    [[deprecated("unsynchronized mutation; use mutateConfig() instead")]]
    PipelineConfig& getConfig(int index);

    // Atomically mutate the config at `index` under the internal mutex and
    // rebuild the corresponding pipeline instance if the fn changed it.
    void mutateConfig(int index, const std::function<void(PipelineConfig&)>& fn);

    void saveConfig(int index, const std::string& config_dir);

    // Process frame through active pipeline
    PipelineResult processFrame(const cv::Mat& frame);

    // MegaTag2 fan-out.  Stored on the manager and pushed to every pipeline
    // on configure, plus routed through whenever an NT callback fires.
    void setImu(const hal::Imu* imu);
    void setRobotOrientation(const std::array<double, 6>& orient);
    void setImuMode(int mode);
    void setImuAssistAlpha(double alpha);

    // Frame capture timestamp (CLOCK_MONOTONIC ns).  main() calls this
    // immediately after camera->getFrame() so the fiducial pipeline can
    // pull the contemporaneous IMU sample from the ring buffer instead of
    // "latest" — important when the vision loop runs slower than the IMU.
    void setFrameCaptureTimestampNs(uint64_t ts_ns);

private:
    void rebuildPipeline(int index);

    std::array<PipelineConfig, NUM_PIPELINES> configs_;
    std::array<std::unique_ptr<Pipeline>, NUM_PIPELINES> pipelines_;
    std::array<FieldMap, NUM_PIPELINES> field_maps_;
    std::atomic<int> active_index_{0};
    mutable std::mutex mutex_;

    // Latched MegaTag2 inputs so newly-built pipelines inherit them.
    const hal::Imu*               imu_ = nullptr;
    std::array<double, 6>         robot_orientation_{};
    bool                          robot_orientation_valid_ = false;
    int                           imu_mode_ = 0;
    double                        imu_assist_alpha_ = 1.0;
};

} // namespace limelight
