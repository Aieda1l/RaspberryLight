#pragma once

#include "hal/thermal.h"

#include <string>
#include <utility>
#include <vector>

namespace limelight::hal {

// RK3588 thermal + fan HAL.  On boot we enumerate every
// /sys/class/thermal/thermal_zoneN and remember (type, temp-path)
// pairs so readAll() doesn't depend on a hard-coded zone ordering.
//
// Common RK3588 zone types:
//   soc-thermal          -- whole-SoC die sensor
//   bigcore0-thermal     -- Cortex-A76 cluster 0
//   bigcore1-thermal     -- Cortex-A76 cluster 1
//   littlecore-thermal   -- Cortex-A55 cluster
//   gpu-thermal          -- Mali-G610
//   npu-thermal          -- 6 TOPS NPU
class Opi5Thermal : public Thermal {
public:
    bool init() override;
    std::vector<ThermalZone> readAll() override;
    double readCpuTemp() override;
    double readGpuTemp() override;
    void setFanSpeed(int percent) override;
    int getFanSpeed() const override;

private:
    double readThermalZone(const std::string& path);

    // Pairs of (zone type, /sys/.../temp) sorted by type.
    std::vector<std::pair<std::string, std::string>> zone_paths_;
    // sysfs path we write to when setFanSpeed is called.  Empty if none.
    std::string fan_pwm_path_;
    int fan_speed_ = 0;
};

}  // namespace limelight::hal
