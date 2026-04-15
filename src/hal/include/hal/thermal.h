#pragma once

#include <memory>
#include <string>
#include <vector>

namespace limelight::hal {

struct ThermalZone {
    std::string name;
    double temperature_c;
};

class Thermal {
public:
    virtual ~Thermal() = default;

    virtual bool init() = 0;
    virtual std::vector<ThermalZone> readAll() = 0;
    virtual double readCpuTemp() = 0;
    virtual double readGpuTemp() = 0;

    // Fan control (0-100%)
    virtual void setFanSpeed(int percent) = 0;
    virtual int getFanSpeed() const = 0;

    static std::unique_ptr<Thermal> create();
};

} // namespace limelight::hal
