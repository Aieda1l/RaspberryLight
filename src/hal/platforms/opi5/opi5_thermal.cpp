// Orange Pi 5 thermal + fan HAL for RK3588.
//
// RK3588 exposes multiple thermal zones.  We enumerate every
// /sys/class/thermal/thermal_zoneN at init() and read each zone's `type`
// file so we don't depend on a fixed numeric order (vendor kernels shuffle
// them around).  Fan control goes through whatever hwmonN/pwm1 is
// available, or as a fallback the cooling_device/cur_state node.

#include "opi5_thermal.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <filesystem>
#include <fstream>

namespace limelight::hal {

namespace fs = std::filesystem;

namespace {

bool readIntFile(const fs::path& p, int& out) {
    std::ifstream f(p);
    if (!f.is_open()) return false;
    f >> out;
    return f.good() || f.eof();
}

bool writeIntFile(const fs::path& p, int value) {
    std::ofstream f(p);
    if (!f.is_open()) return false;
    f << value;
    return f.good();
}

std::string readStringFile(const fs::path& p) {
    std::ifstream f(p);
    if (!f.is_open()) return {};
    std::string s;
    std::getline(f, s);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' '))
        s.pop_back();
    return s;
}

// Scan /sys/class/hwmon for a PWM-capable hwmon node controlling a fan.
// Return an empty path if nothing suitable is found.
fs::path findFanPwm() {
    const fs::path hwmon_root = "/sys/class/hwmon";
    if (!fs::exists(hwmon_root)) return {};
    for (const auto& e : fs::directory_iterator(hwmon_root)) {
        const fs::path pwm = e.path() / "pwm1";
        if (fs::exists(pwm)) return pwm;
    }
    // Fallback: cooling_deviceN whose type contains "fan" or "pwm".
    const fs::path cool_root = "/sys/class/thermal";
    if (fs::exists(cool_root)) {
        for (const auto& e : fs::directory_iterator(cool_root)) {
            const std::string name = e.path().filename().string();
            if (name.rfind("cooling_device", 0) != 0) continue;
            const std::string type = readStringFile(e.path() / "type");
            if (type.find("fan") != std::string::npos ||
                type.find("pwm") != std::string::npos) {
                return e.path() / "cur_state";
            }
        }
    }
    return {};
}

}  // namespace

bool Opi5Thermal::init() {
    const fs::path root = "/sys/class/thermal";
    if (!fs::exists(root)) {
        spdlog::warn("Opi5Thermal: {} does not exist", root.string());
        return false;
    }

    // Cache every thermal_zoneN that we find so readAll() is O(n_zones).
    zone_paths_.clear();
    for (const auto& e : fs::directory_iterator(root)) {
        const std::string name = e.path().filename().string();
        if (name.rfind("thermal_zone", 0) != 0) continue;
        const std::string zone_type = readStringFile(e.path() / "type");
        if (!zone_type.empty()) {
            zone_paths_.push_back({zone_type, (e.path() / "temp").string()});
        }
    }
    std::sort(zone_paths_.begin(), zone_paths_.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    if (zone_paths_.empty()) {
        spdlog::warn("Opi5Thermal: no thermal zones found");
        return false;
    }
    spdlog::info("Opi5Thermal: found {} thermal zones", zone_paths_.size());

    fan_pwm_path_ = findFanPwm().string();
    if (!fan_pwm_path_.empty()) {
        spdlog::info("Opi5Thermal: fan control via {}", fan_pwm_path_);
    } else {
        spdlog::info("Opi5Thermal: no fan control node found — setFanSpeed is a no-op");
    }
    return true;
}

double Opi5Thermal::readThermalZone(const std::string& path) {
    int millideg = 0;
    if (!readIntFile(path, millideg)) return -1.0;
    return millideg / 1000.0;
}

std::vector<ThermalZone> Opi5Thermal::readAll() {
    std::vector<ThermalZone> out;
    out.reserve(zone_paths_.size());
    for (const auto& [name, path] : zone_paths_) {
        const double t = readThermalZone(path);
        if (t >= 0.0) out.push_back({name, t});
    }
    return out;
}

double Opi5Thermal::readCpuTemp() {
    // RK3588: prefer bigcore0-thermal, then soc-thermal, then first available.
    for (const auto& [name, path] : zone_paths_) {
        if (name == "bigcore0-thermal" || name == "cpu-thermal") {
            return readThermalZone(path);
        }
    }
    for (const auto& [name, path] : zone_paths_) {
        if (name == "soc-thermal") return readThermalZone(path);
    }
    if (!zone_paths_.empty()) return readThermalZone(zone_paths_.front().second);
    return -1.0;
}

double Opi5Thermal::readGpuTemp() {
    for (const auto& [name, path] : zone_paths_) {
        if (name == "gpu-thermal") return readThermalZone(path);
    }
    return -1.0;
}

void Opi5Thermal::setFanSpeed(int percent) {
    fan_speed_ = std::max(0, std::min(100, percent));
    if (fan_pwm_path_.empty()) return;

    // Two flavours of sysfs fan node to worry about:
    //   * hwmonN/pwm1       -- byte value 0..255
    //   * cooling_device/cur_state -- 0..max_state (stepwise)
    const fs::path p = fan_pwm_path_;
    if (p.filename() == "pwm1") {
        const int duty = (fan_speed_ * 255) / 100;
        writeIntFile(p, duty);
    } else {
        // cooling_device: read max_state, scale.
        int max_state = 0;
        readIntFile(p.parent_path() / "max_state", max_state);
        if (max_state <= 0) max_state = 1;
        const int step = (fan_speed_ * max_state + 50) / 100;
        writeIntFile(p, step);
    }
}

int Opi5Thermal::getFanSpeed() const {
    return fan_speed_;
}

}  // namespace limelight::hal
