#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <cstdint>

namespace limelight {

struct GlobalSettings {
    int default_vision_profile = 0;
    int preferred_calibration = 0;
    int stream_quality = 30;
    int stream_rate = 22;
    int stream_res = 0;
    int team_number = 0;
    int modbus_port = 502;
    std::string nt_server = "0.0.0.0";
    int usb_id = 0;

    static GlobalSettings load(const std::string& path);
    void save(const std::string& path) const;
};

void from_json(const nlohmann::json& j, GlobalSettings& s);
void to_json(nlohmann::json& j, const GlobalSettings& s);

} // namespace limelight
