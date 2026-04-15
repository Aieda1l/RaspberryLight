#include "config.h"
#include <fstream>
#include <stdexcept>

namespace limelight {

void from_json(const nlohmann::json& j, GlobalSettings& s) {
    if (j.contains("default_vision_profile")) j.at("default_vision_profile").get_to(s.default_vision_profile);
    if (j.contains("preferred_calibration")) j.at("preferred_calibration").get_to(s.preferred_calibration);
    if (j.contains("stream_quality")) j.at("stream_quality").get_to(s.stream_quality);
    if (j.contains("stream_rate")) j.at("stream_rate").get_to(s.stream_rate);
    if (j.contains("stream_res")) j.at("stream_res").get_to(s.stream_res);
    if (j.contains("team_number")) j.at("team_number").get_to(s.team_number);
    if (j.contains("modbus_port")) j.at("modbus_port").get_to(s.modbus_port);
    if (j.contains("nt_server")) j.at("nt_server").get_to(s.nt_server);
    if (j.contains("usb_id")) j.at("usb_id").get_to(s.usb_id);
}

void to_json(nlohmann::json& j, const GlobalSettings& s) {
    j = nlohmann::json{
        {"default_vision_profile", s.default_vision_profile},
        {"preferred_calibration", s.preferred_calibration},
        {"stream_quality", s.stream_quality},
        {"stream_rate", s.stream_rate},
        {"stream_res", s.stream_res},
        {"team_number", s.team_number},
        {"modbus_port", s.modbus_port},
        {"nt_server", s.nt_server},
        {"usb_id", s.usb_id}
    };
}

GlobalSettings GlobalSettings::load(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot open global settings: " + path);
    }
    auto j = nlohmann::json::parse(f);
    return j.get<GlobalSettings>();
}

void GlobalSettings::save(const std::string& path) const {
    std::ofstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot write global settings: " + path);
    }
    nlohmann::json j = *this;
    f << j.dump();
}

} // namespace limelight
