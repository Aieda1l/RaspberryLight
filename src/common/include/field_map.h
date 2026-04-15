#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>

namespace limelight {

struct Fiducial {
    int id;
    std::string family;
    double size;  // mm
    std::array<double, 16> transform;  // 4x4 homogeneous matrix (row-major)
};

struct FieldMap {
    std::string family;
    std::vector<Fiducial> fiducials;
    std::unordered_map<int, size_t> id_to_index;

    static FieldMap load(const std::string& path);
    const Fiducial* getFiducial(int id) const;
};

void from_json(const nlohmann::json& j, Fiducial& f);
void from_json(const nlohmann::json& j, FieldMap& m);

} // namespace limelight
