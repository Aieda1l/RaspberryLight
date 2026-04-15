#include "field_map.h"
#include <fstream>
#include <stdexcept>

namespace limelight {

void from_json(const nlohmann::json& j, Fiducial& f) {
    j.at("id").get_to(f.id);
    j.at("family").get_to(f.family);
    j.at("size").get_to(f.size);

    auto& t = j.at("transform");
    for (int i = 0; i < 16 && i < static_cast<int>(t.size()); i++) {
        f.transform[i] = t[i].get<double>();
    }
}

void from_json(const nlohmann::json& j, FieldMap& m) {
    if (j.contains("fiducials")) {
        auto& arr = j.at("fiducials");
        m.fiducials.reserve(arr.size());
        for (auto& elem : arr) {
            Fiducial f;
            from_json(elem, f);
            m.id_to_index[f.id] = m.fiducials.size();
            m.fiducials.push_back(std::move(f));
        }
    }
    // Extract family from first fiducial if present
    if (!m.fiducials.empty()) {
        m.family = m.fiducials[0].family;
    }
}

FieldMap FieldMap::load(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot open field map: " + path);
    }
    auto j = nlohmann::json::parse(f);
    return j.get<FieldMap>();
}

const Fiducial* FieldMap::getFiducial(int id) const {
    auto it = id_to_index.find(id);
    if (it == id_to_index.end()) return nullptr;
    return &fiducials[it->second];
}

} // namespace limelight
