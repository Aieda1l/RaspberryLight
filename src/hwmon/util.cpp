#include "util.h"

#include <array>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <sys/wait.h>

namespace limelight::hwmon {

std::optional<std::string> execCapture(const std::string& cmd, int* exit_status_out) {
    FILE* pipe = ::popen(cmd.c_str(), "r");
    if (!pipe) {
        if (exit_status_out) *exit_status_out = -1;
        return std::nullopt;
    }
    std::string out;
    std::array<char, 4096> buf{};
    while (true) {
        size_t n = std::fread(buf.data(), 1, buf.size(), pipe);
        if (n == 0) break;
        out.append(buf.data(), n);
    }
    int rc = ::pclose(pipe);
    if (exit_status_out) {
        if (WIFEXITED(rc)) {
            *exit_status_out = WEXITSTATUS(rc);
        } else {
            *exit_status_out = -1;
        }
    }
    return out;
}

std::optional<std::string> readFile(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return std::nullopt;
    std::ostringstream oss;
    oss << f.rdbuf();
    return oss.str();
}

std::optional<std::string> readFileLine(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return std::nullopt;
    std::string line;
    std::getline(f, line);
    return trim(line);
}

std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

std::vector<std::string> split(const std::string& s, char delim) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == delim) {
            out.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    out.push_back(cur);
    return out;
}

std::vector<std::string> splitWs(const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (std::isspace(static_cast<unsigned char>(c))) {
            if (!cur.empty()) {
                out.push_back(cur);
                cur.clear();
            }
        } else {
            cur.push_back(c);
        }
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

bool isValidBootId(const std::string& s) {
    // Matches ^[a-f0-9]{32}$ — same regex the original uses at 0x16d6e8.
    if (s.size() != 32) return false;
    for (char c : s) {
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    }
    return true;
}

bool isSafeServiceName(const std::string& s) {
    if (s.empty() || s.size() > 128) return false;
    for (char c : s) {
        const bool ok =
            (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') ||
            c == '.' || c == '_' || c == '-' || c == '@' || c == ':';
        if (!ok) return false;
    }
    return true;
}

nlohmann::json makeError(const std::string& msg) {
    nlohmann::json j;
    j["success"] = false;
    j["error"] = msg;
    return j;
}

}  // namespace limelight::hwmon
