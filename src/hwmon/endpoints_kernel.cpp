#include "endpoints.h"
#include "util.h"

#include <nlohmann/json.hpp>

#include <cctype>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using nlohmann::json;

namespace limelight::hwmon {

// --- /api/kernel/info -----------------------------------------------------
//
// Matches the original getKernelInfo() which was linked in from FUN_00116a40.
// Populates a JSON object with the output of `uname -r`, `uname -m`,
// `uname -a`, /proc/cmdline, /proc/version, /proc/uptime, total memory from
// /proc/meminfo, the CPU model from /proc/cpuinfo, and boot time via
// `who -b`. The original also shells out to `zcat /proc/config.gz` (falling
// back to `cat /boot/config-$(uname -r)`) and includes the first 50 lines
// as "config". We do the same here.

json buildKernelInfoJson() {
    json j = json::object();

    auto strip = [](std::string s) {
        while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
        return s;
    };

    if (auto out = execCapture("uname -r")) j["version"] = strip(*out);
    else j["version"] = "Unknown";
    if (auto out = execCapture("uname -m")) j["architecture"] = strip(*out);
    else j["architecture"] = "Unknown";
    if (auto out = execCapture("uname -a")) j["full_info"] = strip(*out);
    else j["full_info"] = "Unknown";

    if (auto uptime = readFileLine("/proc/uptime")) j["uptime"] = *uptime;
    else j["uptime"] = "";

    if (auto out = execCapture("who -b 2>/dev/null | awk '{print $3, $4}'")) {
        j["boot_time"] = strip(*out);
    } else {
        j["boot_time"] = "";
    }

    if (auto cmdline = readFileLine("/proc/cmdline")) j["cmdline"] = *cmdline;
    else j["cmdline"] = "";

    if (auto version = readFile("/proc/version")) j["version_detailed"] = trim(*version);
    else j["version_detailed"] = "";

    // /proc/meminfo: extract MemTotal as total_memory_kb.
    if (auto mi = readFile("/proc/meminfo")) {
        std::istringstream iss(*mi);
        std::string line;
        while (std::getline(iss, line)) {
            if (line.compare(0, 9, "MemTotal:") == 0) {
                // "MemTotal:    12345 kB"
                auto parts = splitWs(line.substr(9));
                if (!parts.empty()) {
                    try { j["total_memory_kb"] = std::stoll(parts[0]); }
                    catch (...) { j["total_memory_kb"] = 0; }
                }
                break;
            }
        }
    }

    // /proc/cpuinfo first "model name" line.
    if (auto ci = readFile("/proc/cpuinfo")) {
        std::istringstream iss(*ci);
        std::string line;
        std::string cpu_model = "Unknown";
        while (std::getline(iss, line)) {
            size_t colon = line.find(':');
            if (colon == std::string::npos) continue;
            std::string key = trim(line.substr(0, colon));
            if (key == "model name") {
                cpu_model = trim(line.substr(colon + 1));
                break;
            }
        }
        j["cpu_model"] = cpu_model;
    } else {
        j["cpu_model"] = "Unknown";
    }

    // Kernel config preview.
    std::string cfg;
    if (auto out = execCapture("zcat /proc/config.gz 2>/dev/null | head -50")) {
        cfg = *out;
    }
    if (cfg.empty()) {
        if (auto out = execCapture("cat /boot/config-$(uname -r) 2>/dev/null | head -50")) {
            cfg = *out;
        }
    }
    j["config"] = cfg;

    return j;
}

// --- /api/kernel/modules --------------------------------------------------
//
// Matches getKernelModules() / FUN_0011a660. Reads /proc/modules, collects
// each row, and also counts builtin modules from /lib/modules/$(uname
// -r)/modules.builtin, plus the =y/=m counts from the kernel config.

json buildKernelModulesJson() {
    json j = json::object();
    json loaded = json::array();
    int64_t total_size = 0;

    // /proc/modules format:
    // <name> <size> <refcount> <used_by> <state> <address>
    if (auto out = readFile("/proc/modules")) {
        std::istringstream iss(*out);
        std::string line;
        while (std::getline(iss, line)) {
            auto parts = splitWs(line);
            if (parts.size() < 6) continue;
            json m = json::object();
            m["name"] = parts[0];
            try { m["size_bytes"] = std::stoll(parts[1]); total_size += std::stoll(parts[1]); }
            catch (...) { m["size_bytes"] = 0; }
            try { m["used_by_count"] = std::stoi(parts[2]); }
            catch (...) { m["used_by_count"] = 0; }
            m["used_by"] = parts[3];
            m["state"] = parts[4];
            m["address"] = parts[5];

            // modinfo description — best-effort.
            std::string cmd = "modinfo " + parts[0] +
                              " 2>/dev/null | grep '^description:' | cut -d: -f2- | sed 's/^ *//'";
            if (auto desc = execCapture(cmd)) {
                std::string d = *desc;
                while (!d.empty() && (d.back() == '\n' || d.back() == '\r')) d.pop_back();
                m["description"] = d;
            } else {
                m["description"] = "";
            }
            loaded.push_back(std::move(m));
        }
    }
    j["loaded_modules"] = std::move(loaded);
    j["total_size"] = total_size;

    // Available module files on disk.
    int available = 0;
    if (auto out = execCapture("find /lib/modules/$(uname -r) -name '*.ko' | wc -l")) {
        try { available = std::stoi(trim(*out)); } catch (...) {}
    }
    j["available_modules"] = available;

    // Builtin module list.
    json builtin = json::array();
    if (auto out = execCapture("cat /lib/modules/$(uname -r)/modules.builtin 2>/dev/null")) {
        std::istringstream iss(*out);
        std::string line;
        while (std::getline(iss, line)) {
            if (!line.empty()) builtin.push_back(line);
        }
    }
    j["builtin_modules"] = std::move(builtin);

    // Count =y/=m lines in kernel config (tries both locations).
    int y_count = 0, m_count = 0;
    auto count_cmd = [](const std::string& cmd) {
        if (auto out = execCapture(cmd)) {
            try { return std::stoi(trim(*out)); } catch (...) { return 0; }
        }
        return 0;
    };
    y_count = count_cmd("zcat /proc/config.gz 2>/dev/null | grep '=y$' | wc -l");
    if (y_count == 0) {
        y_count = count_cmd("cat /boot/config-$(uname -r) 2>/dev/null | grep '=y$' | wc -l");
    }
    m_count = count_cmd("zcat /proc/config.gz 2>/dev/null | grep '=m$' | wc -l");
    if (m_count == 0) {
        m_count = count_cmd("cat /boot/config-$(uname -r) 2>/dev/null | grep '=m$' | wc -l");
    }

    json cfg_info = json::object();
    cfg_info["builtin_in_config"] = y_count;
    cfg_info["modules_in_config"] = m_count;
    j["config_info"] = std::move(cfg_info);
    j["total_modules"] = static_cast<int>(j["loaded_modules"].size()) +
                         static_cast<int>(j["builtin_modules"].size());

    if (auto out = execCapture("uname -r")) {
        std::string v = *out;
        while (!v.empty() && (v.back() == '\n' || v.back() == '\r')) v.pop_back();
        j["kernel_version"] = v;
    }
    return j;
}

// --- /api/kernel/ringbuffer -----------------------------------------------
//
// getKernelRingbuffer() / FUN_00119f60. Grabs the last 100 lines of dmesg
// with level decoding plus per-level counts and a total.

json buildKernelRingbufferJson() {
    json j = json::object();

    std::string recent;
    if (auto out = execCapture(
            "dmesg --level=emerg,alert,crit,err,warn,notice,info,debug --human --decode "
            "2>/dev/null | tail -100")) {
        recent = *out;
    }
    j["recent_messages"] = recent;

    auto count = [](const char* cmd) {
        if (auto out = execCapture(cmd)) {
            try { return std::stoll(trim(*out)); } catch (...) { return 0LL; }
        }
        return 0LL;
    };

    json lvl = json::object();
    lvl["emergency"] = count("dmesg --level=emerg 2>/dev/null | wc -l");
    lvl["alert"]     = count("dmesg --level=alert 2>/dev/null | wc -l");
    lvl["critical"]  = count("dmesg --level=crit 2>/dev/null | wc -l");
    lvl["error"]     = count("dmesg --level=err 2>/dev/null | wc -l");
    lvl["warning"]   = count("dmesg --level=warn 2>/dev/null | wc -l");
    j["level_counts"] = std::move(lvl);
    j["total_messages"] = count("dmesg 2>/dev/null | wc -l");
    return j;
}

// --- /api/logs/dmesg ------------------------------------------------------
//
// getDmesgLogs(lines) / FUN_00115ba0. Runs `dmesg | tail -n <lines>`.
// `lines` comes from the ?lines= query parameter (default 100).

static int parseLinesParam(const std::string& query, int def = 100) {
    // Look for "lines=<n>" in the query.
    size_t p = query.find("lines=");
    if (p == std::string::npos) return def;
    p += 6;
    int n = 0;
    while (p < query.size() && std::isdigit(static_cast<unsigned char>(query[p]))) {
        n = n * 10 + (query[p] - '0');
        ++p;
    }
    return n > 0 ? n : def;
}

json buildDmesgJson(int lines) {
    json j = json::object();
    std::string cmd = "dmesg | tail -n " + std::to_string(lines);
    if (auto out = execCapture(cmd)) {
        j["lines"] = *out;
    } else {
        j["lines"] = "";
    }
    return j;
}

// --- /api/kernel/all ------------------------------------------------------
//
// FUN_00123200 aggregates info + modules + ringbuffer + dmesg(100).

static HttpResponse jsonResponse(const json& j) {
    HttpResponse r;
    r.status = 200;
    r.content_type = "application/json";
    r.body = j.dump();
    return r;
}

HttpResponse handleKernelInfo(const HttpRequest&) {
    return jsonResponse(buildKernelInfoJson());
}

HttpResponse handleKernelModules(const HttpRequest&) {
    return jsonResponse(buildKernelModulesJson());
}

HttpResponse handleKernelRingbuffer(const HttpRequest&) {
    return jsonResponse(buildKernelRingbufferJson());
}

HttpResponse handleLogsDmesg(const HttpRequest& req) {
    int lines = parseLinesParam(req.query, 100);
    return jsonResponse(buildDmesgJson(lines));
}

HttpResponse handleKernelAll(const HttpRequest&) {
    json j = json::object();
    j["info"] = buildKernelInfoJson();
    j["modules"] = buildKernelModulesJson();
    j["ringbuffer"] = buildKernelRingbufferJson();
    j["dmesg"] = buildDmesgJson(100);
    return jsonResponse(j);
}

}  // namespace limelight::hwmon
