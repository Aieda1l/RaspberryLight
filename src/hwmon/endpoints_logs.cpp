#include "endpoints.h"
#include "util.h"

#include <nlohmann/json.hpp>

#include <cctype>
#include <regex>
#include <sstream>
#include <string>

using nlohmann::json;

namespace limelight::hwmon {

static HttpResponse jsonResp(int status, const json& j) {
    HttpResponse r;
    r.status = status;
    r.content_type = "application/json";
    r.body = j.dump();
    return r;
}

static int parseLines(const std::string& query, int def) {
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

// --- /api/logs/service/:service -------------------------------------------
//
// FUN_001236e0 / getServiceLogs(service, lines). Runs two commands:
//   journalctl -u <service> -b --no-pager -n <lines>
//   systemctl status <service> --no-pager
// and returns both, keyed as "service", "status", plus the trailing logs.

json buildServiceLogsJson(const std::string& service, int lines) {
    json j = json::object();
    j["service"] = service;

    if (!isSafeServiceName(service)) {
        j["error"] = "Invalid service name";
        return j;
    }

    if (auto out = execCaptureArgv({"journalctl", "-u", service, "-b",
                                     "--no-pager", "-n",
                                     std::to_string(lines)})) {
        j["lines"] = *out;
    } else {
        j["lines"] = "";
    }

    if (auto out = execCaptureArgv({"systemctl", "status", service,
                                     "--no-pager"})) {
        j["status"] = *out;
    } else {
        j["status"] = "";
    }
    return j;
}

HttpResponse handleLogsService(const HttpRequest& req) {
    auto it = req.params.find("service");
    if (it == req.params.end()) {
        return jsonResp(400, makeError("Missing service parameter"));
    }
    int lines = parseLines(req.query, 100);
    return jsonResp(200, buildServiceLogsJson(it->second, lines));
}

// --- /api/logs/boots ------------------------------------------------------
//
// FUN_00132bc0 / getBootList(). Parses `journalctl --list-boots --no-pager`
// into a list of { boot_id, is_current_boot, first_log_time, last_log_time,
// log_count }. For each boot, also queries:
//   journalctl --boot=<id> --no-pager --lines=1 --output=json
// to find the first timestamp.
//
// The regex at 0x16ddc0 is: ^\s*(-?\d+)\s+([a-f0-9]{32})\s+(.+)$

json buildBootListJson() {
    json j = json::object();
    j["boots"] = json::array();

    std::string raw;
    if (auto out = execCapture("journalctl --list-boots --no-pager")) {
        raw = *out;
    }
    j["raw_output"] = raw;

    std::regex row_re(R"(^\s*(-?\d+)\s+([a-f0-9]{32})\s+(.+)$)");
    std::istringstream iss(raw);
    std::string line;
    json boots = json::array();
    while (std::getline(iss, line)) {
        if (line.empty()) continue;
        std::smatch m;
        if (!std::regex_match(line, m, row_re)) continue;

        int offset = 0;
        try { offset = std::stoi(m[1].str()); } catch (...) {}
        std::string boot_id = m[2].str();
        std::string rest = m[3].str();

        json b = json::object();
        b["boot_id"] = boot_id;
        b["is_current_boot"] = (offset == 0);
        b["boot_info"] = rest;

        // Fetch first log timestamp via journalctl's JSON output.
        if (auto out = execCaptureArgv({"journalctl", "--boot=" + boot_id,
                                         "--no-pager", "--lines=1",
                                         "--output=json"})) {
            try {
                auto entry = json::parse(*out);
                if (entry.contains("__REALTIME_TIMESTAMP")) {
                    b["first_log_time"] = entry["__REALTIME_TIMESTAMP"];
                }
            } catch (...) {
                b["first_log_time"] = "Unable to parse";
            }
        }
        b["last_log_time"] = "";
        b["log_count"] = 0;
        boots.push_back(std::move(b));
    }
    j["boots"] = std::move(boots);
    j["total_boots"] = static_cast<int>(j["boots"].size());
    return j;
}

HttpResponse handleLogsBoots(const HttpRequest&) {
    return jsonResp(200, buildBootListJson());
}

// --- /api/logs/boot/:bootid -----------------------------------------------
//
// FUN_0012ca44 / getBootLogs(bootid, lines). Validates the boot ID against
// ^[a-f0-9]{32}$ (strings at 0x16d6e8), then runs:
//   journalctl --boot=<id> --no-pager --lines=<n>
// Plus fetches per-boot info:
//   journalctl --list-boots --no-pager | grep <id>

json buildBootLogsJson(const std::string& boot_id, int lines) {
    json j = json::object();
    j["boot_id"] = boot_id;

    if (!isValidBootId(boot_id)) {
        j["error"] = "Invalid boot ID format";
        return j;
    }

    if (auto out = execCaptureArgv({"journalctl", "--boot=" + boot_id,
                                     "--no-pager",
                                     "--lines=" + std::to_string(lines)})) {
        j["lines"] = *out;
    } else {
        j["lines"] = "";
    }

    // "journalctl --list-boots | grep <boot_id>" — replace shell pipe with
    // in-process filter so boot_id can't be interpreted as shell syntax.
    if (auto out = execCaptureArgv({"journalctl", "--list-boots", "--no-pager"})) {
        std::istringstream iss(*out);
        std::string line;
        std::string matched;
        while (std::getline(iss, line)) {
            if (line.find(boot_id) != std::string::npos) {
                matched += line;
                matched += '\n';
            }
        }
        j["boot_info"] = matched;
    } else {
        j["boot_info"] = "";
    }
    return j;
}

HttpResponse handleLogsBoot(const HttpRequest& req) {
    auto it = req.params.find("bootid");
    if (it == req.params.end()) {
        return jsonResp(400, makeError("Missing bootid parameter"));
    }
    int lines = parseLines(req.query, 100);
    return jsonResp(200, buildBootLogsJson(it->second, lines));
}

// --- /api/logs/journal-stats ----------------------------------------------
//
// FUN_00115ea0 / getJournalStats(). Collects four journalctl sub-commands.

json buildJournalStatsJson() {
    json j = json::object();

    if (auto out = execCapture("journalctl --disk-usage --no-pager")) {
        j["disk_usage"] = *out;
    } else {
        j["disk_usage"] = "";
    }
    if (auto out = execCapture("journalctl --verify --no-pager")) {
        j["verification"] = *out;
    } else {
        j["verification"] = "";
    }
    if (auto out = execCapture("journalctl --no-pager --lines=0 --output=json | head -1")) {
        j["config_sample"] = *out;
    } else {
        j["config_sample"] = "";
    }
    if (auto out = execCapture("systemctl status systemd-journald --no-pager")) {
        j["journald_status"] = *out;
    } else {
        j["journald_status"] = "";
    }
    return j;
}

HttpResponse handleJournalStats(const HttpRequest&) {
    return jsonResp(200, buildJournalStatsJson());
}

}  // namespace limelight::hwmon
