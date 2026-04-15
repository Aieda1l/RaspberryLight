#include "endpoints.h"
#include "util.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <unordered_map>
#include <utility>
#include <vector>

using nlohmann::json;

namespace limelight::hwmon {

static HttpResponse jsonResp(int status, const json& j) {
    HttpResponse r;
    r.status = status;
    r.content_type = "application/json";
    r.body = j.dump();
    return r;
}

// --- /api/usb -------------------------------------------------------------
//
// FUN_0012eca0 / getUsbInfo(). Runs `lsusb` + `lsusb -t` and parses each
// line against the regex:
//   Bus (\d+) Device (\d+): ID ([0-9a-f]{4}):([0-9a-f]{4}) (.+)
// then for each device reads /sys/bus/usb/devices/<bus-N>/speed. The
// original also counts hubs via `find /sys/bus/usb/devices -name
// bDeviceClass -exec grep -l '09' ...`.

json buildUsbJson() {
    json j = json::object();

    json devices = json::array();
    std::regex line_re(R"(Bus (\d+) Device (\d+): ID ([0-9a-f]{4}):([0-9a-f]{4}) (.+))");

    if (auto out = execCapture("lsusb")) {
        std::istringstream iss(*out);
        std::string line;
        while (std::getline(iss, line)) {
            std::smatch m;
            if (!std::regex_search(line, m, line_re)) continue;
            json d = json::object();
            d["bus"] = m[1].str();
            d["device"] = m[2].str();
            d["vendor_id"] = m[3].str();
            d["product_id"] = m[4].str();
            d["description"] = m[5].str();

            // Try reading USB speed for this device.
            std::string speed_path =
                "/sys/bus/usb/devices/" + m[1].str() + "-" + m[2].str() + "/speed";
            if (auto s = readFileLine(speed_path)) {
                d["speed"] = *s;
            }
            devices.push_back(std::move(d));
        }
    }
    j["devices"] = std::move(devices);

    if (auto out = execCapture("lsusb -t")) {
        j["tree"] = *out;
    } else {
        j["tree"] = "";
    }

    if (auto out = execCapture(
            "find /sys/bus/usb/devices -name 'bDeviceClass' "
            "-exec grep -l '09' {} \\; 2>/dev/null | head -20")) {
        j["hubs_raw"] = *out;
    } else {
        j["hubs_raw"] = "";
    }
    return j;
}

HttpResponse handleUsb(const HttpRequest&) {
    return jsonResp(200, buildUsbJson());
}

// --- /api/i2c -------------------------------------------------------------
//
// FUN_001234a0 / getI2cInfo(). Scans /dev/i2c-* buses with `i2cdetect -y`,
// parses the output, and classifies each found address against a hardcoded
// device type table. The device type table (strings at 0x16cb80..0x16cdb0)
// is encoded here as a lookup map.

static const std::unordered_map<int, const char*>& i2cTypeTable() {
    static const std::unordered_map<int, const char*> table{
        // Addresses derived from the Limelight strings table.
        {0x53, "Accelerometer (ADXL345/MMA8451)"},
        {0x1d, "Accelerometer (ADXL345)"},
        {0x68, "RTC/IMU (DS1307/DS3231/MPU6050)"},
        {0x69, "IMU (MPU6050/MPU9250 ALT)"},
        {0x76, "Pressure/Temp (BMP280/BME280)"},
        {0x77, "Pressure/Temp (BMP180/BMP280 ALT)"},
        {0x3c, "OLED Display (SSD1306)"},
        {0x3d, "OLED Display (SSD1306 ALT)"},
        {0x48, "ADC/Temp (ADS1115/TMP102)"},
        {0x49, "ADC/Temp (ADS1115/TMP102 ALT)"},
        {0x50, "EEPROM (24C32/24C64)"},
        {0x51, "EEPROM (24C32/24C64 ALT)"},
        {0x20, "I/O Expander (MCP23017)"},
        {0x21, "I/O Expander (MCP23017 ALT)"},
        {0x31, "GoBilda Pinpoint"},
        {0x40, "PWM Driver (PCA9685)"},
        {0x70, "I2C Multiplexer (TCA9548A)"},
        {0x71, "I2C Multiplexer (TCA9548A ALT)"},
        {0x60, "Motor Driver (Various)"},
        {0x61, "Motor Driver (Various ALT)"},
        {0x0c, "Magnetometer (AK8975)"},
        {0x0d, "Magnetometer (AK8963)"},
        {0x0e, "Magnetometer (AK8963 ALT)"},
    };
    return table;
}

static const char* classifyI2cAddr(int addr) {
    const auto& table = i2cTypeTable();
    auto it = table.find(addr);
    return it != table.end() ? it->second : "Unknown Device";
}

json buildI2cJson() {
    json j = json::object();
    json buses = json::array();
    int total_devices = 0;

    DIR* d = ::opendir("/dev");
    if (!d) {
        j["buses"] = buses;
        j["device_count"] = 0;
        return j;
    }
    struct dirent* e;
    std::vector<int> bus_ids;
    while ((e = ::readdir(d)) != nullptr) {
        std::string name(e->d_name);
        if (name.rfind("i2c-", 0) != 0) continue;
        try { bus_ids.push_back(std::stoi(name.substr(4))); }
        catch (...) {}
    }
    ::closedir(d);
    std::sort(bus_ids.begin(), bus_ids.end());

    for (int bus : bus_ids) {
        std::string cmd = "i2cdetect -y " + std::to_string(bus) + " 2>/dev/null";
        auto out = execCapture(cmd);
        if (!out) continue;

        json b = json::object();
        b["bus_number"] = bus;
        b["detection_method"] = "i2cdetect";
        json devices = json::array();

        // i2cdetect output lines look like:
        //   00:          -- -- -- -- -- -- -- -- -- -- -- -- --
        //   10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        std::istringstream iss(*out);
        std::string line;
        while (std::getline(iss, line)) {
            if (line.size() < 4 || line[2] != ':') continue;
            int row = 0;
            try { row = std::stoi(line.substr(0, 2), nullptr, 16); }
            catch (...) { continue; }

            for (int col = 0; col < 16; ++col) {
                // Each cell is at column 4 + col*3 and is two hex chars.
                size_t cell = 4 + static_cast<size_t>(col) * 3;
                if (cell + 1 >= line.size()) break;
                char c1 = line[cell];
                char c2 = line[cell + 1];
                if (c1 == '-' || c1 == ' ' || c1 == 'U') continue;
                int addr = row + col;
                json dev = json::object();
                char buf[8];
                std::snprintf(buf, sizeof(buf), "0x%02x", addr);
                dev["address"] = buf;
                dev["likely_type"] = classifyI2cAddr(addr);
                devices.push_back(std::move(dev));
                total_devices++;
            }
        }
        b["devices"] = std::move(devices);
        buses.push_back(std::move(b));
    }

    j["buses"] = std::move(buses);
    j["device_count"] = total_devices;
    return j;
}

HttpResponse handleI2c(const HttpRequest&) {
    return jsonResp(200, buildI2cJson());
}

// --- /api/interrupts ------------------------------------------------------
//
// FUN_001205c0 / getInterruptsInfo(). Parses /proc/interrupts into a list
// of { number, cpu_counts[], total, trigger, hardware_irq } plus a summary
// with total_interrupts and top_interrupts[].

json buildInterruptsJson() {
    json j = json::object();
    std::ifstream f("/proc/interrupts");
    if (!f.is_open()) {
        j["error"] = "Failed to open /proc/interrupts";
        return j;
    }

    std::string header_line;
    if (!std::getline(f, header_line)) {
        j["error"] = "Empty /proc/interrupts";
        return j;
    }
    auto cpu_headers = splitWs(header_line);
    const size_t cpu_n = cpu_headers.size();
    j["cpu_headers"] = cpu_headers;
    j["cpu_count"] = static_cast<int>(cpu_n);

    json interrupts = json::array();
    int64_t total_interrupts = 0;
    std::string line;
    while (std::getline(f, line)) {
        // Format: "<irq>: <counts...> <trigger> <desc>"
        auto cols = splitWs(line);
        if (cols.size() < 2) continue;
        std::string irq = cols[0];
        if (!irq.empty() && irq.back() == ':') irq.pop_back();

        json item = json::object();
        item["number"] = irq;

        json counts = json::array();
        int64_t row_total = 0;
        size_t i = 1;
        while (i < cols.size() && counts.size() < cpu_n) {
            bool all_digit = !cols[i].empty();
            for (char c : cols[i]) if (!std::isdigit(static_cast<unsigned char>(c))) { all_digit = false; break; }
            if (!all_digit) break;
            try {
                int64_t v = std::stoll(cols[i]);
                counts.push_back(v);
                row_total += v;
            } catch (...) { counts.push_back(0); }
            ++i;
        }
        item["cpu_counts"] = std::move(counts);
        item["total"] = row_total;
        total_interrupts += row_total;

        std::string rest;
        while (i < cols.size()) { rest += cols[i]; rest += " "; ++i; }
        if (!rest.empty() && rest.back() == ' ') rest.pop_back();
        item["hardware_irq"] = rest;
        item["trigger"] = "";
        interrupts.push_back(std::move(item));
    }
    j["interrupts"] = std::move(interrupts);
    j["total_interrupts"] = total_interrupts;

    // Pick the top 10 interrupts by total count for the summary.
    std::vector<std::pair<int64_t, std::string>> tops;
    for (auto& it : j["interrupts"]) {
        tops.emplace_back(it["total"].get<int64_t>(), it["number"].get<std::string>());
    }
    std::sort(tops.begin(), tops.end(),
              [](auto& a, auto& b) { return a.first > b.first; });
    if (tops.size() > 10) tops.resize(10);
    json top_j = json::array();
    for (auto& kv : tops) {
        json e = json::object();
        e["number"] = kv.second;
        e["total"] = kv.first;
        top_j.push_back(std::move(e));
    }
    j["top_interrupts"] = std::move(top_j);
    return j;
}

HttpResponse handleInterrupts(const HttpRequest&) {
    return jsonResp(200, buildInterruptsJson());
}

// --- /api/systemd/services ------------------------------------------------
//
// FUN_0012fd28 / getSystemdServices(). Runs three systemctl commands and
// returns their raw output plus a parsed list of services with state.

json buildSystemdServicesJson() {
    json j = json::object();

    auto run = [](const char* cmd) -> std::string {
        if (auto out = execCapture(cmd)) return *out;
        return "";
    };
    std::string all_out   = run("systemctl list-units --type=service --all --no-pager --plain");
    std::string fail_out  = run("systemctl list-units --type=service --failed --no-pager --plain");
    std::string enab_out  = run("systemctl list-unit-files --type=service --state=enabled --no-pager --plain");

    j["all_services"] = all_out;
    j["failed_services"] = fail_out;
    j["enabled_services"] = enab_out;

    // Parse the "all services" output line-by-line. Rows look like:
    //   UNIT                    LOAD   ACTIVE   SUB     DESCRIPTION
    //   foo.service             loaded active   running Foo daemon
    json parsed = json::array();
    bool header_found = false;
    bool footer_reached = false;
    int parsed_count = 0;
    std::istringstream iss(all_out);
    std::string line;
    while (std::getline(iss, line)) {
        if (!header_found) {
            if (line.find("UNIT") != std::string::npos &&
                line.find("LOAD") != std::string::npos) {
                header_found = true;
            }
            continue;
        }
        if (line.empty()) { footer_reached = true; continue; }
        if (footer_reached) continue;
        if (line.find("units listed") != std::string::npos ||
            line.find("units loaded") != std::string::npos ||
            line.find("To show")      != std::string::npos) {
            footer_reached = true;
            continue;
        }
        auto cols = splitWs(line);
        if (cols.size() < 4) continue;
        if (cols[0].find(".service") == std::string::npos) continue;

        json s = json::object();
        s["name"] = cols[0];
        s["load"] = cols[1];
        s["active"] = cols[2];
        s["sub"] = cols[3];
        std::string desc;
        for (size_t i = 4; i < cols.size(); ++i) {
            if (i > 4) desc += " ";
            desc += cols[i];
        }
        s["description"] = desc;
        parsed.push_back(std::move(s));
        parsed_count++;
    }

    json parsing = json::object();
    parsing["header_found"] = header_found;
    parsing["footer_reached"] = footer_reached;
    parsing["valid_services_parsed"] = parsed_count;
    j["parsing_info"] = std::move(parsing);
    j["parsed_services"] = std::move(parsed);
    return j;
}

HttpResponse handleSystemdServices(const HttpRequest&) {
    return jsonResp(200, buildSystemdServicesJson());
}

// --- /api/pci -------------------------------------------------------------
//
// FUN_00130ba0 / getPciInfo(). Runs `lspci`, `lspci -v`, `lspci -t`.

json buildPciJson() {
    json j = json::object();
    if (auto out = execCapture("lspci")) j["devices"] = *out;
    else j["devices"] = "";
    if (auto out = execCapture("lspci -v")) j["verbose_info"] = *out;
    else j["verbose_info"] = "";
    if (auto out = execCapture("lspci -t")) j["tree"] = *out;
    else j["tree"] = "";

    // Parse the simple `lspci` output into structured entries.
    json parsed = json::array();
    if (j["devices"].is_string()) {
        std::istringstream iss(j["devices"].get<std::string>());
        std::string line;
        while (std::getline(iss, line)) {
            if (line.empty()) continue;
            size_t sp = line.find(' ');
            if (sp == std::string::npos) continue;
            json d = json::object();
            d["address"] = line.substr(0, sp);
            size_t colon = line.find(':', sp);
            if (colon != std::string::npos) {
                d["class"] = trim(line.substr(sp + 1, colon - sp - 1));
                d["description"] = trim(line.substr(colon + 1));
            } else {
                d["class"] = "";
                d["description"] = trim(line.substr(sp + 1));
            }
            parsed.push_back(std::move(d));
        }
    }
    j["parsed_devices"] = std::move(parsed);
    return j;
}

HttpResponse handlePci(const HttpRequest&) {
    return jsonResp(200, buildPciJson());
}

// --- /api/processes -------------------------------------------------------
//
// FUN_00123b20 / getProcesses(). Walks /proc/<pid>/ for numeric directories,
// reads /proc/<pid>/status, /proc/<pid>/stat, /proc/<pid>/io,
// /proc/<pid>/cmdline and builds a list plus aggregate counts. The strings
// at 0x16c548..0x16c6f8 name the fields that the web UI expects.

static int64_t readIoField(const std::string& pid, const char* key) {
    std::ifstream f("/proc/" + pid + "/io");
    if (!f.is_open()) return 0;
    std::string line;
    std::string prefix = std::string(key) + ":";
    while (std::getline(f, line)) {
        if (line.compare(0, prefix.size(), prefix) == 0) {
            auto parts = splitWs(line.substr(prefix.size()));
            if (!parts.empty()) {
                try { return std::stoll(parts[0]); } catch (...) {}
            }
            break;
        }
    }
    return 0;
}

json buildProcessesJson() {
    json j = json::object();
    json procs = json::array();
    int total = 0;

    DIR* d = ::opendir("/proc");
    if (d) {
        struct dirent* e;
        while ((e = ::readdir(d)) != nullptr) {
            std::string name(e->d_name);
            if (name.empty() || !std::isdigit(static_cast<unsigned char>(name[0]))) continue;
            bool all_digit = true;
            for (char c : name) if (!std::isdigit(static_cast<unsigned char>(c))) { all_digit = false; break; }
            if (!all_digit) continue;

            json p = json::object();
            p["pid"] = name;

            // /proc/<pid>/comm or /proc/<pid>/status Name:
            std::string comm;
            std::ifstream sf("/proc/" + name + "/status");
            if (sf.is_open()) {
                std::string line;
                while (std::getline(sf, line)) {
                    if (line.compare(0, 5, "Name:") == 0) {
                        comm = trim(line.substr(5));
                    } else if (line.compare(0, 7, "VmSize:") == 0) {
                        auto parts = splitWs(line.substr(7));
                        if (!parts.empty()) {
                            try { p["memory_vsz_bytes"] = std::stoll(parts[0]) * 1024; }
                            catch (...) { p["memory_vsz_bytes"] = 0; }
                        }
                    } else if (line.compare(0, 6, "VmRSS:") == 0) {
                        auto parts = splitWs(line.substr(6));
                        if (!parts.empty()) {
                            try { p["memory_rss_bytes"] = std::stoll(parts[0]) * 1024; }
                            catch (...) { p["memory_rss_bytes"] = 0; }
                        }
                    } else if (line.compare(0, 8, "Threads:") == 0) {
                        auto parts = splitWs(line.substr(8));
                        if (!parts.empty()) {
                            try { p["threads"] = std::stoi(parts[0]); }
                            catch (...) { p["threads"] = 0; }
                        }
                    } else if (line.compare(0, 6, "State:") == 0) {
                        auto parts = splitWs(line.substr(6));
                        if (!parts.empty()) p["state"] = parts[0];
                    }
                }
            }
            p["name"] = comm;

            // cmdline
            std::ifstream cf("/proc/" + name + "/cmdline");
            if (cf.is_open()) {
                std::string cmd((std::istreambuf_iterator<char>(cf)),
                                std::istreambuf_iterator<char>());
                for (char& c : cmd) if (c == '\0') c = ' ';
                while (!cmd.empty() && cmd.back() == ' ') cmd.pop_back();
                p["cmdline"] = cmd;
            }

            p["read_bytes"]  = readIoField(name, "read_bytes");
            p["write_bytes"] = readIoField(name, "write_bytes");
            p["read_ops"]    = readIoField(name, "syscr");
            p["write_ops"]   = readIoField(name, "syscw");

            // Open file descriptor count
            std::string fd_dir = "/proc/" + name + "/fd";
            int fd_count = 0;
            if (DIR* fdd = ::opendir(fd_dir.c_str())) {
                struct dirent* fe;
                while ((fe = ::readdir(fdd)) != nullptr) {
                    if (fe->d_name[0] != '.') fd_count++;
                }
                ::closedir(fdd);
            }
            p["file_descriptors"] = fd_count;

            procs.push_back(std::move(p));
            total++;
        }
        ::closedir(d);
    }

    j["processes"] = std::move(procs);
    j["total_processes"] = total;
    j["sampling_method"] = "single_sample";
    return j;
}

HttpResponse handleProcesses(const HttpRequest&) {
    return jsonResp(200, buildProcessesJson());
}

// --- /api/system/all ------------------------------------------------------
//
// FUN_00130908 / getSystemAll(). Aggregates disk, kernel, usb, pci,
// systemd services, processes, interrupts, and logs.

HttpResponse handleSystemAll(const HttpRequest&) {
    json j = json::object();
    j["disk"] = json::object();
    j["disk"]["info"]    = buildDiskInfoJson();
    j["disk"]["io"]      = buildDiskIoJson();
    j["disk"]["largest"] = buildDiskLargestJson();
    j["usb"]             = buildUsbJson();
    j["kernel"] = json::object();
    j["kernel"]["info"]       = buildKernelInfoJson();
    j["kernel"]["modules"]    = buildKernelModulesJson();
    j["kernel"]["ringbuffer"] = buildKernelRingbufferJson();
    j["kernel"]["dmesg"]      = buildDmesgJson(100);
    j["i2c"]           = buildI2cJson();
    j["interrupts"]    = buildInterruptsJson();
    j["systemd"]       = buildSystemdServicesJson();
    j["pci"]           = buildPciJson();
    j["processes"]     = buildProcessesJson();
    j["journal_stats"] = buildJournalStatsJson();
    j["boot_list"]     = buildBootListJson();
    return jsonResp(200, j);
}

}  // namespace limelight::hwmon
