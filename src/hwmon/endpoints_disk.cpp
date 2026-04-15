#include "endpoints.h"
#include "util.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/statvfs.h>
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

// --- /api/disk/io ---------------------------------------------------------
//
// getDiskIoStats() / FUN_0011e484. Parses /proc/diskstats into a list of
// { name, major, minor, reads_completed, ... } objects plus aggregate
// totals. Sector size is assumed to be 512 bytes (standard for
// /proc/diskstats).

json buildDiskIoJson() {
    json j = json::object();
    std::ifstream f("/proc/diskstats");
    if (!f.is_open()) {
        j["error"] = "Failed to open /proc/diskstats";
        return j;
    }

    json devices = json::array();
    int64_t total_reads = 0, total_writes = 0;
    int64_t total_bytes_read = 0, total_bytes_written = 0;

    std::string line;
    while (std::getline(f, line)) {
        auto parts = splitWs(line);
        // /proc/diskstats has 14+ columns (kernel >= 2.6.33).
        if (parts.size() < 14) continue;
        json d = json::object();
        try {
            d["major"]            = std::stoi(parts[0]);
            d["minor"]            = std::stoi(parts[1]);
            d["name"]             = parts[2];
            d["reads_completed"]  = std::stoll(parts[3]);
            d["reads_merged"]     = std::stoll(parts[4]);
            d["sectors_read"]     = std::stoll(parts[5]);
            d["read_time_ms"]     = std::stoll(parts[6]);
            d["writes_completed"] = std::stoll(parts[7]);
            d["writes_merged"]    = std::stoll(parts[8]);
            d["sectors_written"]  = std::stoll(parts[9]);
            d["write_time_ms"]    = std::stoll(parts[10]);
            d["ios_in_progress"]  = std::stoll(parts[11]);
            d["io_time_ms"]       = std::stoll(parts[12]);
            d["weighted_io_time_ms"] = std::stoll(parts[13]);

            int64_t sectors_r = std::stoll(parts[5]);
            int64_t sectors_w = std::stoll(parts[9]);
            int64_t br = sectors_r * 512;
            int64_t bw = sectors_w * 512;
            d["bytes_read"] = br;
            d["bytes_written"] = bw;

            total_reads  += std::stoll(parts[3]);
            total_writes += std::stoll(parts[7]);
            total_bytes_read += br;
            total_bytes_written += bw;
        } catch (...) {
            continue;
        }
        devices.push_back(std::move(d));
    }

    j["devices"] = std::move(devices);
    j["total_reads"] = total_reads;
    j["total_writes"] = total_writes;
    j["total_bytes_read"] = total_bytes_read;
    j["total_bytes_written"] = total_bytes_written;
    return j;
}

// --- /api/disk/info -------------------------------------------------------
//
// getDiskInfo() / FUN_0012c008. Parses /proc/mounts, filters out virtual
// filesystems (sysfs, devpts, tmpfs, devtmpfs, cgroup, pstore, /dev/loop,
// /snap/), calls statvfs() on each mountpoint, and for real block devices
// also reads /sys/block/<dev>/device/model and /sys/block/<dev>/size.

static bool isVirtualFs(const std::string& fstype) {
    return fstype == "sysfs" || fstype == "devpts" || fstype == "tmpfs" ||
           fstype == "devtmpfs" || fstype == "cgroup" || fstype == "pstore" ||
           fstype == "cgroup2" || fstype == "proc" || fstype == "securityfs" ||
           fstype == "debugfs" || fstype == "tracefs" || fstype == "fusectl" ||
           fstype == "configfs" || fstype == "bpf" || fstype == "autofs" ||
           fstype == "mqueue" || fstype == "hugetlbfs" || fstype == "rpc_pipefs";
}

json buildDiskInfoJson() {
    json j = json::object();
    json disks = json::array();

    int64_t total_storage = 0;
    int64_t total_used = 0;
    int64_t total_available = 0;
    int total_disks = 0;

    std::ifstream mf("/proc/mounts");
    std::string line;
    std::regex block_re("/dev/([a-z]+)\\d+");
    while (std::getline(mf, line)) {
        auto parts = splitWs(line);
        if (parts.size() < 3) continue;
        const std::string& dev = parts[0];
        const std::string& mp  = parts[1];
        const std::string& fs  = parts[2];

        if (isVirtualFs(fs)) continue;
        if (dev.rfind("/dev/loop", 0) == 0) continue;
        if (mp.find("/snap/") != std::string::npos) continue;

        struct statvfs sv {};
        if (::statvfs(mp.c_str(), &sv) != 0) continue;

        int64_t total = static_cast<int64_t>(sv.f_blocks) * static_cast<int64_t>(sv.f_frsize);
        int64_t avail = static_cast<int64_t>(sv.f_bavail) * static_cast<int64_t>(sv.f_frsize);
        int64_t used  = total - static_cast<int64_t>(sv.f_bfree) *
                                    static_cast<int64_t>(sv.f_frsize);

        json d = json::object();
        d["device"] = dev;
        d["mountpoint"] = mp;
        d["filesystem"] = fs;
        d["total_bytes"] = total;
        d["used_bytes"] = used;
        d["available_bytes"] = avail;

        // Derive /sys/block/<dev> name for physical-disk lookup.
        std::smatch m;
        bool is_physical = false;
        int64_t physical_size = 0;
        if (std::regex_search(dev, m, block_re) && m.size() >= 2) {
            std::string base = m[1].str();
            std::string model_path = "/sys/block/" + base + "/device/model";
            std::string size_path  = "/sys/block/" + base + "/size";
            if (auto model = readFileLine(model_path)) {
                d["device_model"] = *model;
                is_physical = true;
            }
            if (auto size_str = readFileLine(size_path)) {
                try {
                    physical_size = std::stoll(*size_str) * 512;
                    d["physical_size_bytes"] = physical_size;
                } catch (...) {}
            }
        }
        d["is_physical_disk"] = is_physical;

        total_storage   += total;
        total_used      += used;
        total_available += avail;
        total_disks++;

        disks.push_back(std::move(d));
    }

    j["disks"] = std::move(disks);
    j["total_disks"] = total_disks;
    j["total_storage_bytes"] = total_storage;
    j["total_used_bytes"] = total_used;
    j["total_available_bytes"] = total_available;
    if (total_storage > 0) {
        double pct = (double(total_used) / double(total_storage)) * 100.0;
        j["overall_utilization_percent"] = pct;
    } else {
        j["overall_utilization_percent"] = 0.0;
    }
    return j;
}

// --- /api/disk/largest ----------------------------------------------------
//
// getLargestFiles() / FUN_00128a08. Walks the filesystem from the given
// ?root= (default "/") looking for the largest files/directories by size.
// The original supports ?minsize= and ?maxresults= params, plus a default
// set of exclusions. We implement the common case: "find / -type f -size
// +<min> -printf '%s %p\n' | sort -n | tail -<max>".

json buildDiskLargestJson() {
    json j = json::object();

    std::string scan_root = "/";
    int64_t min_size = 1024 * 1024;  // 1 MiB default
    int max_results = 20;

    j["scan_root"] = scan_root;
    j["max_results"] = max_results;
    j["min_size_bytes"] = min_size;

    // Collect largest files via a single find|sort|tail pipeline. This is
    // the observable behavior from the strings table.
    std::string files_cmd =
        "find " + scan_root +
        " -xdev -type f -size +" + std::to_string(min_size / 1024) + "k "
        "-not -path '*/proc/*' -not -path '*/sys/*' -not -path '*/dev/*' "
        "-printf '%s\\t%T@\\t%U\\t%m\\t%p\\n' 2>/dev/null "
        "| sort -n -r | head -" + std::to_string(max_results);
    json files = json::array();
    if (auto out = execCapture(files_cmd)) {
        std::istringstream iss(*out);
        std::string line;
        while (std::getline(iss, line)) {
            auto cols = split(line, '\t');
            if (cols.size() < 5) continue;
            json f = json::object();
            try { f["size_bytes"] = std::stoll(cols[0]); }
            catch (...) { f["size_bytes"] = 0; }
            f["modified_time"] = cols[1];
            f["owner"] = cols[2];
            f["permissions"] = cols[3];
            f["path"] = cols[4];
            f["is_directory"] = false;
            files.push_back(std::move(f));
        }
    }
    j["largest_files"] = std::move(files);

    // Largest directories: du --max-depth=2 -x / | sort -nr | head
    std::string dirs_cmd = "du -b --max-depth=2 -x " + scan_root +
                           " 2>/dev/null | sort -n -r | head -" +
                           std::to_string(max_results);
    json dirs = json::array();
    if (auto out = execCapture(dirs_cmd)) {
        std::istringstream iss(*out);
        std::string line;
        while (std::getline(iss, line)) {
            auto cols = splitWs(line);
            if (cols.size() < 2) continue;
            json d = json::object();
            try { d["size_bytes"] = std::stoll(cols[0]); }
            catch (...) { d["size_bytes"] = 0; }
            std::string path;
            for (size_t i = 1; i < cols.size(); ++i) {
                if (i > 1) path += " ";
                path += cols[i];
            }
            d["path"] = path;
            d["is_directory"] = true;
            dirs.push_back(std::move(d));
        }
    }
    j["largest_directories"] = std::move(dirs);
    return j;
}

// --- /api/disk/eject ------------------------------------------------------
//
// POST /api/disk/eject with a JSON body { "device": "/dev/sda1" }. The
// original (FUN_0010cf40) does a `sudo umount <dev>` followed by `sudo
// eject <dev>` and returns the combined output. Device names must start
// with "/dev/" (strings at 0x16bf38).

static bool looksLikeValidDevice(const std::string& dev) {
    if (dev.compare(0, 5, "/dev/") != 0) return false;
    // Allow only [A-Za-z0-9._/-] to keep this safe from shell injection.
    for (char c : dev) {
        const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                        (c >= '0' && c <= '9') || c == '.' || c == '_' ||
                        c == '-' || c == '/';
        if (!ok) return false;
    }
    return true;
}

HttpResponse handleDiskEject(const HttpRequest& req) {
    json resp = json::object();
    try {
        auto body = json::parse(req.body);
        if (!body.contains("device")) {
            resp["success"] = false;
            resp["error"] = "Missing 'device' field";
            return jsonResp(400, resp);
        }
        std::string dev = body["device"].get<std::string>();
        if (!looksLikeValidDevice(dev)) {
            resp["success"] = false;
            resp["error"] = "Invalid device name";
            return jsonResp(400, resp);
        }

        struct stat st {};
        if (::stat(dev.c_str(), &st) != 0) {
            resp["success"] = false;
            resp["error"] = std::string("Device not found: ") + dev;
            return jsonResp(404, resp);
        }

        std::string unmount_cmd = "sudo umount " + dev + " 2>&1";
        std::string eject_cmd   = "sudo eject " + dev + " 2>&1";

        int unmount_rc = 0;
        int eject_rc = 0;
        auto unmount_out = execCapture(unmount_cmd, &unmount_rc).value_or("");
        auto eject_out   = execCapture(eject_cmd,   &eject_rc).value_or("");

        // Interpret common error strings (from the strings table).
        auto contains = [](const std::string& hay, const char* needle) {
            return hay.find(needle) != std::string::npos;
        };
        if (contains(unmount_out, "not found") || contains(eject_out, "not found") ||
            contains(unmount_out, "No such file") || contains(eject_out, "No such file")) {
            resp["success"] = false;
            resp["error"] = "Device not found";
        } else if (contains(unmount_out, "target is busy") ||
                   contains(eject_out,   "target is busy")) {
            resp["success"] = false;
            resp["error"] = "Device is busy - some processes may still be using it";
        } else if (contains(unmount_out, "Permission denied") ||
                   contains(eject_out,   "Permission denied")) {
            resp["success"] = false;
            resp["error"] = "Permission denied - check sudo configuration";
        } else if (contains(eject_out, "not ejectable")) {
            resp["success"] = false;
            resp["error"] = "Device is not ejectable";
        } else {
            resp["success"] = true;
            resp["message"] = std::string("Device ") + dev + " ejected successfully";
        }
        resp["device"] = dev;
        resp["unmount_output"] = unmount_out;
        resp["eject_output"] = eject_out;
    } catch (const std::exception& e) {
        resp["success"] = false;
        resp["error"] = std::string("Exception: ") + e.what();
        return jsonResp(500, resp);
    }
    return jsonResp(200, resp);
}

HttpResponse handleDiskEjectOptions(const HttpRequest&) {
    HttpResponse r;
    r.status = 204;
    r.content_type = "application/json";
    r.body = "";
    r.extra_headers["Access-Control-Allow-Methods"] = "POST, OPTIONS";
    r.extra_headers["Access-Control-Allow-Headers"] = "Content-Type";
    return r;
}

HttpResponse handleDiskInfo(const HttpRequest&) {
    return jsonResp(200, buildDiskInfoJson());
}

HttpResponse handleDiskLargest(const HttpRequest&) {
    return jsonResp(200, buildDiskLargestJson());
}

HttpResponse handleDiskIo(const HttpRequest&) {
    return jsonResp(200, buildDiskIoJson());
}

}  // namespace limelight::hwmon
