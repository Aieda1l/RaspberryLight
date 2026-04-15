// Limelight visionserver REST API implementation.
//
// Recreated from the route table at .rodata 0x003191b0..0x003194e0 in the
// original visionserver binary — see
// .ghidra_work/notes/visionserver_rest_endpoints.md for the full map. All 41
// routes are registered here in the same order they appear in the .rodata
// string pool, which preserves the original registration order one-to-one.
//
// Staging directories referenced by the handlers (also from the notes file):
//   /var/www/html/snapshots/           — snapshot GET/POST/DELETE
//   /var/www/html/calibration/         — cal-* endpoints
//   /usr/local/bin/visionserver/recordings  — recording-* endpoints
//   /tmp/llpack_upload_*               — pipeline bundle upload staging
//   /tmp/llpack_extract_*              — pipeline bundle extract staging
//   /tmp/llpack_validate_*             — pipeline bundle validation staging
//   /tmp/llpack_manifest_*             — pipeline bundle manifest staging

#include "visionserver/rest_api.h"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;
using nlohmann::json;

namespace limelight {

namespace {

// Paths the original binary uses. Keep them as string constants here so
// every handler that touches disk references the same canonical location.
constexpr const char* kSnapshotDir  = "/var/www/html/snapshots";
constexpr const char* kCalDir       = "/var/www/html/calibration";
constexpr const char* kRecordingDir = "/usr/local/bin/visionserver/recordings";
constexpr const char* kHwReportPath = "/sys/devices/virtual/thermal/thermal_zone0/temp";
constexpr const char* kEepromPath   = "/sys/class/i2c-adapter/i2c-10/10-0050/eeprom";
constexpr const char* kHostnamePath = "/etc/hostname";

// ---- Utility helpers ------------------------------------------------------

// URL-decode an application/x-www-form-urlencoded or query string value.
std::string urlDecode(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        char c = s[i];
        if (c == '+') { out.push_back(' '); }
        else if (c == '%' && i + 2 < s.size()) {
            auto hex = [](char h) -> int {
                if (h >= '0' && h <= '9') return h - '0';
                if (h >= 'a' && h <= 'f') return h - 'a' + 10;
                if (h >= 'A' && h <= 'F') return h - 'A' + 10;
                return 0;
            };
            int v = (hex(s[i + 1]) << 4) | hex(s[i + 2]);
            out.push_back(static_cast<char>(v));
            i += 2;
        } else {
            out.push_back(c);
        }
    }
    return out;
}

// Parse a key=value query string into a map.
std::unordered_map<std::string, std::string> parseQuery(const std::string& q) {
    std::unordered_map<std::string, std::string> out;
    size_t i = 0;
    while (i < q.size()) {
        size_t amp = q.find('&', i);
        if (amp == std::string::npos) amp = q.size();
        std::string pair = q.substr(i, amp - i);
        size_t eq = pair.find('=');
        if (eq == std::string::npos) {
            out[urlDecode(pair)] = "";
        } else {
            out[urlDecode(pair.substr(0, eq))] = urlDecode(pair.substr(eq + 1));
        }
        i = amp + 1;
    }
    return out;
}

// Get an integer query parameter with a default. Tolerates parse failure.
int queryInt(const std::unordered_map<std::string, std::string>& q,
             const std::string& key, int fallback = 0) {
    auto it = q.find(key);
    if (it == q.end()) return fallback;
    try { return std::stoi(it->second); }
    catch (...) { return fallback; }
}

// Get a string query parameter.
std::string queryStr(const std::unordered_map<std::string, std::string>& q,
                     const std::string& key, const std::string& fallback = "") {
    auto it = q.find(key);
    return it == q.end() ? fallback : it->second;
}

// Read a file to a string. Returns empty on failure.
std::string slurpFile(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    if (!in) return "";
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

bool writeFile(const fs::path& p, const std::string& data) {
    std::error_code ec;
    fs::create_directories(p.parent_path(), ec);
    std::ofstream out(p, std::ios::binary);
    if (!out) return false;
    out.write(data.data(), static_cast<std::streamsize>(data.size()));
    return static_cast<bool>(out);
}

HttpResponse jsonOk(const json& j) {
    HttpResponse r;
    r.status = 200;
    r.content_type = "application/json";
    r.body = j.dump();
    return r;
}

HttpResponse jsonError(int status, const std::string& msg) {
    HttpResponse r;
    r.status = status;
    r.content_type = "application/json";
    r.body = json{{"error", msg}}.dump();
    return r;
}

HttpResponse textOk(const std::string& body,
                    const std::string& ctype = "text/plain") {
    HttpResponse r;
    r.status = 200;
    r.content_type = ctype;
    r.body = body;
    return r;
}

HttpResponse binaryOk(std::string body, const std::string& ctype) {
    HttpResponse r;
    r.status = 200;
    r.content_type = ctype;
    r.body = std::move(body);
    return r;
}

// Simple JSON serialization of a PipelineResult matching the shape the web
// UI expects (`/results` key names come from the original JSON the binary
// publishes to NT's `json` topic — see visionserver_nt_topics.md).
json resultToJson(const PipelineResult& r) {
    json j;
    j["tv"] = r.has_target ? 1 : 0;
    j["tx"] = r.tx;
    j["ty"] = r.ty;
    j["ta"] = r.ta;
    j["ts"] = r.ts;
    j["tid"] = r.tid;
    j["tclass"] = r.tclass;
    if (!r.botpose.empty())       j["botpose"]       = r.botpose;
    if (!r.botpose_blue.empty())  j["botpose_wpiblue"] = r.botpose_blue;
    if (!r.botpose_red.empty())   j["botpose_wpired"]  = r.botpose_red;
    if (!r.targetpose_robotspace.empty())
        j["targetpose_robotspace"] = r.targetpose_robotspace;
    if (!r.targetpose_cameraspace.empty())
        j["targetpose_cameraspace"] = r.targetpose_cameraspace;
    if (!r.corners.empty()) {
        json arr = json::array();
        for (const auto& c : r.corners) arr.push_back(c);
        j["corners"] = std::move(arr);
    }
    json lp = json::array();
    for (double v : r.llpython) lp.push_back(v);
    j["llpython"] = std::move(lp);
    return j;
}

}  // namespace

// ---- RestApi --------------------------------------------------------------

RestApi::RestApi(PipelineManager& mgr, GlobalSettings& settings)
    : mgr_(mgr), settings_(settings), server_(std::make_unique<HttpServer>()) {
    registerRoutes();
}

RestApi::~RestApi() {
    stop();
}

void RestApi::setCallbacks(Callbacks cbs) {
    cbs_ = std::move(cbs);
}

void RestApi::updateLiveState(const RestLiveState& s) {
    std::lock_guard<std::mutex> lk(state_mu_);
    state_ = s;
}

bool RestApi::start(uint16_t port) {
    return server_->start(port);
}

void RestApi::stop() {
    if (server_) server_->stop();
}

// --- Route table -----------------------------------------------------------

void RestApi::registerRoutes() {
    auto& s = *server_;

    // Health + status + state dumps
    s.get ("/status",             [this](const HttpRequest& r){ return handleStatus(r); });
    s.get ("/results",            [this](const HttpRequest& r){ return handleResults(r); });
    s.get ("/dump",               [this](const HttpRequest& r){ return handleDump(r); });
    s.get ("/dumpd",              [this](const HttpRequest& r){ return handleDumpd(r); });
    s.get ("/hwreport",           [this](const HttpRequest& r){ return handleHwReport(r); });

    // Snapshots
    s.get ("/snapshotmanifest",   [this](const HttpRequest& r){ return handleSnapshotManifest(r); });
    s.post("/upload-snapshot",    [this](const HttpRequest& r){ return handleUploadSnapshot(r); });
    s.post("/capture-snapshot",   [this](const HttpRequest& r){ return handleCaptureSnapshot(r); });
    s.post("/delete-snapshots",   [this](const HttpRequest& r){ return handleDeleteSnapshots(r); });
    s.post("/delete-snapshot",    [this](const HttpRequest& r){ return handleDeleteSnapshot(r); });

    // Videos / recordings
    s.get ("/videolist",          [this](const HttpRequest& r){ return handleVideoList(r); });
    s.post("/delete-video",       [this](const HttpRequest& r){ return handleDeleteVideo(r); });
    s.post("/delete-videos",      [this](const HttpRequest& r){ return handleDeleteVideos(r); });
    s.get ("/recording-list",     [this](const HttpRequest& r){ return handleRecordingList(r); });
    s.get ("/recording/*",        [this](const HttpRequest& r){ return handleRecordingStream(r); });
    s.post("/recording-flush",    [this](const HttpRequest& r){ return handleRecordingFlush(r); });

    // Pipelines
    s.post("/reload-pipeline",    [this](const HttpRequest& r){ return handleReloadPipeline(r); });
    s.get ("/getsnapscriptnames", [this](const HttpRequest& r){ return handleGetSnapScriptNames(r); });
    s.post("/pipeline-default",   [this](const HttpRequest& r){ return handlePipelineDefault(r); });
    s.get ("/pipeline-atindex",   [this](const HttpRequest& r){ return handlePipelineAtIndex(r); });
    s.post("/pipeline-switch",    [this](const HttpRequest& r){ return handlePipelineSwitch(r); });
    s.post("/upload-pipeline",    [this](const HttpRequest& r){ return handleUploadPipeline(r); });
    s.post("/upload-fieldmap",    [this](const HttpRequest& r){ return handleUploadFieldmap(r); });
    s.post("/upload-nn",          [this](const HttpRequest& r){ return handleUploadNn(r); });
    s.post("/upload-nnlabels",    [this](const HttpRequest& r){ return handleUploadNnLabels(r); });
    s.post("/upload-python",      [this](const HttpRequest& r){ return handleUploadPython(r); });

    // Pipeline updates (PATCH for mutation matches the web UI's XHR verb)
    s.post ("/update-pipeline",        [this](const HttpRequest& r){ return handleUpdatePipeline(r); });
    s.patch("/update-pipeline",        [this](const HttpRequest& r){ return handleUpdatePipeline(r); });
    s.post ("/update-pythoninputs",    [this](const HttpRequest& r){ return handleUpdatePythonInputs(r); });
    s.post ("/update-robotorientation",[this](const HttpRequest& r){ return handleUpdateRobotOrientation(r); });
    s.post ("/update-imumode",         [this](const HttpRequest& r){ return handleUpdateImuMode(r); });
    s.post ("/update-throttle",        [this](const HttpRequest& r){ return handleUpdateThrottle(r); });
    s.post ("/update-imuassistalpha",  [this](const HttpRequest& r){ return handleUpdateImuAssistAlpha(r); });

    // Calibration
    s.get ("/cal-default",        [this](const HttpRequest& r){ return handleCalDefault(r); });
    s.get ("/cal-file",           [this](const HttpRequest& r){ return handleCalFile(r); });
    s.get ("/cal-eeprom",         [this](const HttpRequest& r){ return handleCalEeprom(r); });
    s.get ("/cal-latest",         [this](const HttpRequest& r){ return handleCalLatest(r); });
    s.get ("/cal-pointcloud",     [this](const HttpRequest& r){ return handleCalPointcloud(r); });
    s.get ("/cal-stats",          [this](const HttpRequest& r){ return handleCalStats(r); });

    // Pipeline bundle API (llpack archives)
    s.get ("/api/pipeline-bundle/download", [this](const HttpRequest& r){ return handleBundleDownload(r); });
    s.post("/api/pipeline-bundle/upload",   [this](const HttpRequest& r){ return handleBundleUpload(r); });
    s.post("/api/pipeline-bundle/validate", [this](const HttpRequest& r){ return handleBundleValidate(r); });
}

// ---- Handlers -------------------------------------------------------------

HttpResponse RestApi::handleStatus(const HttpRequest&) {
    RestLiveState s;
    { std::lock_guard<std::mutex> lk(state_mu_); s = state_; }
    const auto& cfg = mgr_.getActiveConfig();
    json j;
    j["status"]     = "ok";
    j["pipeline"]   = s.active_pipeline_index;
    j["pipeline_type"] = cfg.pipeline_type;
    j["pipeline_desc"] = cfg.desc;
    j["cl_ms"]      = s.capture_latency_ms;
    j["tl_ms"]      = s.pipeline_latency_ms;
    j["heartbeat"]  = s.heartbeat;
    j["team"]       = settings_.team_number;
    j["nt_server"]  = settings_.nt_server;
    return jsonOk(j);
}

HttpResponse RestApi::handleResults(const HttpRequest&) {
    RestLiveState s;
    { std::lock_guard<std::mutex> lk(state_mu_); s = state_; }
    json j = resultToJson(s.latest);
    j["pipelineIndex"] = s.active_pipeline_index;
    j["cl"]  = s.capture_latency_ms;
    j["tl"]  = s.pipeline_latency_ms;
    j["hb"]  = s.heartbeat;
    return jsonOk(j);
}

HttpResponse RestApi::handleDump(const HttpRequest&) {
    // Full state dump: settings + all 10 pipeline configs + latest result.
    json j;
    j["settings"] = {
        {"team_number",             settings_.team_number},
        {"default_vision_profile",  settings_.default_vision_profile},
        {"preferred_calibration",   settings_.preferred_calibration},
        {"stream_quality",          settings_.stream_quality},
        {"stream_rate",             settings_.stream_rate},
        {"stream_res",              settings_.stream_res},
        {"modbus_port",             settings_.modbus_port},
        {"nt_server",               settings_.nt_server},
        {"usb_id",                  settings_.usb_id},
    };
    json pipes = json::array();
    for (int i = 0; i < NUM_PIPELINES; ++i) {
        json pj;
        to_json(pj, mgr_.getConfig(i));
        pipes.push_back(std::move(pj));
    }
    j["pipelines"] = std::move(pipes);

    RestLiveState s;
    { std::lock_guard<std::mutex> lk(state_mu_); s = state_; }
    j["result"] = resultToJson(s.latest);
    j["active"] = s.active_pipeline_index;
    return jsonOk(j);
}

HttpResponse RestApi::handleDumpd(const HttpRequest& req) {
    // Diagnostic dump: same as /dump plus runtime tracebacks / env. We have
    // no live tracebacks to emit in the recreation, so enrich with timing
    // and build info instead.
    HttpResponse base = handleDump(req);
    json j = json::parse(base.body);
    j["build"] = {
        {"version",  "1.0.0"},
        {"platform",
#ifdef PLATFORM_OPI5
            "opi5"
#elif defined(PLATFORM_RPI)
            "rpi"
#else
            "unknown"
#endif
        },
    };
    return jsonOk(j);
}

HttpResponse RestApi::handleHwReport(const HttpRequest&) {
    // Thermal + hostname + EEPROM serial — the fields the hwreport endpoint
    // returns in the original firmware. All three files are tolerated to be
    // missing on a dev box.
    json j;
    j["hostname"] = [] {
        std::string h = slurpFile(kHostnamePath);
        while (!h.empty() && (h.back() == '\n' || h.back() == '\r')) h.pop_back();
        return h;
    }();
    {
        std::string t = slurpFile(kHwReportPath);
        try { j["cpu_temp_millideg"] = std::stoll(t); }
        catch (...) { j["cpu_temp_millideg"] = nullptr; }
    }
    {
        std::string eeprom = slurpFile(kEepromPath);
        // EEPROM is binary; just report the first ~64 hex bytes as a serial.
        std::string hex;
        for (size_t i = 0; i < std::min<size_t>(eeprom.size(), 64); ++i) {
            char buf[3];
            std::snprintf(buf, sizeof(buf), "%02x", static_cast<unsigned char>(eeprom[i]));
            hex.append(buf, 2);
        }
        j["eeprom_hex"] = hex;
    }
    return jsonOk(j);
}

// ---- Snapshots ------------------------------------------------------------

HttpResponse RestApi::handleSnapshotManifest(const HttpRequest&) {
    json arr = json::array();
    std::error_code ec;
    if (fs::exists(kSnapshotDir, ec)) {
        for (auto& e : fs::directory_iterator(kSnapshotDir, ec)) {
            if (!e.is_regular_file(ec)) continue;
            arr.push_back(e.path().filename().string());
        }
    }
    std::sort(arr.begin(), arr.end());
    return jsonOk(arr);
}

HttpResponse RestApi::handleUploadSnapshot(const HttpRequest& req) {
    auto q = parseQuery(req.query);
    std::string name = queryStr(q, "name");
    if (name.empty()) {
        // Fall back to a timestamp-based name.
        auto now = std::chrono::system_clock::now().time_since_epoch();
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
        name = "snap_" + std::to_string(ms) + ".jpg";
    }
    if (cbs_.uploadSnapshot) {
        if (!cbs_.uploadSnapshot(name, req.body)) {
            return jsonError(500, "upload failed");
        }
    } else {
        fs::path p = fs::path(kSnapshotDir) / name;
        if (!writeFile(p, req.body)) return jsonError(500, "write failed");
    }
    return jsonOk({{"status", "ok"}, {"name", name}});
}

HttpResponse RestApi::handleCaptureSnapshot(const HttpRequest&) {
    if (!cbs_.captureSnapshot) return jsonError(500, "no capture callback");
    if (!cbs_.captureSnapshot()) return jsonError(500, "capture failed");
    return jsonOk({{"status", "ok"}});
}

HttpResponse RestApi::handleDeleteSnapshots(const HttpRequest&) {
    if (cbs_.deleteAllSnapshots) cbs_.deleteAllSnapshots();
    else {
        std::error_code ec;
        for (auto& e : fs::directory_iterator(kSnapshotDir, ec)) {
            if (e.is_regular_file(ec)) fs::remove(e.path(), ec);
        }
    }
    return jsonOk({{"status", "ok"}});
}

HttpResponse RestApi::handleDeleteSnapshot(const HttpRequest& req) {
    auto q = parseQuery(req.query);
    std::string name = queryStr(q, "name");
    if (name.empty()) return jsonError(400, "missing name");
    // Path traversal guard — only allow a bare filename.
    if (name.find('/') != std::string::npos || name.find("..") != std::string::npos) {
        return jsonError(400, "invalid name");
    }
    bool ok = false;
    if (cbs_.deleteSnapshot) ok = cbs_.deleteSnapshot(name);
    else {
        std::error_code ec;
        ok = fs::remove(fs::path(kSnapshotDir) / name, ec);
    }
    return ok ? jsonOk({{"status", "ok"}}) : jsonError(500, "delete failed");
}

// ---- Videos ---------------------------------------------------------------

HttpResponse RestApi::handleVideoList(const HttpRequest&) {
    json arr = json::array();
    std::error_code ec;
    if (fs::exists(kRecordingDir, ec)) {
        for (auto& e : fs::directory_iterator(kRecordingDir, ec)) {
            if (e.is_regular_file(ec)) {
                arr.push_back(e.path().filename().string());
            }
        }
    }
    std::sort(arr.begin(), arr.end());
    return jsonOk(arr);
}

HttpResponse RestApi::handleDeleteVideo(const HttpRequest& req) {
    auto q = parseQuery(req.query);
    std::string name = queryStr(q, "name");
    if (name.empty()) return jsonError(400, "missing name");
    if (name.find('/') != std::string::npos || name.find("..") != std::string::npos) {
        return jsonError(400, "invalid name");
    }
    bool ok = false;
    if (cbs_.deleteVideo) ok = cbs_.deleteVideo(name);
    else {
        std::error_code ec;
        ok = fs::remove(fs::path(kRecordingDir) / name, ec);
    }
    return ok ? jsonOk({{"status", "ok"}}) : jsonError(500, "delete failed");
}

HttpResponse RestApi::handleDeleteVideos(const HttpRequest&) {
    if (cbs_.deleteAllVideos) cbs_.deleteAllVideos();
    else {
        std::error_code ec;
        for (auto& e : fs::directory_iterator(kRecordingDir, ec)) {
            if (e.is_regular_file(ec)) fs::remove(e.path(), ec);
        }
    }
    return jsonOk({{"status", "ok"}});
}

HttpResponse RestApi::handleRecordingList(const HttpRequest& req) {
    return handleVideoList(req);
}

HttpResponse RestApi::handleRecordingStream(const HttpRequest& req) {
    // `/recording/<name>` — stream one file directly. We use the wildcard
    // remainder placed into req.params["*"] by the router.
    auto it = req.params.find("*");
    if (it == req.params.end() || it->second.empty()) {
        return jsonError(404, "no recording specified");
    }
    const std::string& rel = it->second;
    if (rel.find("..") != std::string::npos || (!rel.empty() && rel[0] == '/') || fs::path(rel).is_absolute()) {
        return jsonError(400, "invalid path");
    }
    fs::path p = fs::path(kRecordingDir) / rel;
    std::error_code ec;
    if (!fs::exists(p, ec) || !fs::is_regular_file(p, ec)) {
        return jsonError(404, "not found");
    }
    // Pick content type from extension.
    std::string ext = p.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    std::string ctype = "application/octet-stream";
    if (ext == ".mp4")      ctype = "video/mp4";
    else if (ext == ".mkv") ctype = "video/x-matroska";
    else if (ext == ".avi") ctype = "video/x-msvideo";
    else if (ext == ".jpg" || ext == ".jpeg") ctype = "image/jpeg";
    else if (ext == ".png") ctype = "image/png";
    return binaryOk(slurpFile(p), ctype);
}

HttpResponse RestApi::handleRecordingFlush(const HttpRequest&) {
    if (cbs_.recordingFlush) cbs_.recordingFlush();
    return jsonOk({{"status", "ok"}});
}

// ---- Pipelines ------------------------------------------------------------

HttpResponse RestApi::handleReloadPipeline(const HttpRequest&) {
    if (cbs_.reloadPipeline) cbs_.reloadPipeline();
    return jsonOk({{"status", "ok"}});
}

HttpResponse RestApi::handleGetSnapScriptNames(const HttpRequest&) {
    // Enumerate Python scripts under extern/pipe*/python/
    json arr = json::array();
    std::error_code ec;
    for (int i = 0; i < NUM_PIPELINES; ++i) {
        fs::path pydir = fs::path("extern") / ("pipe" + std::to_string(i)) / "python";
        if (!fs::exists(pydir, ec)) continue;
        for (auto& e : fs::directory_iterator(pydir, ec)) {
            if (!e.is_regular_file(ec)) continue;
            if (e.path().extension() == ".py") {
                arr.push_back(json{
                    {"pipeline", i},
                    {"name",     e.path().filename().string()},
                });
            }
        }
    }
    return jsonOk(arr);
}

HttpResponse RestApi::handlePipelineDefault(const HttpRequest&) {
    if (cbs_.pipelineDefault) cbs_.pipelineDefault();
    return jsonOk({{"status", "ok"}});
}

HttpResponse RestApi::handlePipelineAtIndex(const HttpRequest& req) {
    auto q = parseQuery(req.query);
    int idx = queryInt(q, "index", -1);
    if (idx < 0 || idx >= NUM_PIPELINES) return jsonError(400, "bad index");
    json j;
    to_json(j, mgr_.getConfig(idx));
    return jsonOk(j);
}

HttpResponse RestApi::handlePipelineSwitch(const HttpRequest& req) {
    auto q = parseQuery(req.query);
    int idx = queryInt(q, "index", -1);
    if (idx < 0 || idx >= NUM_PIPELINES) return jsonError(400, "bad index");
    if (cbs_.switchPipeline) cbs_.switchPipeline(idx);
    else mgr_.setActivePipeline(idx);
    return jsonOk({{"status", "ok"}, {"index", idx}});
}

// ---- File uploads ---------------------------------------------------------

HttpResponse RestApi::handleUploadPipeline(const HttpRequest& req) {
    auto q = parseQuery(req.query);
    int idx = queryInt(q, "index", -1);
    if (idx < 0 || idx >= NUM_PIPELINES) return jsonError(400, "bad index");
    if (cbs_.uploadPipeline) {
        if (!cbs_.uploadPipeline(idx, req.body)) return jsonError(500, "upload failed");
    } else {
        fs::path p = std::to_string(idx) + ".vpr";
        if (!writeFile(p, req.body)) return jsonError(500, "write failed");
    }
    return jsonOk({{"status", "ok"}, {"index", idx}});
}

HttpResponse RestApi::handleUploadFieldmap(const HttpRequest& req) {
    auto q = parseQuery(req.query);
    int idx = queryInt(q, "index", -1);
    if (idx < 0 || idx >= NUM_PIPELINES) return jsonError(400, "bad index");
    if (cbs_.uploadFieldmap) {
        if (!cbs_.uploadFieldmap(idx, req.body)) return jsonError(500, "upload failed");
    } else {
        fs::path p = fs::path("extern") / ("pipe" + std::to_string(idx)) / "fmap.fmap";
        if (!writeFile(p, req.body)) return jsonError(500, "write failed");
    }
    return jsonOk({{"status", "ok"}, {"index", idx}});
}

HttpResponse RestApi::handleUploadNn(const HttpRequest& req) {
    auto q = parseQuery(req.query);
    int idx = queryInt(q, "index", -1);
    std::string kind = queryStr(q, "type", "detector");
    if (idx < 0 || idx >= NUM_PIPELINES) return jsonError(400, "bad index");
    if (cbs_.uploadNn) {
        if (!cbs_.uploadNn(idx, req.body)) return jsonError(500, "upload failed");
    } else {
        const char* base = kind == "classifier" ? "classifier/classifier"
                                                : "detector/detector";
        // Extension comes from Content-Type of the body.
        std::string ext = ".tflite";
        auto ct = req.headers.find("content-type");
        if (ct != req.headers.end() && ct->second.find("hef") != std::string::npos)
            ext = ".hef";
        fs::path p = fs::path("extern") / ("pipe" + std::to_string(idx))
                     / (std::string(base) + ext);
        if (!writeFile(p, req.body)) return jsonError(500, "write failed");
    }
    return jsonOk({{"status", "ok"}, {"index", idx}, {"type", kind}});
}

HttpResponse RestApi::handleUploadNnLabels(const HttpRequest& req) {
    auto q = parseQuery(req.query);
    int idx = queryInt(q, "index", -1);
    std::string kind = queryStr(q, "type", "detector");
    if (idx < 0 || idx >= NUM_PIPELINES) return jsonError(400, "bad index");
    if (cbs_.uploadNnLabels) {
        if (!cbs_.uploadNnLabels(idx, req.body)) return jsonError(500, "upload failed");
    } else {
        const char* base = kind == "classifier" ? "classifier/classifierlabels.txt"
                                                : "detector/detectorlabels.txt";
        fs::path p = fs::path("extern") / ("pipe" + std::to_string(idx)) / base;
        if (!writeFile(p, req.body)) return jsonError(500, "write failed");
    }
    return jsonOk({{"status", "ok"}, {"index", idx}, {"type", kind}});
}

HttpResponse RestApi::handleUploadPython(const HttpRequest& req) {
    auto q = parseQuery(req.query);
    int idx = queryInt(q, "index", -1);
    std::string name = queryStr(q, "name", "pythonScript.py");
    if (idx < 0 || idx >= NUM_PIPELINES) return jsonError(400, "bad index");
    if (name.find('/') != std::string::npos || name.find("..") != std::string::npos)
        return jsonError(400, "invalid name");
    if (cbs_.uploadPython) {
        if (!cbs_.uploadPython(idx, req.body)) return jsonError(500, "upload failed");
    } else {
        fs::path p = fs::path("extern") / ("pipe" + std::to_string(idx)) / "python" / name;
        if (!writeFile(p, req.body)) return jsonError(500, "write failed");
    }
    return jsonOk({{"status", "ok"}, {"index", idx}, {"name", name}});
}

// ---- Pipeline / runtime updates -------------------------------------------

HttpResponse RestApi::handleUpdatePipeline(const HttpRequest& req) {
    if (cbs_.updatePipeline) {
        if (!cbs_.updatePipeline(req.body)) return jsonError(400, "invalid patch");
    } else {
        try {
            json patch = json::parse(req.body);
            auto& cfg = mgr_.getConfig(mgr_.getActivePipelineIndex());
            from_json(patch, cfg);
        } catch (const std::exception& e) {
            return jsonError(400, e.what());
        }
    }
    return jsonOk({{"status", "ok"}});
}

HttpResponse RestApi::handleUpdatePythonInputs(const HttpRequest& req) {
    std::vector<double> inputs;
    try {
        json j = json::parse(req.body);
        if (j.is_array()) {
            for (auto& v : j) inputs.push_back(v.get<double>());
        }
    } catch (const std::exception& e) {
        return jsonError(400, e.what());
    }
    if (cbs_.updatePythonInputs) cbs_.updatePythonInputs(inputs);
    return jsonOk({{"status", "ok"}, {"count", inputs.size()}});
}

HttpResponse RestApi::handleUpdateRobotOrientation(const HttpRequest& req) {
    std::array<double, 6> rot{};
    try {
        json j = json::parse(req.body);
        if (!j.is_array() || j.size() < 6) return jsonError(400, "expected 6-element array");
        for (size_t i = 0; i < 6; ++i) rot[i] = j[i].get<double>();
    } catch (const std::exception& e) {
        return jsonError(400, e.what());
    }
    if (cbs_.updateRobotOrientation) cbs_.updateRobotOrientation(rot);
    return jsonOk({{"status", "ok"}});
}

HttpResponse RestApi::handleUpdateImuMode(const HttpRequest& req) {
    auto q = parseQuery(req.query);
    int mode = queryInt(q, "mode", -1);
    if (mode < 0 && !req.body.empty()) {
        try { mode = std::stoi(req.body); } catch (...) {}
    }
    if (mode < 0) return jsonError(400, "missing mode");
    if (cbs_.updateImuMode) cbs_.updateImuMode(mode);
    return jsonOk({{"status", "ok"}, {"mode", mode}});
}

HttpResponse RestApi::handleUpdateThrottle(const HttpRequest& req) {
    auto q = parseQuery(req.query);
    int t = queryInt(q, "value", -1);
    if (t < 0 && !req.body.empty()) {
        try { t = std::stoi(req.body); } catch (...) {}
    }
    if (t < 0) return jsonError(400, "missing value");
    if (cbs_.updateThrottle) cbs_.updateThrottle(t);
    return jsonOk({{"status", "ok"}, {"value", t}});
}

HttpResponse RestApi::handleUpdateImuAssistAlpha(const HttpRequest& req) {
    auto q = parseQuery(req.query);
    double a = 0.0;
    auto it = q.find("value");
    if (it != q.end()) { try { a = std::stod(it->second); } catch (...) {} }
    else if (!req.body.empty()) { try { a = std::stod(req.body); } catch (...) {} }
    else return jsonError(400, "missing value");
    if (cbs_.updateImuAssistAlpha) cbs_.updateImuAssistAlpha(a);
    return jsonOk({{"status", "ok"}, {"value", a}});
}

// ---- Calibration ----------------------------------------------------------

HttpResponse RestApi::handleCalDefault(const HttpRequest&) {
    // Default intrinsics for the active pipeline resolution. The exact
    // defaults come from the factory calibration file shipped with the
    // original firmware; here we serve an empty record if none is on disk.
    fs::path p = fs::path(kCalDir) / "default.json";
    std::string body = slurpFile(p);
    if (body.empty()) body = "{\"status\":\"no default calibration\"}";
    return textOk(body, "application/json");
}

HttpResponse RestApi::handleCalFile(const HttpRequest&) {
    fs::path p = fs::path(kCalDir) / "calibration.json";
    std::string body = slurpFile(p);
    if (body.empty()) return jsonError(404, "no calibration file");
    return textOk(body, "application/json");
}

HttpResponse RestApi::handleCalEeprom(const HttpRequest&) {
    fs::path p = fs::path(kCalDir) / "eeprom.json";
    std::string body = slurpFile(p);
    if (body.empty()) return jsonError(404, "no eeprom calibration");
    return textOk(body, "application/json");
}

HttpResponse RestApi::handleCalLatest(const HttpRequest&) {
    fs::path p = fs::path(kCalDir) / "latest.json";
    std::string body = slurpFile(p);
    if (body.empty()) return jsonError(404, "no latest calibration");
    return textOk(body, "application/json");
}

HttpResponse RestApi::handleCalPointcloud(const HttpRequest&) {
    fs::path p = fs::path(kCalDir) / "pointcloud.json";
    std::string body = slurpFile(p);
    if (body.empty()) return jsonError(404, "no point cloud");
    return textOk(body, "application/json");
}

HttpResponse RestApi::handleCalStats(const HttpRequest&) {
    fs::path p = fs::path(kCalDir) / "stats.json";
    std::string body = slurpFile(p);
    if (body.empty()) return jsonError(404, "no cal stats");
    return textOk(body, "application/json");
}

// ---- Pipeline bundle ("llpack") -------------------------------------------

HttpResponse RestApi::handleBundleDownload(const HttpRequest&) {
    std::string tar;
    if (cbs_.bundleDownload) tar = cbs_.bundleDownload();
    if (tar.empty()) return jsonError(500, "no bundle");
    HttpResponse r;
    r.status = 200;
    r.content_type = "application/x-tar";
    r.body = std::move(tar);
    r.extra_headers["Content-Disposition"] = "attachment; filename=\"pipeline-bundle.tar\"";
    return r;
}

HttpResponse RestApi::handleBundleUpload(const HttpRequest& req) {
    if (cbs_.bundleUpload) {
        if (!cbs_.bundleUpload(req.body)) return jsonError(500, "upload failed");
    } else {
        // Default: stage to /tmp/llpack_upload_<ts>/bundle.tar and rely on
        // the main loop to complete extract/validate when it picks it up.
        auto now = std::chrono::system_clock::now().time_since_epoch();
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
        fs::path p = fs::path("/tmp") / ("llpack_upload_" + std::to_string(ms))
                   / "bundle.tar";
        if (!writeFile(p, req.body)) return jsonError(500, "stage failed");
    }
    return jsonOk({{"status", "ok"}});
}

HttpResponse RestApi::handleBundleValidate(const HttpRequest& req) {
    if (cbs_.bundleValidate) {
        if (!cbs_.bundleValidate(req.body)) return jsonError(400, "invalid bundle");
    }
    return jsonOk({{"status", "ok"}});
}

}  // namespace limelight
