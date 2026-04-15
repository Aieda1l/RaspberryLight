#pragma once

// Limelight visionserver REST API.
//
// Port 5807 in the original binary (distinct from the MJPEG stream on 5800
// and the websocket on 5805 — confirmed by the contiguous route table at
// .rodata 0x003191b0..0x003194e0). Route list is frozen in
// .ghidra_work/notes/visionserver_rest_endpoints.md (41 endpoints total).
//
// The RestApi class owns an HttpServer, registers every route against it,
// and dispatches to handler methods that touch the PipelineManager, the
// GlobalSettings, and the various on-disk staging areas described in the
// notes file.
//
// Long-lived state (latest pipeline result, calibration state, snapshot
// manifest, recording manifest) is provided through a simple state struct
// protected by a mutex. The main loop updates the state every frame; the
// REST handlers read from it without blocking the capture path.

#include "config.h"
#include "pipeline_config.h"
#include "visionserver/http_server.h"
#include "visionserver/pipeline_manager.h"
#include "visionserver/pipeline.h"

#include <array>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace limelight {

// Snapshot of live vision-server state that the REST handlers expose as JSON.
struct RestLiveState {
    PipelineResult latest;          // /results
    int active_pipeline_index = 0;  // /getpipe equivalent
    int active_pipeline_type  = 0;  // /getpipetype equivalent
    double capture_latency_ms  = 0.0;
    double pipeline_latency_ms = 0.0;
    uint64_t heartbeat = 0;
};

class RestApi {
public:
    // Callbacks the server-facing code wires up from main(). The handlers
    // only touch these callbacks — they never reach into the main loop's
    // private state directly.
    struct Callbacks {
        // /reload-pipeline — re-read current .vpr from disk and rebuild.
        std::function<void()> reloadPipeline;
        // /pipeline-switch?index=N — switch the active pipeline.
        std::function<void(int)> switchPipeline;
        // /pipeline-default — reset active pipeline to default values.
        std::function<void()> pipelineDefault;
        // /upload-snapshot — persist a JPEG payload to snapshots/.
        std::function<bool(const std::string& name, const std::string& jpeg)> uploadSnapshot;
        // /capture-snapshot — grab the current annotated frame to disk.
        std::function<bool()> captureSnapshot;
        // /delete-snapshot?name=... — delete one snapshot.
        std::function<bool(const std::string& name)> deleteSnapshot;
        // /delete-snapshots — delete all snapshots.
        std::function<void()> deleteAllSnapshots;
        // /delete-video?name=... — delete one recorded video.
        std::function<bool(const std::string& name)> deleteVideo;
        // /delete-videos — delete all recorded videos.
        std::function<void()> deleteAllVideos;
        // /recording-flush — flush the current recording buffer.
        std::function<void()> recordingFlush;
        // /update-pipeline — merge a JSON patch into the active pipeline.
        std::function<bool(const std::string& json_patch)> updatePipeline;
        // /update-pythoninputs — replace the Python input buffer.
        std::function<void(const std::vector<double>&)> updatePythonInputs;
        // /update-robotorientation — set the 6-DOF robot orientation.
        std::function<void(const std::array<double, 6>&)> updateRobotOrientation;
        // /update-imumode — set the IMU assist mode.
        std::function<void(int)> updateImuMode;
        // /update-throttle — set the pipeline throttle divisor.
        std::function<void(int)> updateThrottle;
        // /update-imuassistalpha — set the IMU assist alpha.
        std::function<void(double)> updateImuAssistAlpha;
        // /upload-pipeline — write a .vpr to slot N.
        std::function<bool(int index, const std::string& body)> uploadPipeline;
        // /upload-fieldmap — write fmap.fmap to slot N.
        std::function<bool(int index, const std::string& body)> uploadFieldmap;
        // /upload-nn, /upload-nnlabels — write NN model / labels for slot N.
        std::function<bool(int index, const std::string& body)> uploadNn;
        std::function<bool(int index, const std::string& body)> uploadNnLabels;
        // /upload-python — write a Python script for slot N.
        std::function<bool(int index, const std::string& body)> uploadPython;
        // /api/pipeline-bundle/* — archive import/export.
        std::function<std::string()>               bundleDownload;
        std::function<bool(const std::string&)>    bundleUpload;
        std::function<bool(const std::string&)>    bundleValidate;
    };

    RestApi(PipelineManager& mgr, GlobalSettings& settings);
    ~RestApi();

    RestApi(const RestApi&) = delete;
    RestApi& operator=(const RestApi&) = delete;

    void setCallbacks(Callbacks cbs);

    // Update the live state snapshot exposed via /results, /status, /dump.
    // Called once per frame from the main capture loop.
    void updateLiveState(const RestLiveState& state);

    // Bind the listening socket and start serving requests.
    bool start(uint16_t port);
    void stop();

private:
    void registerRoutes();

    // ---- Handlers (one per route) -----------------------------------------
    HttpResponse handleStatus       (const HttpRequest& req);
    HttpResponse handleResults      (const HttpRequest& req);
    HttpResponse handleDump         (const HttpRequest& req);
    HttpResponse handleDumpd        (const HttpRequest& req);
    HttpResponse handleHwReport     (const HttpRequest& req);
    HttpResponse handleSnapshotManifest(const HttpRequest& req);
    HttpResponse handleUploadSnapshot  (const HttpRequest& req);
    HttpResponse handleCaptureSnapshot (const HttpRequest& req);
    HttpResponse handleDeleteSnapshots (const HttpRequest& req);
    HttpResponse handleDeleteSnapshot  (const HttpRequest& req);
    HttpResponse handleVideoList       (const HttpRequest& req);
    HttpResponse handleDeleteVideo     (const HttpRequest& req);
    HttpResponse handleDeleteVideos    (const HttpRequest& req);
    HttpResponse handleReloadPipeline  (const HttpRequest& req);
    HttpResponse handleGetSnapScriptNames(const HttpRequest& req);
    HttpResponse handleCalDefault      (const HttpRequest& req);
    HttpResponse handleCalFile         (const HttpRequest& req);
    HttpResponse handleCalEeprom       (const HttpRequest& req);
    HttpResponse handleCalLatest       (const HttpRequest& req);
    HttpResponse handleCalPointcloud   (const HttpRequest& req);
    HttpResponse handleCalStats        (const HttpRequest& req);
    HttpResponse handlePipelineDefault (const HttpRequest& req);
    HttpResponse handlePipelineAtIndex (const HttpRequest& req);
    HttpResponse handlePipelineSwitch  (const HttpRequest& req);
    HttpResponse handleUploadPipeline  (const HttpRequest& req);
    HttpResponse handleUploadFieldmap  (const HttpRequest& req);
    HttpResponse handleUploadNn        (const HttpRequest& req);
    HttpResponse handleUploadNnLabels  (const HttpRequest& req);
    HttpResponse handleUploadPython    (const HttpRequest& req);
    HttpResponse handleUpdatePipeline  (const HttpRequest& req);
    HttpResponse handleUpdatePythonInputs(const HttpRequest& req);
    HttpResponse handleUpdateRobotOrientation(const HttpRequest& req);
    HttpResponse handleUpdateImuMode   (const HttpRequest& req);
    HttpResponse handleUpdateThrottle  (const HttpRequest& req);
    HttpResponse handleUpdateImuAssistAlpha(const HttpRequest& req);
    HttpResponse handleRecordingList   (const HttpRequest& req);
    HttpResponse handleRecordingStream (const HttpRequest& req);
    HttpResponse handleRecordingFlush  (const HttpRequest& req);
    HttpResponse handleBundleDownload  (const HttpRequest& req);
    HttpResponse handleBundleUpload    (const HttpRequest& req);
    HttpResponse handleBundleValidate  (const HttpRequest& req);

    PipelineManager&         mgr_;
    GlobalSettings&          settings_;
    std::unique_ptr<HttpServer> server_;
    Callbacks                cbs_;

    mutable std::mutex       state_mu_;
    RestLiveState            state_;
};

}  // namespace limelight
