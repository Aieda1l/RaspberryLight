#pragma once

#include "visionserver/pipeline.h"
#include <atomic>
#include <array>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

// Forward declaration — ntcore is a heavyweight header.
namespace nt {
class NetworkTableInstance;
class NetworkTable;
}  // namespace nt

namespace limelight {

// Limelight NetworkTables interface. Recovered from Ghidra decompilation of
// the original visionserver binary (FUN_00194440 / CommInterface_NT).
//
// Table naming: /limelight-<usb_id>/ when usb_id != 0, otherwise /limelight/.
// All published topics are double or double[] (Limelight historically uses
// numeric arrays even where a struct would be cleaner — robot code depends
// on the exact array ordering).
//
// The 42 topics below are the complete set registered by CommInterface_NT.
// Per-frame target data (tx, ty, ta, ts, tid, tv, tl, cl, hb) is emitted by
// the per-pipeline output code rather than this class, but NTPublisher owns
// the NT handles for them too.
class NTPublisher {
public:
    // Robot-to-LL input callbacks. Fired when the corresponding _set topic
    // is written to by the robot. All callbacks run on the NT listener
    // thread; the implementation must be internally synchronized.
    struct Callbacks {
        std::function<void(int)> on_pipeline;
        std::function<void(int)> on_led_mode;
        std::function<void(int)> on_crosshairs;
        std::function<void(int)> on_stream;
        std::function<void(int)> on_snapshot;          // rising edge
        std::function<void(int)> on_priority_id;
        std::function<void(std::vector<double>)> on_llrobot;
        std::function<void(std::vector<double>)> on_llpython;
        std::function<void(std::array<double, 6>)> on_robot_orientation;
        std::function<void(std::array<double, 6>)> on_camerapose_robotspace;
        std::function<void(std::vector<double>)> on_fiducial_id_filters;
        std::function<void(double)>              on_fiducial_downscale;
        std::function<void(std::vector<double>)> on_fiducial_offset;
        std::function<void(int)>                 on_imu_mode;
        std::function<void(double)>              on_imu_assist_alpha;
        std::function<void(double)>              on_throttle;
        std::function<void(std::vector<double>)> on_keystone;
        std::function<void(int)>                 on_rewind_enable;
        std::function<void(int)>                 on_capture_rewind; // rising edge
    };

    NTPublisher(const std::string& nt_server,
                int team_number,
                int usb_id);
    ~NTPublisher();

    NTPublisher(const NTPublisher&) = delete;
    NTPublisher& operator=(const NTPublisher&) = delete;

    void setCallbacks(Callbacks cb);

    // Start the NT client and open all topics. Returns false if the NT
    // instance can't be created (should never happen in practice —
    // ntcore is infallible during startup).
    bool start();
    void stop();

    // Publish a frame's worth of data. Called from the main loop after the
    // active pipeline returns a PipelineResult.
    void publish(const PipelineResult& result,
                 int active_pipeline_index,
                 const std::string& active_pipeline_type,
                 double capture_latency_ms,
                 double pipeline_latency_ms);

    // Heartbeat — incremented every frame on `hb`. Robot code uses this to
    // detect a hung visionserver.
    void publishHeartbeat();

    // The `json` topic — full JSON payload of the frame. Called from the
    // pipeline dispatcher after publish() when send_json is enabled.
    void publishJson(const std::string& json_payload);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    std::string table_name_;  // e.g. "limelight" or "limelight-3"
    int         team_number_;
    std::string nt_server_;

    std::atomic<uint64_t> heartbeat_{0};

    // Mutex split — publish() runs every frame on the vision thread, while
    // setCallbacks() is rare but touches a different subset of state.  Using
    // separate mutexes means the hot publish path never waits on someone
    // installing callbacks, and NT listener threads only contend with
    // setCallbacks() for a few instructions.
    std::mutex pub_mutex_;  // publishers + ntcore handles
    std::mutex cb_mutex_;   // callback struct only
};

}  // namespace limelight
