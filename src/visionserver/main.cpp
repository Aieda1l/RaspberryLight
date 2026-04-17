// Limelight Vision Server — main entry point.
//
// Recreated from Ghidra decompilation of FUN_00154880 in the original
// visionserver binary. See .ghidra_work/notes/visionserver_main_boot.md for
// the full boot-phase map; the phases below are numbered to match.
//
// Working directory: /usr/local/bin/visionserver
// Loads: global.settings, 0-9.vpr, extern/pipe*/fmap.fmap, extern/pipe*/python/
// Ports: 5800 (MJPEG annotated), 5802 (MJPEG raw), 5805 (WebSocket),
//        5807 (REST), 502 (Modbus)
// Connects to: NetworkTables server (nt_server from global.settings)

#include "config.h"
#include "pipeline_config.h"
#include "visionserver/pipeline_manager.h"
#include "visionserver/stream_server.h"
#include "visionserver/websocket_server.h"
#include "visionserver/nt_publisher.h"
#include "visionserver/modbus_server.h"
#include "visionserver/rest_api.h"
#include "hal/camera.h"
#include "hal/gpio.h"
#include "hal/npu.h"

#include <spdlog/spdlog.h>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> g_running{true};

void signalHandler(int) { g_running.store(false); }

// Ports used by the original firmware. These are fixed — the web UI hard-codes
// them in its React bundle. See .ghidra_work/notes/visionserver_rest_endpoints.md
// and visionserver_main_boot.md for the provenance.
constexpr int kStreamPortAnnotated = 5800;  // MJPEG annotated output
constexpr int kStreamPortRaw       = 5802;  // MJPEG raw output
constexpr int kWebSocketPort       = 5805;  // WebSocket push
constexpr int kRestPort            = 5807;  // REST API (HTTP)

}  // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    // =========================================================
    // Phase 1: banner
    // =========================================================
    spdlog::info("Booting Limelight Vision Server...");

    // =========================================================
    // Phase 9: Load global settings
    // (The original "SETTINGS" log line lives here too.)
    // =========================================================
    spdlog::info("SETTINGS");
    limelight::GlobalSettings settings;
    try {
        settings = limelight::GlobalSettings::load("global.settings");
        spdlog::info("  team={}, nt_server={}, usb_id={}",
                     settings.team_number, settings.nt_server, settings.usb_id);
    } catch (const std::exception& e) {
        spdlog::error("Failed to load global settings: {}", e.what());
        return 1;
    }

    // =========================================================
    // Phase 13: Pipeline subsystem init
    // =========================================================
    limelight::PipelineManager pipeline_mgr;
    try {
        pipeline_mgr.loadConfigs(".");
        pipeline_mgr.loadFieldMaps(".");
        pipeline_mgr.setActivePipeline(settings.default_vision_profile);
        spdlog::info("Loaded {} pipeline configs, active={}",
                     limelight::NUM_PIPELINES, settings.default_vision_profile);
    } catch (const std::exception& e) {
        spdlog::error("Failed to load pipeline configs: {}", e.what());
        return 1;
    }

    // =========================================================
    // Phase 3: Camera HAL (hardware detection is baked into HAL::create)
    // =========================================================
    auto camera = limelight::hal::Camera::create();
    if (!camera->open()) {
        spdlog::error("Failed to open camera");
        return 1;
    }

    {
        const auto initial_config = pipeline_mgr.getConfigSnapshot(
            pipeline_mgr.getActivePipelineIndex());
        limelight::hal::CameraControls cam_ctrl;
        cam_ctrl.exposure     = initial_config.exposure;
        cam_ctrl.gain         = initial_config.lcgain;
        cam_ctrl.red_balance  = initial_config.red_balance;
        cam_ctrl.blue_balance = initial_config.blue_balance;
        cam_ctrl.black_level  = initial_config.black_level;
        cam_ctrl.flip         = initial_config.image_flip;
        cam_ctrl.resolution   = initial_config.pipeline_res;
        camera->setControls(cam_ctrl);
    }

    if (!camera->startCapture()) {
        spdlog::error("Failed to start camera capture");
        return 1;
    }
    spdlog::info("Camera started: {}x{} @ {}fps ({})",
                 camera->getInfo().width, camera->getInfo().height,
                 camera->getInfo().fps, camera->getInfo().sensor_name);

    limelight::PipelineConfig active_config;  // re-populated each frame below

    // =========================================================
    // Phase 6: LED controller
    // =========================================================
    spdlog::info("LEDS");
    auto gpio = limelight::hal::Gpio::create();
    gpio->init();

    // =========================================================
    // Phase 8: WebSocket server ("WEBSOCK")
    // =========================================================
    spdlog::info("WEBSOCK");
    limelight::WebSocketServer ws_server(kWebSocketPort);
    if (!ws_server.start()) {
        spdlog::error("Failed to start WebSocket server on port {}", kWebSocketPort);
    }

    // =========================================================
    // Phase 10: MJPEG stream servers ("MJPG")
    // Original runs two instances: annotated on 5800, raw on 5802.
    // =========================================================
    spdlog::info("MJPG");
    limelight::StreamServer stream_annotated(kStreamPortAnnotated);
    limelight::StreamServer stream_raw(kStreamPortRaw);
    if (!stream_annotated.start()) {
        spdlog::error("Failed to start annotated stream server on {}", kStreamPortAnnotated);
    }
    if (!stream_raw.start()) {
        spdlog::error("Failed to start raw stream server on {}", kStreamPortRaw);
    }

    // =========================================================
    // Phase 11: NetworkTables client ("COMMS")
    // =========================================================
    spdlog::info("COMMS");
    limelight::NTPublisher nt_publisher(settings.nt_server,
                                        settings.team_number,
                                        settings.usb_id);

    limelight::NTPublisher::Callbacks nt_cbs;
    nt_cbs.on_pipeline = [&](int idx) {
        if (idx >= 0 && idx < limelight::NUM_PIPELINES) {
            pipeline_mgr.setActivePipeline(idx);
        }
    };
    nt_publisher.setCallbacks(std::move(nt_cbs));

    if (!nt_publisher.start()) {
        spdlog::error("Failed to start NT client");
    }

    // =========================================================
    // Phase 12: Modbus server ("MODBUS ENABLE")
    // =========================================================
    spdlog::info("MODBUS ENABLE");
    limelight::ModbusServer modbus_server(settings.modbus_port);
    if (!modbus_server.start()) {
        spdlog::error("Failed to start Modbus server on port {}", settings.modbus_port);
    }

    // =========================================================
    // REST API (served on its own HTTP port)
    // =========================================================
    limelight::RestApi rest_api(pipeline_mgr, settings);

    limelight::RestApi::Callbacks rest_cbs;
    rest_cbs.reloadPipeline = [&]() {
        int idx = pipeline_mgr.getActivePipelineIndex();
        pipeline_mgr.loadConfigs(".");
        pipeline_mgr.setActivePipeline(idx);
    };
    rest_cbs.switchPipeline   = [&](int idx) { pipeline_mgr.setActivePipeline(idx); };
    rest_cbs.pipelineDefault  = [&]() {
        const int idx = pipeline_mgr.getActivePipelineIndex();
        pipeline_mgr.mutateConfig(idx, [](limelight::PipelineConfig& c) {
            c = limelight::PipelineConfig{};
        });
        pipeline_mgr.setActivePipeline(idx);
    };
    rest_api.setCallbacks(std::move(rest_cbs));

    if (!rest_api.start(kRestPort)) {
        spdlog::error("Failed to start REST API on port {}", kRestPort);
    }

    // =========================================================
    // Phase 16+17: "LEDS ON", "Entering main loop"
    // =========================================================
    spdlog::info("LEDS ON");
    spdlog::info("Entering main loop");

    uint64_t heartbeat = 0;
    auto last_frame_ts = std::chrono::steady_clock::now();

    while (g_running.load()) {
        // Re-sample the active pipeline config every frame so that a pipeline
        // switch or a REST /update-pipeline takes effect immediately rather
        // than staying bound to whatever slot was active at startup.
        active_config = pipeline_mgr.getConfigSnapshot(
            pipeline_mgr.getActivePipelineIndex());

        auto t_capture_start = std::chrono::steady_clock::now();
        cv::Mat frame = camera->getFrame(100);
        auto t_capture_end = std::chrono::steady_clock::now();
        if (frame.empty()) continue;

        double capture_ms = std::chrono::duration<double, std::milli>(
                                t_capture_end - t_capture_start).count();

        auto t_pipe_start = std::chrono::steady_clock::now();
        auto result = pipeline_mgr.processFrame(frame);
        auto t_pipe_end = std::chrono::steady_clock::now();
        double pipeline_ms = std::chrono::duration<double, std::milli>(
                                 t_pipe_end - t_pipe_start).count();

        // LED state tracks target-valid flag when LED control is enabled.
        if (active_config.pipeline_led_enabled) {
            if (result.has_target) {
                gpio->setLed(limelight::hal::LedColor::GREEN,
                             active_config.pipeline_led_power);
            } else {
                gpio->setLed(limelight::hal::LedColor::OFF);
            }
        }

        // Publish to NetworkTables.
        nt_publisher.publish(result,
                             pipeline_mgr.getActivePipelineIndex(),
                             active_config.pipeline_type,
                             capture_ms,
                             pipeline_ms);
        nt_publisher.publishHeartbeat();

        // Update Modbus input registers.
        modbus_server.updateFromResult(result,
                                       pipeline_mgr.getActivePipelineIndex(),
                                       capture_ms,
                                       pipeline_ms);

        // Push annotated frame to stream servers. The raw server gets the
        // unprocessed frame directly, matching the original split.
        stream_annotated.updateFrame(
            result.annotated_frame.empty() ? frame : result.annotated_frame,
            settings.stream_quality);
        stream_raw.updateFrame(frame, settings.stream_quality);

        // WebSocket push.
        ws_server.pushResult(result.to_json());

        // Update REST API live state.
        limelight::RestLiveState state;
        state.latest               = result;
        state.active_pipeline_index = pipeline_mgr.getActivePipelineIndex();
        state.capture_latency_ms   = capture_ms;
        state.pipeline_latency_ms  = pipeline_ms;
        state.heartbeat            = heartbeat;
        rest_api.updateLiveState(state);

        ++heartbeat;
        last_frame_ts = t_pipe_end;
        (void)last_frame_ts;
    }

    // =========================================================
    // Cleanup
    // =========================================================
    spdlog::info("Shutting down...");
    rest_api.stop();
    nt_publisher.stop();
    modbus_server.stop();
    stream_annotated.stop();
    stream_raw.stop();
    ws_server.stop();
    camera->stopCapture();
    camera->close();
    gpio->setLedOff();

    return 0;
}
