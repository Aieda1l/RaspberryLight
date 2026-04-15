#pragma once

#include "visionserver/pipeline.h"
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

namespace limelight {

// Modbus TCP server for the Limelight. Runs on a single dedicated thread
// that was originally spawned as a detached std::thread from main (see
// FUN_0019fee0 / FUN_001a0324 in the Ghidra decompile). The detached model
// is preserved here because the server never needs a clean shutdown —
// it's torn down via process exit — but we also expose stop() so the main
// thread can join deterministically in tests.
//
// Register layout: the original binary uses 128 holding registers
// (function codes 3/6/16) and 128 input registers (function code 4). The
// values below are the documented Limelight register map.
class ModbusServer {
public:
    static constexpr int kRegCount = 128;

    // Input registers (FC 04) — read-only from robot.
    //   0: tv (target valid, 0/1)
    //   1: tx_hundredths (signed, -2700..2700 ~= -27..27 deg * 100)
    //   2: ty_hundredths
    //   3: ta_tenths  (0..1000)
    //   4: tl_ms
    //   5: cl_ms
    //   6: tid
    //   7: pipeline index
    //   8..13: botpose xyz (mm) + rpy (deg * 10)
    //   14..19: targetpose_cameraspace
    //   ...
    // Holding registers (FC 03/06/16) — writable control plane.
    //   0: active pipeline index
    //   1: led mode
    //   2: crosshair select
    //   3: stream layout
    //   4: snapshot trigger (rising edge)
    //   ...
    //
    // The full layout is documented in docs/modbus.md.

    explicit ModbusServer(int port);
    ~ModbusServer();

    ModbusServer(const ModbusServer&) = delete;
    ModbusServer& operator=(const ModbusServer&) = delete;

    // Start the accept loop on a background thread. Returns false if
    // modbus_new_tcp or modbus_tcp_listen failed.
    bool start();
    void stop();

    // Update the input register bank from the latest pipeline result.
    // Safe to call from the main loop; takes a mutex.
    void updateFromResult(const PipelineResult& result,
                          int active_pipeline_index,
                          double capture_latency_ms,
                          double pipeline_latency_ms);

    // Snapshot of the holding registers (for the main loop to apply
    // control-plane writes from the robot). Returns the current values
    // under lock.
    std::array<uint16_t, kRegCount> snapshotHoldingRegs();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    int port_;
    std::atomic<bool> running_{false};
    std::thread worker_;
    std::mutex mutex_;

    std::array<uint16_t, kRegCount> input_regs_{};   // published to client
    std::array<uint16_t, kRegCount> holding_regs_{}; // received from client

    void runLoop();
};

}  // namespace limelight
