// Modbus TCP server for the Limelight vision server.
//
// Recreated from Ghidra decompilation of FUN_0019fee0 (ModbusDevice
// constructor / init), FUN_002a0c20 (thread worker shim), and FUN_001a0324
// (run loop) in the original visionserver binary.
//
// Observed behavior:
//   1. modbus_new_tcp(NULL, port)           // bind 0.0.0.0:port
//   2. modbus_set_debug(ctx, 0)
//   3. modbus_set_error_recovery(ctx, 6)    // LINK | PROTOCOL
//   4. modbus_mapping_new(0, 0, 0x80, 0x80) // 128 holding + 128 input regs
//   5. modbus_tcp_listen(ctx, 1)
//   6. loop { modbus_tcp_accept; modbus_receive; modbus_reply }
//   7. Before each reply, the latest input-register values are memcpy'd
//      from the ModbusDevice's input_regs vector into
//      mapping->tab_input_registers (and the same for holding).
//
// Logs observed in strings:
//   "starting modbus server"
//   "Failed to allocate Modbus mapping"
//   "acquire modbus listen socket"
//   "WAITING FOR NEW CLIENT"
//   "Failed to accept Modbus client"
//   "MODBUS CLIENT ACCEPTED"
//   "Failed to listen for Modbus clients"
//   "MODBUS SERVER START AT <port>"
//   "MSERVER START"   (from the thread shim)

#include "visionserver/modbus_server.h"

#include <modbus/modbus.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>

namespace limelight {

struct ModbusServer::Impl {
    modbus_t* ctx = nullptr;
    modbus_mapping_t* mapping = nullptr;
    int listen_fd = -1;
};

// Clamp a double value to [min, max] and cast to int16_t. Used to pack
// floating-point target data (tx, ty, etc.) into integer registers.
static inline int16_t clampToI16(double v) {
    if (std::isnan(v)) return 0;
    if (v >  32767.0) return  32767;
    if (v < -32768.0) return -32768;
    return static_cast<int16_t>(v);
}

ModbusServer::ModbusServer(int port) : impl_(std::make_unique<Impl>()), port_(port) {}

ModbusServer::~ModbusServer() { stop(); }

bool ModbusServer::start() {
    impl_->ctx = ::modbus_new_tcp(nullptr, port_);
    if (!impl_->ctx) {
        spdlog::error("Unable to create Modbus context");
        return false;
    }

    ::modbus_set_debug(impl_->ctx, 0);
    ::modbus_set_error_recovery(
        impl_->ctx,
        static_cast<modbus_error_recovery_mode>(
            MODBUS_ERROR_RECOVERY_LINK | MODBUS_ERROR_RECOVERY_PROTOCOL));

    spdlog::info("MODBUS SERVER START AT {}", port_);

    impl_->mapping = ::modbus_mapping_new(0, 0, kRegCount, kRegCount);
    if (!impl_->mapping) {
        spdlog::error("Failed to allocate Modbus mapping");
        ::modbus_free(impl_->ctx);
        impl_->ctx = nullptr;
        return false;
    }

    impl_->listen_fd = ::modbus_tcp_listen(impl_->ctx, 1);
    if (impl_->listen_fd == -1) {
        spdlog::error("Failed to listen for Modbus clients");
        ::modbus_mapping_free(impl_->mapping);
        ::modbus_free(impl_->ctx);
        impl_->mapping = nullptr;
        impl_->ctx = nullptr;
        return false;
    }

    running_ = true;
    worker_ = std::thread([this]() { runLoop(); });
    return true;
}

void ModbusServer::stop() {
    running_ = false;
    if (impl_->listen_fd != -1) {
        ::close(impl_->listen_fd);
        impl_->listen_fd = -1;
    }
    if (worker_.joinable()) worker_.join();
    if (impl_->mapping) {
        ::modbus_mapping_free(impl_->mapping);
        impl_->mapping = nullptr;
    }
    if (impl_->ctx) {
        ::modbus_close(impl_->ctx);
        ::modbus_free(impl_->ctx);
        impl_->ctx = nullptr;
    }
}

void ModbusServer::runLoop() {
    spdlog::info("MSERVER START");
    spdlog::info("starting modbus server");

    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];

    while (running_) {
        spdlog::debug("WAITING FOR NEW CLIENT");
        int rc = ::modbus_tcp_accept(impl_->ctx, &impl_->listen_fd);
        if (rc == -1) {
            if (!running_) break;
            spdlog::error("Failed to accept Modbus client");
            continue;
        }
        spdlog::info("MODBUS CLIENT ACCEPTED");

        while (running_) {
            rc = ::modbus_receive(impl_->ctx, query);
            if (rc == -1) break;  // client disconnected

            // Refresh mapping from our register banks under the lock, then
            // reply. This mirrors the original binary's pattern of copying
            // the vector<uint16_t> into modbus_mapping_t each request.
            {
                std::lock_guard<std::mutex> lock(mutex_);
                std::memcpy(impl_->mapping->tab_input_registers, input_regs_.data(),
                            kRegCount * sizeof(uint16_t));
                std::memcpy(impl_->mapping->tab_registers, holding_regs_.data(),
                            kRegCount * sizeof(uint16_t));
            }

            rc = ::modbus_reply(impl_->ctx, query, rc, impl_->mapping);

            // Pull any holding-register writes back out for the main loop
            // to apply.
            {
                std::lock_guard<std::mutex> lock(mutex_);
                std::memcpy(holding_regs_.data(), impl_->mapping->tab_registers,
                            kRegCount * sizeof(uint16_t));
            }

            if (rc == -1) break;
        }
    }
}

void ModbusServer::updateFromResult(const PipelineResult& result,
                                    int active_pipeline_index,
                                    double capture_latency_ms,
                                    double pipeline_latency_ms) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto& r = input_regs_;
    r.fill(0);

    r[0] = result.has_target ? 1u : 0u;
    r[1] = static_cast<uint16_t>(clampToI16(result.tx * 100.0));
    r[2] = static_cast<uint16_t>(clampToI16(result.ty * 100.0));
    r[3] = static_cast<uint16_t>(clampToI16(result.ta * 10.0));
    r[4] = static_cast<uint16_t>(std::clamp(pipeline_latency_ms, 0.0, 65535.0));
    r[5] = static_cast<uint16_t>(std::clamp(capture_latency_ms, 0.0, 65535.0));
    r[6] = static_cast<uint16_t>(std::max(0, result.tid));
    r[7] = static_cast<uint16_t>(active_pipeline_index);

    // Botpose (x_mm, y_mm, z_mm, rx_deg*10, ry_deg*10, rz_deg*10)
    if (result.botpose.size() >= 6) {
        r[ 8] = static_cast<uint16_t>(clampToI16(result.botpose[0] * 1000.0));
        r[ 9] = static_cast<uint16_t>(clampToI16(result.botpose[1] * 1000.0));
        r[10] = static_cast<uint16_t>(clampToI16(result.botpose[2] * 1000.0));
        r[11] = static_cast<uint16_t>(clampToI16(result.botpose[3] * 10.0));
        r[12] = static_cast<uint16_t>(clampToI16(result.botpose[4] * 10.0));
        r[13] = static_cast<uint16_t>(clampToI16(result.botpose[5] * 10.0));
    }

    // Target pose in camera space (6-DOF)
    if (result.targetpose_cameraspace.size() >= 6) {
        r[14] = static_cast<uint16_t>(clampToI16(result.targetpose_cameraspace[0] * 1000.0));
        r[15] = static_cast<uint16_t>(clampToI16(result.targetpose_cameraspace[1] * 1000.0));
        r[16] = static_cast<uint16_t>(clampToI16(result.targetpose_cameraspace[2] * 1000.0));
        r[17] = static_cast<uint16_t>(clampToI16(result.targetpose_cameraspace[3] * 10.0));
        r[18] = static_cast<uint16_t>(clampToI16(result.targetpose_cameraspace[4] * 10.0));
        r[19] = static_cast<uint16_t>(clampToI16(result.targetpose_cameraspace[5] * 10.0));
    }
}

std::array<uint16_t, ModbusServer::kRegCount> ModbusServer::snapshotHoldingRegs() {
    std::lock_guard<std::mutex> lock(mutex_);
    return holding_regs_;
}

}  // namespace limelight
