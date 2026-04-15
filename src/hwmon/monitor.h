#pragma once

// Limelight hwmon daemon. Despite the name, the original binary is not a
// classic thermal-monitor loop — it is an HTTP/WebSocket API server on
// port 4802 (0x12c2) that exposes read-only system introspection endpoints
// used by the Limelight web UI:
//
//   * disk/usb/pci/i2c/kernel/process information gathered via popen() on
//     standard Linux tools (lsblk, lsusb, lspci, i2cdetect, ps, cat, ...)
//   * systemd service status and per-boot logs via journalctl/systemctl
//   * WebSocket /logs/* endpoints that tail logs in real time
//
// The original is linked against uWebSockets + zlib. This reimplementation
// uses the tiny HTTP/WS server in http_server.h to match the observable
// on-the-wire behavior.

#include "http_server.h"

#include <memory>

namespace limelight::hwmon {

class HwmonServer {
public:
    HwmonServer();
    ~HwmonServer();

    // Bind + listen on the configured port and register all routes.
    // Returns false if bind fails.
    bool start(uint16_t port);

    // Blocks the caller in the accept loop until stop() is called.
    void run();

    // Ask the accept loop to exit. Safe to call from a signal handler.
    void stop();

private:
    void registerRoutes();

    std::unique_ptr<HttpServer> http_;
};

}  // namespace limelight::hwmon
