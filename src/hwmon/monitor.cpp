#include "monitor.h"
#include "endpoints.h"

#include <syslog.h>

namespace limelight::hwmon {

// The original hwmon binary listens on TCP port 4802 (observed at
// FUN_00123c40 as 0x12c2 in the FUN_0016a2e0 listen() call).
constexpr uint16_t kHwmonPort = 4802;

HwmonServer::HwmonServer() : http_(std::make_unique<HttpServer>()) {}
HwmonServer::~HwmonServer() = default;

bool HwmonServer::start(uint16_t port) {
    registerRoutes();
    if (!http_->listen(port)) {
        ::syslog(3, "Failed to listen on port %u", static_cast<unsigned>(port));
        return false;
    }
    ::syslog(6, "Starting HTTP API server");
    return true;
}

void HwmonServer::run() {
    http_->run();
}

void HwmonServer::stop() {
    http_->stop();
}

void HwmonServer::registerRoutes() {
    // The order and paths match FUN_00123c40 exactly.
    http_->websocket("/logs/*", &handleLogStream);

    http_->get("/api/disk/info",       &handleDiskInfo);
    http_->get("/api/disk/largest",    &handleDiskLargest);
    http_->get("/api/disk/io",         &handleDiskIo);
    http_->post("/api/disk/eject",     &handleDiskEject);
    http_->options("/api/disk/eject",  &handleDiskEjectOptions);

    http_->get("/api/usb",             &handleUsb);

    http_->get("/api/kernel/info",     &handleKernelInfo);
    http_->get("/api/kernel/modules",  &handleKernelModules);
    http_->get("/api/kernel/ringbuffer", &handleKernelRingbuffer);
    http_->get("/api/logs/dmesg",      &handleLogsDmesg);
    http_->get("/api/kernel/all",      &handleKernelAll);

    http_->get("/api/i2c",             &handleI2c);
    http_->get("/api/interrupts",      &handleInterrupts);

    http_->get("/api/systemd/services",     &handleSystemdServices);
    http_->get("/api/logs/service/:service", &handleLogsService);
    http_->get("/api/logs/boots",           &handleLogsBoots);
    http_->get("/api/logs/boot/:bootid",    &handleLogsBoot);
    http_->get("/api/logs/journal-stats",   &handleJournalStats);

    http_->get("/api/pci",             &handlePci);
    http_->get("/api/processes",       &handleProcesses);
    http_->get("/api/system/all",      &handleSystemAll);
}

}  // namespace limelight::hwmon
