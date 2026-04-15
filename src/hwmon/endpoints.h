#pragma once

// Endpoint handlers for the hwmon HTTP/WebSocket API. These are thin
// wrappers that shell out to the same Linux tools the original binary
// invokes (see the strings table at 0x16bb60+ in the original) and pack
// the results into JSON responses matching the schema the Limelight web
// UI expects.

#include "http_server.h"

namespace limelight::hwmon {

// --- Disk group -----------------------------------------------------------
HttpResponse handleDiskInfo(const HttpRequest& req);
HttpResponse handleDiskLargest(const HttpRequest& req);
HttpResponse handleDiskIo(const HttpRequest& req);
HttpResponse handleDiskEject(const HttpRequest& req);
HttpResponse handleDiskEjectOptions(const HttpRequest& req);

// --- Kernel group ---------------------------------------------------------
HttpResponse handleKernelInfo(const HttpRequest& req);
HttpResponse handleKernelModules(const HttpRequest& req);
HttpResponse handleKernelRingbuffer(const HttpRequest& req);
HttpResponse handleKernelAll(const HttpRequest& req);
HttpResponse handleLogsDmesg(const HttpRequest& req);

// --- Logs group -----------------------------------------------------------
HttpResponse handleLogsService(const HttpRequest& req);
HttpResponse handleLogsBoots(const HttpRequest& req);
HttpResponse handleLogsBoot(const HttpRequest& req);
HttpResponse handleJournalStats(const HttpRequest& req);

// --- System group ---------------------------------------------------------
HttpResponse handleUsb(const HttpRequest& req);
HttpResponse handleI2c(const HttpRequest& req);
HttpResponse handleInterrupts(const HttpRequest& req);
HttpResponse handleSystemdServices(const HttpRequest& req);
HttpResponse handlePci(const HttpRequest& req);
HttpResponse handleProcesses(const HttpRequest& req);
HttpResponse handleSystemAll(const HttpRequest& req);

// --- WebSocket /logs/* ----------------------------------------------------
void handleLogStream(std::shared_ptr<WebSocketSession> session,
                     const HttpRequest& req);

// Reusable JSON builders (called from their /api/... endpoints as well as
// from /api/kernel/all and /api/system/all to avoid duplicating the logic).
nlohmann::json buildDiskIoJson();
nlohmann::json buildKernelInfoJson();
nlohmann::json buildKernelModulesJson();
nlohmann::json buildKernelRingbufferJson();
nlohmann::json buildDmesgJson(int lines);
nlohmann::json buildServiceLogsJson(const std::string& service, int lines);
nlohmann::json buildJournalStatsJson();
nlohmann::json buildProcessesJson();
nlohmann::json buildI2cJson();
nlohmann::json buildInterruptsJson();
nlohmann::json buildUsbJson();
nlohmann::json buildPciJson();
nlohmann::json buildSystemdServicesJson();
nlohmann::json buildDiskInfoJson();
nlohmann::json buildDiskLargestJson();
nlohmann::json buildBootListJson();
nlohmann::json buildBootLogsJson(const std::string& boot_id, int lines);

}  // namespace limelight::hwmon
