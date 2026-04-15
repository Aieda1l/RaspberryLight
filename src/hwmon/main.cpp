// Limelight hwmon daemon entry point.
// Recreated from Ghidra decompilation of the original hwmon binary.
//
// Original startup sequence (FUN_00107960):
//   syslog(LOG_INFO, "LL HWMON START")
//   printBanner()         -- prints version/commit/build-time banner to stdout
//   startHttpServer()     -- binds TCP/4802 and registers 21 endpoints
//   return 0              -- the HTTP server holds the process open
//
// Working directory: /usr/local/bin/hwmon

#include "monitor.h"

#include <csignal>
#include <iostream>
#include <string>
#include <syslog.h>

namespace {

// Mirrors the banner in FUN_0010f120. Commit / build-time strings are the
// ones embedded in the original binary at 0x16bda8 and 0x16bdd8 — they
// correspond to the firmware this codebase was reverse-engineered from, so
// we print them verbatim for provenance.
constexpr const char* kOriginalCommit    = "e687e3815f76c1c5ea9fb52b6558bedfe53ab117";
constexpr const char* kOriginalBuildTime = "2026-01-24 18:49:33 UTC";

void printBanner() {
    std::cout << "=== Limelight Hardware Monitor ===" << std::endl;
    std::cout << "Commit: " << kOriginalCommit << std::endl;
    std::cout << "Built: "  << kOriginalBuildTime << std::endl;
    std::cout << "======================================" << std::endl;
}

limelight::hwmon::HwmonServer* g_server = nullptr;

void signalHandler(int) {
    if (g_server) g_server->stop();
}

}  // namespace

int main(int /*argc*/, char* /*argv*/[]) {
    ::openlog("hwmon", LOG_PID, LOG_DAEMON);
    ::syslog(6, "LL HWMON START");

    printBanner();

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    limelight::hwmon::HwmonServer server;
    g_server = &server;

    if (!server.start(/*port=*/4802)) {
        std::cerr << "Failed to start HTTP API server" << std::endl;
        return 1;
    }

    server.run();
    ::closelog();
    return 0;
}
