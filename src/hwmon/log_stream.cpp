#include "endpoints.h"
#include "util.h"

#include <nlohmann/json.hpp>

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <syslog.h>
#include <thread>

namespace limelight::hwmon {

// Log stream WebSocket / FUN_001331e8. Behavior observed from the strings
// table (0x16d1c0..0x16d1f8) and surrounding decompilation:
//
//   - The client sends a JSON message on the upgraded socket after the
//     handshake, of the form:
//         { "type": "journalctl", "service": "<optional>" }
//         { "type": "dmesg" }
//   - The server validates the "type" field, assembles the appropriate
//     tail command, and streams each line back as a text frame.
//
// The original logs "Log stream WebSocket opened" at syslog level 6 on
// successful upgrade. Unknown types produce "Unknown log type for
// streaming: %s" and an immediate close.

namespace {

// Tails a shell command (journalctl -f, dmesg -w, ...) and forwards each
// line over a WebSocket session as a text frame. Spawns one worker thread
// and owns it. The destructor tears the worker down cleanly: it flips the
// `running` flag, and the worker notices on its next poll() timeout (at
// most ~500ms), closes the pipe, and returns. The destructor then joins.
struct LogTailer {
    std::shared_ptr<WebSocketSession> session;
    std::string cmd;
    std::thread worker;
    std::atomic<bool> running{true};

    LogTailer(std::shared_ptr<WebSocketSession> s, std::string c)
        : session(std::move(s)), cmd(std::move(c)) {}

    void start() {
        worker = std::thread([this]() { runLoop(); });
    }

    void runLoop() {
        FILE* pipe = ::popen(cmd.c_str(), "r");
        if (!pipe) {
            session->sendText("{\"error\":\"Failed to start log stream\"}");
            return;
        }

        // Make the underlying fd non-blocking so poll() can enforce the
        // shutdown check interval below.
        int fd = ::fileno(pipe);
        int flags = ::fcntl(fd, F_GETFL, 0);
        if (flags >= 0) ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        std::string pending;
        std::array<char, 8192> buf;
        while (running.load()) {
            pollfd pfd{fd, POLLIN, 0};
            int rc = ::poll(&pfd, 1, 500);
            if (rc < 0) {
                if (errno == EINTR) continue;
                break;
            }
            if (rc == 0) continue;  // timeout -> re-check running
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                // Drain any trailing data one last time, then exit.
            }
            if (!(pfd.revents & (POLLIN | POLLHUP))) continue;

            ssize_t n = ::read(fd, buf.data(), buf.size());
            if (n == 0) break;  // EOF
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                if (errno == EINTR) continue;
                break;
            }
            pending.append(buf.data(), static_cast<size_t>(n));

            // Flush any complete lines as individual text frames.
            size_t start = 0;
            while (true) {
                size_t nl = pending.find('\n', start);
                if (nl == std::string::npos) break;
                std::string line = pending.substr(start, nl - start);
                while (!line.empty() && line.back() == '\r') line.pop_back();
                if (!session->sendText(line)) {
                    running.store(false);
                    break;
                }
                start = nl + 1;
            }
            if (start > 0) pending.erase(0, start);
        }

        ::pclose(pipe);
    }

    ~LogTailer() {
        running.store(false);
        // Unblock any recv the session is in (e.g. awaitClose on another
        // thread) and force-close the socket so sendText returns false.
        if (session) session->shutdown();
        if (worker.joinable()) worker.join();
    }
};

// Wait for the client's first text frame and return its payload. Returns
// empty on error/close. This reads directly from the socket because the
// WebSocketSession::awaitClose loop drains frames without surfacing them.
std::string receiveFirstTextFrame(int fd) {
    uint8_t hdr[2];
    if (::recv(fd, hdr, 2, MSG_WAITALL) != 2) return "";
    uint8_t opcode = hdr[0] & 0x0f;
    bool masked = (hdr[1] & 0x80) != 0;
    uint64_t plen = hdr[1] & 0x7f;

    if (plen == 126) {
        uint8_t ext[2];
        if (::recv(fd, ext, 2, MSG_WAITALL) != 2) return "";
        plen = (uint64_t(ext[0]) << 8) | ext[1];
    } else if (plen == 127) {
        uint8_t ext[8];
        if (::recv(fd, ext, 8, MSG_WAITALL) != 8) return "";
        plen = 0;
        for (int i = 0; i < 8; ++i) plen = (plen << 8) | ext[i];
    }

    uint8_t mask[4] = {0, 0, 0, 0};
    if (masked) {
        if (::recv(fd, mask, 4, MSG_WAITALL) != 4) return "";
    }

    if (opcode != 0x1 || plen == 0 || plen > 8192) return "";
    std::string payload(plen, '\0');
    ssize_t got = ::recv(fd, payload.data(), plen, MSG_WAITALL);
    if (got != static_cast<ssize_t>(plen)) return "";
    if (masked) {
        for (size_t i = 0; i < plen; ++i) {
            payload[i] ^= mask[i & 3];
        }
    }
    return payload;
}

}  // namespace

void handleLogStream(std::shared_ptr<WebSocketSession> session,
                     const HttpRequest& /*req*/) {
    ::syslog(6, "Log stream WebSocket opened");

    std::string msg = receiveFirstTextFrame(session->fd());
    if (msg.empty()) return;

    std::string type;
    std::string service;
    try {
        auto j = nlohmann::json::parse(msg);
        if (j.contains("type")) type = j["type"].get<std::string>();
        if (j.contains("service") && j["service"].is_string()) {
            service = j["service"].get<std::string>();
        }
    } catch (const std::exception& e) {
        ::syslog(3, "Error parsing log stream request: %s", e.what());
        return;
    }

    std::string cmd;
    if (type == "journalctl") {
        if (!service.empty() && isSafeServiceName(service)) {
            cmd = "journalctl -u " + service + " -f --no-pager 2>&1";
        } else {
            cmd = "journalctl -f --no-pager 2>&1";
        }
    } else if (type == "dmesg") {
        cmd = "dmesg -w 2>&1";
    } else {
        ::syslog(3, "Unknown log type for streaming: %s", type.c_str());
        return;
    }

    // The tailer owns its worker thread. awaitClose blocks us here until
    // the peer disconnects (or the tailer forces the session closed from
    // the worker side by returning false from sendText). When this frame
    // unwinds, ~LogTailer flips `running`, shuts down the socket, and
    // joins the worker — no leaks, no detached threads.
    LogTailer tailer(session, cmd);
    tailer.start();
    session->awaitClose();
}

}  // namespace limelight::hwmon
