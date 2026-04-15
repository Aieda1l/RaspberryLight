#pragma once

// Limelight visionserver WebSocket push server.
//
// The original binary has a class called `WebSocketServer` (confirmed by the
// mangled symbols `15WebSocketServer10ProcessNewEv`, `29ActionContext_WebSocketServer`,
// and `13PerSocketData` — see .ghidra_work/output/visionserver/raw_strings.txt
// lines 225..1178). It links against uWebSockets (`uWebSockets` appears
// verbatim in .rodata at 0x00317d70) and the class internally uses:
//
//   uWS::TemplatedApp<false>           — non-SSL HTTP+WS server
//   uWS::WebSocketBehavior<PerSocketData> — per-connection state
//   std::shared_ptr<StreamState>       — HTTP body streaming (/recording/*)
//   PackProgress                       — upload progress events
//
// All of this runs on a dedicated std::thread (the symbol
// `NSt6thread11_State_implINS_8_InvokerISt5tupleIJPFPvS3_EP15WebSocketServerEEEEEE`
// is the thread's invoker).
//
// This recreation is a minimal hand-rolled WebSocket broadcaster that:
//   - accepts HTTP/1.1 connections on a configurable port
//   - performs the standard RFC 6455 Sec-WebSocket-Key handshake
//   - broadcasts the latest PipelineResult JSON to every connected client as
//     a text frame whenever pushFrame() is called
//   - honors the "WebSocket timed out from inactivity" behavior observed in
//     the original (60-second idle timeout)
//
// It deliberately does NOT include uWS itself — pulling uWS in would drag
// in libz / picohttpparser / a custom event loop that isn't available in
// the OPi5 Buildroot tree. The wire protocol is identical (RFC 6455) so the
// web UI's WebSocket client doesn't care which server is speaking.

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace limelight {

class WebSocketServer {
public:
    explicit WebSocketServer(int port);
    ~WebSocketServer();

    WebSocketServer(const WebSocketServer&) = delete;
    WebSocketServer& operator=(const WebSocketServer&) = delete;

    // Bind, listen, and spawn the accept thread.
    bool start();
    void stop();

    // Broadcast a text frame to every connected client. Safe to call from
    // the main capture thread.
    void broadcast(const std::string& payload);

    // Convenience: broadcast a {"type":"result", ...} JSON envelope.
    // Called by the main loop once per frame.
    void pushResult(const std::string& result_json);

    // Broadcast a log line to anything subscribed to the WS endpoint. The
    // web UI's console panel consumes these.
    void pushLog(const std::string& line);

    int port() const { return port_; }
    size_t clientCount() const;

private:
    struct Client;
    void acceptLoop();
    void serveClient(std::shared_ptr<Client> c);

    // RFC 6455 frame assembly. Returns an unmasked text frame.
    static std::vector<uint8_t> buildTextFrame(const std::string& payload);
    // RFC 6455 close frame.
    static std::vector<uint8_t> buildCloseFrame();

    int port_;
    int listen_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread accept_thread_;

    mutable std::mutex clients_mu_;
    std::vector<std::shared_ptr<Client>> clients_;
};

}  // namespace limelight
