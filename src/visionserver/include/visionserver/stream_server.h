#pragma once

#include <opencv2/core.hpp>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace limelight {

// MJPEG + raw frame HTTP streaming server.
//
// The original Limelight runs TWO instances of this server:
//     port 5800 (0x16a8) — primary MJPEG stream (annotated frames)
//     port 5802 (0x16aa) — raw video stream server (pre-processing frames)
//
// Recovered from Ghidra (FUN_0019fe60 calls FUN_0019e9e0 twice, once per
// port). The original binary uses getaddrinfo(NULL, port, {AI_PASSIVE,
// SOCK_STREAM}), sets SO_REUSEADDR, TCP_NODELAY, SO_RCVTIMEO=500ms, and
// listens with backlog=5. The same pattern is preserved here.
//
// The server spawns a dedicated accept thread and one per-client worker
// thread. Each client receives a continuous multipart/x-mixed-replace
// MJPEG stream at the rate the main thread calls updateFrame().
class StreamServer {
public:
    // `port`: TCP port to bind.
    explicit StreamServer(int port);
    ~StreamServer();

    StreamServer(const StreamServer&) = delete;
    StreamServer& operator=(const StreamServer&) = delete;

    // Bind, listen, and spawn the accept thread. Returns false if any
    // socket setup step fails.
    bool start();
    void stop();

    // Publish the latest frame. Encoded at the given JPEG quality (0..100)
    // and broadcast to every connected client on the client's own thread.
    // Drops frames silently if called faster than clients can consume.
    void updateFrame(const cv::Mat& frame, int quality = 30);

    int port() const { return port_; }
    size_t clientCount() const;

private:
    struct Client;
    void acceptLoop();
    void serveClient(std::shared_ptr<Client> c);

    int port_;
    int listen_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread accept_thread_;

    mutable std::mutex clients_mutex_;
    std::vector<std::shared_ptr<Client>> clients_;

    // Latest JPEG payload. Re-encoded each updateFrame() call so that
    // every client only holds a reference to the shared buffer.
    mutable std::mutex frame_mutex_;
    std::shared_ptr<std::vector<uint8_t>> latest_jpeg_;
    uint64_t frame_seq_ = 0;
};

}  // namespace limelight
