// MJPEG HTTP streaming server.
//
// Recreated from Ghidra decompilation of FUN_0019e9e0 (per-port constructor)
// and FUN_0019fe60 (creates two instances on 5800 and 5802) in the original
// visionserver binary.
//
// Observed socket setup sequence from decompilation:
//   1. puts("getaddrinfo");
//   2. sprintf(portStr, "%d", port);
//   3. getaddrinfo(NULL, portStr, {ai_flags=AI_PASSIVE, ai_socktype=SOCK_STREAM}, &res);
//   4. puts("creating listening socket");
//   5. socket(res->ai_family, res->ai_socktype, res->ai_protocol);
//   6. setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, 1);
//   7. setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, 1);
//   8. setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, {0s, 500000us});
//   9. puts("binding port"); bind(fd, res->ai_addr, res->ai_addrlen);
//   10. puts("listening.."); listen(fd, 5);
//
// Client protocol: a minimal HTTP/1.0 multipart/x-mixed-replace MJPEG
// stream. The original uses the same protocol (confirmed by the web UI's
// <img src="http://limelight.local:5800/"> tag).

#include "visionserver/stream_server.h"

#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace limelight {

struct StreamServer::Client {
    int fd = -1;
    std::thread worker;
    std::atomic<bool> running{true};
    uint64_t last_seq = 0;
};

StreamServer::StreamServer(int port) : port_(port) {}

StreamServer::~StreamServer() { stop(); }

bool StreamServer::start() {
    spdlog::debug("StreamServer[{}]: getaddrinfo", port_);

    char port_str[16];
    std::snprintf(port_str, sizeof(port_str), "%d", port_);

    addrinfo hints{};
    hints.ai_flags    = AI_PASSIVE;
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo* res = nullptr;
    int rc = ::getaddrinfo(nullptr, port_str, &hints, &res);
    if (rc != 0) {
        spdlog::error("StreamServer[{}]: getaddrinfo: {}", port_, ::gai_strerror(rc));
        return false;
    }

    spdlog::debug("StreamServer[{}]: creating listening socket", port_);
    listen_fd_ = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (listen_fd_ == -1) {
        spdlog::error("StreamServer[{}]: socket() failed, errno={}", port_, errno);
        ::freeaddrinfo(res);
        return false;
    }

    int on = 1;
    if (::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) == -1) {
        spdlog::warn("StreamServer[{}]: SO_REUSEADDR failed", port_);
    }
    if (::setsockopt(listen_fd_, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on)) == -1) {
        spdlog::warn("StreamServer[{}]: TCP_NODELAY failed", port_);
    }

    timeval tv{0, 500000};
    if (::setsockopt(listen_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == -1) {
        spdlog::warn("StreamServer[{}]: SO_RCVTIMEO failed", port_);
    }

    spdlog::debug("StreamServer[{}]: binding port", port_);
    if (::bind(listen_fd_, res->ai_addr, res->ai_addrlen) == -1) {
        spdlog::error("StreamServer[{}]: bind failed, errno={}", port_, errno);
        ::freeaddrinfo(res);
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    ::freeaddrinfo(res);

    spdlog::debug("StreamServer[{}]: listening", port_);
    if (::listen(listen_fd_, 5) == -1) {
        spdlog::error("StreamServer[{}]: listen failed, errno={}", port_, errno);
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }

    running_ = true;
    accept_thread_ = std::thread([this]() { acceptLoop(); });
    return true;
}

void StreamServer::stop() {
    running_ = false;
    if (listen_fd_ != -1) {
        ::shutdown(listen_fd_, SHUT_RDWR);
        ::close(listen_fd_);
        listen_fd_ = -1;
    }
    if (accept_thread_.joinable()) accept_thread_.join();

    std::vector<std::shared_ptr<Client>> to_join;
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        to_join = std::move(clients_);
        clients_.clear();
    }
    for (auto& c : to_join) {
        c->running = false;
        if (c->fd != -1) { ::shutdown(c->fd, SHUT_RDWR); ::close(c->fd); c->fd = -1; }
        if (c->worker.joinable()) c->worker.join();
    }
}

void StreamServer::acceptLoop() {
    while (running_) {
        sockaddr_storage addr{};
        socklen_t addrlen = sizeof(addr);
        int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &addrlen);
        if (fd == -1) {
            if (!running_) break;
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            spdlog::warn("StreamServer accept failed: {}", std::strerror(errno));
            continue;
        }
        auto client = std::make_shared<Client>();
        client->fd = fd;
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            clients_.push_back(client);
        }
        client->worker = std::thread(&StreamServer::serveClient, this, client);
    }
}

namespace {

// Read and drop the HTTP request line + headers (we accept any path and
// any method — the original Limelight ignores them too).
bool consumeHttpRequest(int fd) {
    char buf[1024];
    std::string accum;
    while (accum.find("\r\n\r\n") == std::string::npos) {
        ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) return false;
        accum.append(buf, static_cast<size_t>(n));
        if (accum.size() > 16384) return false;
    }
    return true;
}

bool sendAll(int fd, const void* data, size_t len) {
    const char* p = static_cast<const char*>(data);
    while (len > 0) {
        ssize_t n = ::send(fd, p, len, MSG_NOSIGNAL);
        if (n <= 0) return false;
        p += n;
        len -= static_cast<size_t>(n);
    }
    return true;
}

const char* kBoundary = "limelightboundary";

}  // namespace

void StreamServer::serveClient(std::shared_ptr<Client> c) {
    if (!consumeHttpRequest(c->fd)) {
        ::close(c->fd);
        c->fd = -1;
        return;
    }

    // MJPEG response header.
    std::string header =
        "HTTP/1.0 200 OK\r\n"
        "Server: limelight-visionserver\r\n"
        "Connection: close\r\n"
        "Max-Age: 0\r\n"
        "Expires: 0\r\n"
        "Cache-Control: no-cache, private\r\n"
        "Pragma: no-cache\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=--";
    header += kBoundary;
    header += "\r\n\r\n";

    if (!sendAll(c->fd, header.data(), header.size())) {
        c->running = false;
    }

    while (c->running && running_) {
        std::shared_ptr<std::vector<uint8_t>> jpeg;
        uint64_t seq = 0;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            jpeg = latest_jpeg_;
            seq  = frame_seq_;
        }
        if (!jpeg || seq == c->last_seq) {
            // Sleep for ~10ms to avoid busy-spinning when no frame is ready.
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        c->last_seq = seq;

        char part_header[256];
        int hlen = std::snprintf(
            part_header, sizeof(part_header),
            "--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
            kBoundary, jpeg->size());
        if (hlen <= 0) { c->running = false; break; }

        if (!sendAll(c->fd, part_header, static_cast<size_t>(hlen))) { c->running = false; break; }
        if (!sendAll(c->fd, jpeg->data(), jpeg->size()))              { c->running = false; break; }
        if (!sendAll(c->fd, "\r\n", 2))                                { c->running = false; break; }
    }

    if (c->fd != -1) { ::close(c->fd); c->fd = -1; }

    // Remove ourselves from the client list. Use a weak pointer equivalent
    // via raw pointer comparison — we're the only holder of `c` still alive.
    std::lock_guard<std::mutex> lock(clients_mutex_);
    auto it = std::find(clients_.begin(), clients_.end(), c);
    if (it != clients_.end()) {
        // Detach our worker before erase — we ARE that worker.
        it->get()->worker.detach();
        clients_.erase(it);
    }
}

void StreamServer::updateFrame(const cv::Mat& frame, int quality) {
    if (frame.empty()) return;

    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, std::clamp(quality, 1, 100)};
    auto buf = std::make_shared<std::vector<uint8_t>>();
    if (!cv::imencode(".jpg", frame, *buf, params)) return;

    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_jpeg_ = std::move(buf);
    ++frame_seq_;
}

size_t StreamServer::clientCount() const {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    return clients_.size();
}

}  // namespace limelight
