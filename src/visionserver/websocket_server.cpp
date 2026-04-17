// WebSocket broadcast server for the visionserver (port 5805 historically).
//
// Recreated from the uWS-based `WebSocketServer` class in the original
// binary. See src/visionserver/include/visionserver/websocket_server.h for
// the design rationale and the raw_strings.txt references that prove the
// original uses uWebSockets with a PerSocketData / StreamState / PackProgress
// trio. This implementation is a hand-rolled minimal RFC 6455 server that
// exposes the same observable behavior on the wire without linking uWS.

#include "visionserver/websocket_server.h"

#include "ws_crypto.h"

#include <spdlog/spdlog.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace limelight {

// ---- Per-client state -----------------------------------------------------

struct WebSocketServer::Client {
    int fd = -1;
    std::thread worker;
    std::atomic<bool> running{true};
    std::mutex send_mu;        // Serialize frame writes from multiple threads.
};

namespace {

// ---- String helpers -------------------------------------------------------

std::string toLower(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    return out;
}

std::string trimWs(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && (s[a] == ' ' || s[a] == '\t')) ++a;
    while (b > a && (s[b-1] == ' ' || s[b-1] == '\t' || s[b-1] == '\r')) --b;
    return s.substr(a, b - a);
}

// ---- Socket I/O helpers ---------------------------------------------------

bool writeAll(int fd, const void* data, size_t len) {
    const char* p = static_cast<const char*>(data);
    while (len > 0) {
        ssize_t n = ::send(fd, p, len, MSG_NOSIGNAL);
        if (n <= 0) {
            if (n < 0 && errno == EINTR) continue;
            return false;
        }
        p += n;
        len -= static_cast<size_t>(n);
    }
    return true;
}

size_t readHttpHeaders(int fd, std::string& buf) {
    std::array<char, 4096> chunk;
    while (true) {
        size_t pos = buf.find("\r\n\r\n");
        if (pos != std::string::npos) return pos + 4;
        if (buf.size() > 64 * 1024) return 0;
        ssize_t n = ::recv(fd, chunk.data(), chunk.size(), 0);
        if (n <= 0) {
            if (n < 0 && errno == EINTR) continue;
            return 0;
        }
        buf.append(chunk.data(), static_cast<size_t>(n));
    }
}

}  // namespace

// ---- Frame builders -------------------------------------------------------

std::vector<uint8_t> WebSocketServer::buildTextFrame(const std::string& payload) {
    std::vector<uint8_t> frame;
    frame.reserve(payload.size() + 10);
    frame.push_back(0x81);  // FIN=1, opcode=0x1 (text)
    size_t n = payload.size();
    if (n < 126) {
        frame.push_back(static_cast<uint8_t>(n));
    } else if (n <= 0xffff) {
        frame.push_back(126);
        frame.push_back(static_cast<uint8_t>((n >> 8) & 0xff));
        frame.push_back(static_cast<uint8_t>(n & 0xff));
    } else {
        frame.push_back(127);
        for (int i = 7; i >= 0; --i) {
            frame.push_back(static_cast<uint8_t>((n >> (8 * i)) & 0xff));
        }
    }
    frame.insert(frame.end(), payload.begin(), payload.end());
    return frame;
}

std::vector<uint8_t> WebSocketServer::buildCloseFrame() {
    // Opcode 0x8 (close), empty body.
    return {0x88, 0x00};
}

// ---- WebSocketServer ------------------------------------------------------

WebSocketServer::WebSocketServer(int port) : port_(port) {}

WebSocketServer::~WebSocketServer() { stop(); }

bool WebSocketServer::start() {
    char port_str[16];
    std::snprintf(port_str, sizeof(port_str), "%d", port_);

    addrinfo hints{};
    hints.ai_flags    = AI_PASSIVE;
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo* res = nullptr;
    if (::getaddrinfo(nullptr, port_str, &hints, &res) != 0) return false;

    listen_fd_ = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (listen_fd_ < 0) { ::freeaddrinfo(res); return false; }

    int yes = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    ::setsockopt(listen_fd_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

    if (::bind(listen_fd_, res->ai_addr, res->ai_addrlen) != 0) {
        ::freeaddrinfo(res);
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    ::freeaddrinfo(res);

    if (::listen(listen_fd_, 5) != 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }

    running_ = true;
    accept_thread_ = std::thread([this]() { acceptLoop(); });
    return true;
}

void WebSocketServer::stop() {
    running_ = false;
    if (listen_fd_ != -1) {
        ::shutdown(listen_fd_, SHUT_RDWR);
        ::close(listen_fd_);
        listen_fd_ = -1;
    }
    if (accept_thread_.joinable()) accept_thread_.join();

    std::vector<std::shared_ptr<Client>> to_join;
    {
        std::lock_guard<std::mutex> lk(clients_mu_);
        to_join = std::move(clients_);
        clients_.clear();
    }
    for (auto& c : to_join) {
        c->running = false;
        if (c->fd != -1) { ::shutdown(c->fd, SHUT_RDWR); ::close(c->fd); c->fd = -1; }
        if (c->worker.joinable()) c->worker.join();
    }
}

void WebSocketServer::acceptLoop() {
    while (running_) {
        sockaddr_storage peer{};
        socklen_t peer_len = sizeof(peer);
        int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&peer), &peer_len);
        if (fd < 0) {
            if (!running_) break;
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            spdlog::warn("WebSocketServer accept failed: {}", std::strerror(errno));
            continue;
        }
        auto client = std::make_shared<Client>();
        client->fd = fd;
        {
            std::lock_guard<std::mutex> lk(clients_mu_);
            clients_.push_back(client);
        }
        client->worker = std::thread(&WebSocketServer::serveClient, this, client);
    }
}

void WebSocketServer::serveClient(std::shared_ptr<Client> c) {
    // Perform the HTTP/1.1 upgrade handshake.
    std::string buf;
    if (readHttpHeaders(c->fd, buf) == 0) {
        ::close(c->fd);
        c->fd = -1;
        return;
    }

    // Parse headers to find Sec-WebSocket-Key.
    std::string sec_key;
    {
        size_t pos = buf.find("\r\n");
        if (pos == std::string::npos) { ::close(c->fd); c->fd = -1; return; }
        pos += 2;
        while (pos < buf.size()) {
            size_t eol = buf.find("\r\n", pos);
            if (eol == std::string::npos || eol == pos) break;
            std::string line = buf.substr(pos, eol - pos);
            pos = eol + 2;
            size_t colon = line.find(':');
            if (colon == std::string::npos) continue;
            std::string name = toLower(trimWs(line.substr(0, colon)));
            std::string value = trimWs(line.substr(colon + 1));
            if (name == "sec-websocket-key") sec_key = value;
        }
    }

    if (sec_key.empty()) {
        const char* resp = "HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n";
        writeAll(c->fd, resp, std::strlen(resp));
        ::close(c->fd);
        c->fd = -1;
        return;
    }

    std::string accept_key = base64Encode(
        sha1String(sec_key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"));

    std::ostringstream resp;
    resp << "HTTP/1.1 101 Switching Protocols\r\n"
         << "Upgrade: websocket\r\n"
         << "Connection: Upgrade\r\n"
         << "Sec-WebSocket-Accept: " << accept_key << "\r\n\r\n";
    std::string resp_str = resp.str();
    if (!writeAll(c->fd, resp_str.data(), resp_str.size())) {
        ::close(c->fd);
        c->fd = -1;
        return;
    }

    // Enforce a 60-second inactivity timeout (matches the original binary's
    // "WebSocket timed out from inactivity" log).
    timeval tv{60, 0};
    ::setsockopt(c->fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Drain incoming frames until the peer closes. We don't do anything with
    // client-sent payloads — the endpoint is push-only.
    while (c->running && running_) {
        uint8_t hdr[2];
        ssize_t n = ::recv(c->fd, hdr, 2, MSG_WAITALL);
        if (n <= 0) {
            // Either error, timeout, or clean close — bail out.
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                spdlog::debug("WebSocket timed out from inactivity");
            }
            break;
        }

        uint8_t opcode = hdr[0] & 0x0f;
        bool    masked = (hdr[1] & 0x80) != 0;
        uint64_t plen  = hdr[1] & 0x7f;

        if (plen == 126) {
            uint8_t ext[2];
            if (::recv(c->fd, ext, 2, MSG_WAITALL) != 2) break;
            plen = (uint64_t(ext[0]) << 8) | ext[1];
        } else if (plen == 127) {
            uint8_t ext[8];
            if (::recv(c->fd, ext, 8, MSG_WAITALL) != 8) break;
            plen = 0;
            for (int i = 0; i < 8; ++i) plen = (plen << 8) | ext[i];
        }

        uint8_t mask[4] = {0, 0, 0, 0};
        if (masked) {
            if (::recv(c->fd, mask, 4, MSG_WAITALL) != 4) break;
        }

        // Cap per-frame payload to keep a hostile peer from requesting a
        // multi-gigabyte allocation on the push-only endpoint.
        constexpr uint64_t kMaxFramePayload = 1ull * 1024 * 1024;  // 1 MiB
        if (plen > kMaxFramePayload) break;

        // Drain payload without interpreting it.
        std::vector<uint8_t> payload(static_cast<size_t>(plen));
        size_t got = 0;
        while (got < plen) {
            ssize_t m = ::recv(c->fd, payload.data() + got, plen - got, 0);
            if (m <= 0) { opcode = 0x8; break; }
            got += static_cast<size_t>(m);
        }

        // RFC 6455 requires client-to-server frames to be masked. Unmask once
        // so the payload is usable for (e.g.) correct ping/pong echoes and
        // any future interpretation of client messages.
        if (masked) {
            for (size_t j = 0; j < payload.size(); ++j) {
                payload[j] ^= mask[j & 3];
            }
        }

        if (opcode == 0x8) break;          // close
        if (opcode == 0x9) {               // ping -> pong
            std::vector<uint8_t> pong;
            pong.push_back(0x8a);          // FIN | pong
            pong.push_back(static_cast<uint8_t>(payload.size()));
            pong.insert(pong.end(), payload.begin(), payload.end());
            std::lock_guard<std::mutex> lk(c->send_mu);
            if (!writeAll(c->fd, pong.data(), pong.size())) break;
        }
        // text/binary/pong frames are ignored — this endpoint is push-only.
    }

    // Try to write a close frame before hanging up (best-effort).
    {
        std::lock_guard<std::mutex> lk(c->send_mu);
        auto close_frame = buildCloseFrame();
        writeAll(c->fd, close_frame.data(), close_frame.size());
    }

    if (c->fd != -1) { ::close(c->fd); c->fd = -1; }
    c->running = false;

    // Remove self from the client list. Detach our worker because we ARE it.
    std::lock_guard<std::mutex> lk(clients_mu_);
    auto it = std::find(clients_.begin(), clients_.end(), c);
    if (it != clients_.end()) {
        it->get()->worker.detach();
        clients_.erase(it);
    }
}

void WebSocketServer::broadcast(const std::string& payload) {
    auto frame = buildTextFrame(payload);

    std::vector<std::shared_ptr<Client>> snapshot;
    {
        std::lock_guard<std::mutex> lk(clients_mu_);
        snapshot = clients_;
    }

    for (auto& c : snapshot) {
        if (!c->running.load() || c->fd == -1) continue;
        std::lock_guard<std::mutex> lk(c->send_mu);
        if (!writeAll(c->fd, frame.data(), frame.size())) {
            c->running = false;
        }
    }
}

void WebSocketServer::pushResult(const std::string& result_json) {
    // Envelope the payload so the web UI can discriminate frame vs. log.
    std::string env;
    env.reserve(result_json.size() + 32);
    env.append("{\"type\":\"result\",\"data\":");
    env.append(result_json);
    env.append("}");
    broadcast(env);
}

void WebSocketServer::pushLog(const std::string& line) {
    // Escape embedded quotes/backslashes to keep the envelope valid JSON.
    std::string escaped;
    escaped.reserve(line.size() + 16);
    for (char ch : line) {
        switch (ch) {
            case '"':  escaped.append("\\\""); break;
            case '\\': escaped.append("\\\\"); break;
            case '\n': escaped.append("\\n");  break;
            case '\r': escaped.append("\\r");  break;
            case '\t': escaped.append("\\t");  break;
            default:
                if (static_cast<unsigned char>(ch) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", ch);
                    escaped.append(buf);
                } else {
                    escaped.push_back(ch);
                }
        }
    }
    std::string env = "{\"type\":\"log\",\"line\":\"" + escaped + "\"}";
    broadcast(env);
}

size_t WebSocketServer::clientCount() const {
    std::lock_guard<std::mutex> lk(clients_mu_);
    return clients_.size();
}

}  // namespace limelight
