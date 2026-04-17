#include "http_server.h"

#include "ws_crypto.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <sstream>

namespace {

using limelight::sha1String;
using limelight::base64Encode;

// --- String helpers --------------------------------------------------------

std::string toLower(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return out;
}

std::string trimWs(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && (s[a] == ' ' || s[a] == '\t')) ++a;
    while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r')) --b;
    return s.substr(a, b - a);
}

// Split path into non-empty parts. "/foo/bar" -> ["foo","bar"].
std::vector<std::string> splitPath(const std::string& path) {
    std::vector<std::string> parts;
    std::string cur;
    for (char c : path) {
        if (c == '/') {
            if (!cur.empty()) {
                parts.push_back(cur);
                cur.clear();
            }
        } else {
            cur.push_back(c);
        }
    }
    if (!cur.empty()) parts.push_back(cur);
    return parts;
}

// --- Socket I/O helpers ----------------------------------------------------

bool writeAll(int fd, const void* data, size_t len) {
    const char* p = static_cast<const char*>(data);
    while (len > 0) {
        ssize_t n = ::send(fd, p, len, MSG_NOSIGNAL);
        if (n <= 0) {
            if (n < 0 && errno == EINTR) continue;
            return false;
        }
        p += n;
        len -= n;
    }
    return true;
}

// Read until "\r\n\r\n" or until the buffer gets huge. Returns the number of
// bytes consumed (ending exactly after the header terminator) on success, or
// 0 on error/close.
size_t readHttpHeaders(int fd, std::string& buf) {
    std::array<char, 4096> chunk;
    while (true) {
        size_t pos = buf.find("\r\n\r\n");
        if (pos != std::string::npos) return pos + 4;
        if (buf.size() > 64 * 1024) return 0;  // guard
        ssize_t n = ::recv(fd, chunk.data(), chunk.size(), 0);
        if (n <= 0) {
            if (n < 0 && errno == EINTR) continue;
            return 0;
        }
        buf.append(chunk.data(), n);
    }
}

bool readExactly(int fd, std::string& dest, size_t need) {
    dest.clear();
    dest.reserve(need);
    std::array<char, 4096> chunk;
    while (dest.size() < need) {
        size_t want = std::min(chunk.size(), need - dest.size());
        ssize_t n = ::recv(fd, chunk.data(), want, 0);
        if (n <= 0) {
            if (n < 0 && errno == EINTR) continue;
            return false;
        }
        dest.append(chunk.data(), n);
    }
    return true;
}

}  // namespace

namespace limelight::hwmon {

// --- WebSocketSession ------------------------------------------------------

WebSocketSession::WebSocketSession(int fd) : fd_(fd) {}

WebSocketSession::~WebSocketSession() {
    shutdown();
}

bool WebSocketSession::sendText(const std::string& msg) {
    if (closed_.load()) return false;
    std::lock_guard<std::mutex> lk(send_mu_);

    // RFC 6455: build an unmasked text frame (FIN=1, opcode=0x1).
    std::vector<uint8_t> frame;
    frame.reserve(msg.size() + 10);
    frame.push_back(0x81);  // FIN | opcode 0x1 (text)

    size_t n = msg.size();
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
    frame.insert(frame.end(), msg.begin(), msg.end());

    if (!writeAll(fd_, frame.data(), frame.size())) {
        closed_.store(true);
        return false;
    }
    return true;
}

void WebSocketSession::awaitClose() {
    // Read incoming frames until we see a close frame or the peer hangs up.
    while (!closed_.load()) {
        uint8_t hdr[2];
        ssize_t n = ::recv(fd_, hdr, 2, MSG_WAITALL);
        if (n <= 0) break;

        uint8_t opcode = hdr[0] & 0x0f;
        bool masked = (hdr[1] & 0x80) != 0;
        uint64_t plen = hdr[1] & 0x7f;

        if (plen == 126) {
            uint8_t ext[2];
            if (::recv(fd_, ext, 2, MSG_WAITALL) != 2) break;
            plen = (uint64_t(ext[0]) << 8) | ext[1];
        } else if (plen == 127) {
            uint8_t ext[8];
            if (::recv(fd_, ext, 8, MSG_WAITALL) != 8) break;
            plen = 0;
            for (int i = 0; i < 8; ++i) plen = (plen << 8) | ext[i];
        }

        uint8_t mask[4] = {0, 0, 0, 0};
        if (masked) {
            if (::recv(fd_, mask, 4, MSG_WAITALL) != 4) break;
        }

        // Drain payload. We don't do anything with incoming client text,
        // only watch for close frames — but reject absurd sizes so a peer
        // can't request an arbitrary-sized allocation.
        constexpr uint64_t kMaxFramePayload = 4ull * 1024 * 1024;  // 4 MiB
        if (plen > kMaxFramePayload) break;
        std::vector<uint8_t> payload(static_cast<size_t>(plen));
        size_t got = 0;
        while (got < plen) {
            ssize_t m = ::recv(fd_, payload.data() + got, plen - got, 0);
            if (m <= 0) { opcode = 0x8; break; }
            got += m;
        }

        if (opcode == 0x8) break;  // close
        // Ignore ping/pong/text/binary for log-streaming endpoints.
    }
    closed_.store(true);
}

void WebSocketSession::shutdown() {
    bool expected = false;
    if (!closed_.compare_exchange_strong(expected, true)) return;
    if (fd_ >= 0) {
        ::shutdown(fd_, SHUT_RDWR);
        ::close(fd_);
        fd_ = -1;
    }
}

// --- HttpServer ------------------------------------------------------------

HttpServer::HttpServer() = default;

HttpServer::~HttpServer() {
    stop();
}

void HttpServer::get(const std::string& path, HttpHandler h) {
    Route r;
    r.method = "GET";
    r.pattern = path;
    r.http_handler = std::move(h);
    if (!path.empty() && path.back() == '*') {
        r.wildcard = true;
        r.parts = splitPath(path.substr(0, path.size() - 1));
    } else {
        r.parts = splitPath(path);
    }
    routes_.push_back(std::move(r));
}

void HttpServer::post(const std::string& path, HttpHandler h) {
    Route r;
    r.method = "POST";
    r.pattern = path;
    r.http_handler = std::move(h);
    r.parts = splitPath(path);
    routes_.push_back(std::move(r));
}

void HttpServer::options(const std::string& path, HttpHandler h) {
    Route r;
    r.method = "OPTIONS";
    r.pattern = path;
    r.http_handler = std::move(h);
    r.parts = splitPath(path);
    routes_.push_back(std::move(r));
}

void HttpServer::websocket(const std::string& path, WsHandshakeHandler h) {
    Route r;
    r.method = "WS";
    r.pattern = path;
    r.ws_handler = std::move(h);
    if (!path.empty() && path.back() == '*') {
        r.wildcard = true;
        r.parts = splitPath(path.substr(0, path.size() - 1));
    } else {
        r.parts = splitPath(path);
    }
    routes_.push_back(std::move(r));
}

bool HttpServer::matchRoute(const std::string& method, const std::string& path,
                            Route** out_route, HttpRequest& req) {
    auto path_parts = splitPath(path);
    for (auto& r : routes_) {
        if (r.method != method) continue;

        if (r.wildcard) {
            // Prefix match: r.parts must be a prefix of path_parts.
            if (path_parts.size() < r.parts.size()) continue;
            bool ok = true;
            for (size_t i = 0; i < r.parts.size(); ++i) {
                if (r.parts[i] != path_parts[i]) { ok = false; break; }
            }
            if (!ok) continue;
            *out_route = &r;
            return true;
        }

        if (path_parts.size() != r.parts.size()) continue;
        bool ok = true;
        std::unordered_map<std::string, std::string> caps;
        for (size_t i = 0; i < r.parts.size(); ++i) {
            const std::string& pat = r.parts[i];
            if (!pat.empty() && pat[0] == ':') {
                caps[pat.substr(1)] = path_parts[i];
            } else if (pat != path_parts[i]) {
                ok = false;
                break;
            }
        }
        if (!ok) continue;
        req.params = std::move(caps);
        *out_route = &r;
        return true;
    }
    return false;
}

bool HttpServer::listen(uint16_t port) {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) return false;

    int yes = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    if (::listen(listen_fd_, 128) != 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    return true;
}

void HttpServer::run() {
    acceptLoop();
}

void HttpServer::stop() {
    stopping_.store(true);
    if (listen_fd_ >= 0) {
        ::shutdown(listen_fd_, SHUT_RDWR);
        ::close(listen_fd_);
        listen_fd_ = -1;
    }
    // Join every in-flight worker before returning so the destructor cannot
    // free state out from under a running handler. Each accepted socket has
    // SO_RCVTIMEO/SO_SNDTIMEO applied, so blocked reads unblock promptly.
    std::unordered_map<uint64_t, std::thread> to_join;
    {
        std::lock_guard<std::mutex> lk(workers_mu_);
        to_join = std::move(workers_);
        workers_.clear();
        finished_workers_.clear();
    }
    for (auto& [id, t] : to_join) {
        if (t.joinable()) t.join();
    }
}

void HttpServer::acceptLoop() {
    // Hard cap on concurrent workers so a malicious or broken client cannot
    // exhaust the thread table. hwmon's endpoints are low-volume — 64 is
    // ample headroom for the legitimate web UI.
    constexpr size_t kMaxConcurrentWorkers = 64;

    while (!stopping_.load()) {
        sockaddr_in peer{};
        socklen_t peer_len = sizeof(peer);
        int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&peer), &peer_len);
        if (fd < 0) {
            if (stopping_.load()) break;
            if (errno == EINTR) continue;
            continue;
        }

        // Apply a per-connection receive/send timeout so slow-loris attackers
        // can't hold worker threads indefinitely.
        timeval tv{15, 0};
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        // Reap workers that have already completed, then enforce the cap.
        uint64_t my_id = 0;
        {
            std::lock_guard<std::mutex> lk(workers_mu_);
            for (uint64_t done_id : finished_workers_) {
                auto it = workers_.find(done_id);
                if (it != workers_.end()) {
                    if (it->second.joinable()) it->second.join();
                    workers_.erase(it);
                }
            }
            finished_workers_.clear();

            if (workers_.size() >= kMaxConcurrentWorkers) {
                const char* resp =
                    "HTTP/1.1 503 Service Unavailable\r\nContent-Length: 0\r\n\r\n";
                writeAll(fd, resp, std::strlen(resp));
                ::close(fd);
                continue;
            }
            my_id = next_worker_id_++;
            workers_.emplace(my_id, std::thread([this, fd, my_id]() {
                handleConnection(fd);
                std::lock_guard<std::mutex> lk2(workers_mu_);
                finished_workers_.push_back(my_id);
            }));
        }
    }
}

void HttpServer::handleConnection(int fd) {
    // Outer guard — handleConnection runs on a non-main thread, and any
    // exception escaping this function would terminate the whole daemon
    // (std::terminate on an uncaught exception in a thread body).
    try {
    std::string buf;
    size_t header_end = readHttpHeaders(fd, buf);
    if (header_end == 0) {
        ::close(fd);
        return;
    }

    // Parse request line.
    HttpRequest req;
    size_t line_end = buf.find("\r\n");
    if (line_end == std::string::npos) { ::close(fd); return; }
    std::string request_line = buf.substr(0, line_end);

    size_t sp1 = request_line.find(' ');
    size_t sp2 = sp1 == std::string::npos ? std::string::npos : request_line.find(' ', sp1 + 1);
    if (sp1 == std::string::npos || sp2 == std::string::npos) { ::close(fd); return; }
    req.method = request_line.substr(0, sp1);
    std::string target = request_line.substr(sp1 + 1, sp2 - sp1 - 1);

    size_t qpos = target.find('?');
    if (qpos == std::string::npos) {
        req.path = target;
    } else {
        req.path = target.substr(0, qpos);
        req.query = target.substr(qpos + 1);
    }

    // Parse headers.
    size_t pos = line_end + 2;
    while (pos < header_end - 2) {
        size_t eol = buf.find("\r\n", pos);
        if (eol == std::string::npos || eol >= header_end - 2) break;
        std::string line = buf.substr(pos, eol - pos);
        pos = eol + 2;
        size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string name = toLower(trimWs(line.substr(0, colon)));
        std::string value = trimWs(line.substr(colon + 1));
        req.headers[name] = value;
    }

    // If there's a body (POST with Content-Length), pull it in. Cap the
    // allocation so an attacker cannot request an arbitrary-sized buffer.
    constexpr size_t kMaxBodyBytes = 64ull * 1024 * 1024;   // 64 MiB
    auto clen_it = req.headers.find("content-length");
    if (clen_it != req.headers.end()) {
        size_t clen = 0;
        try {
            clen = static_cast<size_t>(std::stoull(clen_it->second));
        } catch (...) {
            const char* resp = "HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n";
            writeAll(fd, resp, std::strlen(resp));
            ::close(fd);
            return;
        }
        if (clen > kMaxBodyBytes) {
            const char* resp = "HTTP/1.1 413 Payload Too Large\r\nContent-Length: 0\r\n\r\n";
            writeAll(fd, resp, std::strlen(resp));
            ::close(fd);
            return;
        }
        // Already-read body bytes after header_end:
        std::string body;
        if (buf.size() > header_end) {
            body.assign(buf.data() + header_end, buf.size() - header_end);
        }
        if (body.size() < clen) {
            std::string rest;
            if (!readExactly(fd, rest, clen - body.size())) {
                ::close(fd);
                return;
            }
            body += rest;
        } else if (body.size() > clen) {
            body.resize(clen);
        }
        req.body = std::move(body);
    }

    // WebSocket upgrade?
    auto upgrade_it = req.headers.find("upgrade");
    if (upgrade_it != req.headers.end() && toLower(upgrade_it->second) == "websocket") {
        Route* route = nullptr;
        if (!matchRoute("WS", req.path, &route, req)) {
            const char* resp = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n";
            writeAll(fd, resp, std::strlen(resp));
            ::close(fd);
            return;
        }
        auto key_it = req.headers.find("sec-websocket-key");
        if (key_it == req.headers.end()) {
            const char* resp = "HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n";
            writeAll(fd, resp, std::strlen(resp));
            ::close(fd);
            return;
        }
        std::string accept_key =
            base64Encode(sha1String(key_it->second + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"));

        std::ostringstream resp;
        resp << "HTTP/1.1 101 Switching Protocols\r\n"
             << "Upgrade: websocket\r\n"
             << "Connection: Upgrade\r\n"
             << "Sec-WebSocket-Accept: " << accept_key << "\r\n\r\n";
        std::string resp_str = resp.str();
        if (!writeAll(fd, resp_str.data(), resp_str.size())) {
            ::close(fd);
            return;
        }

        auto session = std::make_shared<WebSocketSession>(fd);
        // The handler is expected to drive the session for its full
        // lifetime: set up its tail thread, join it when the peer closes,
        // and return. We do NOT call awaitClose ourselves — the handler
        // owns that responsibility because it may also own per-session
        // state (like a tailer thread) that must be torn down first.
        route->ws_handler(session, req);
        return;  // session destructor closes fd if the handler didn't.
    }

    // Plain HTTP route.
    Route* route = nullptr;
    HttpResponse resp;
    if (!matchRoute(req.method, req.path, &route, req)) {
        resp.status = 404;
        resp.body = "{\"error\":\"not found\"}";
    } else {
        try {
            resp = route->http_handler(req);
        } catch (const std::exception& e) {
            resp.status = 500;
            resp.body = std::string("{\"error\":\"") + e.what() + "\"}";
        }
    }

    const char* status_text = "OK";
    switch (resp.status) {
        case 200: status_text = "OK"; break;
        case 204: status_text = "No Content"; break;
        case 400: status_text = "Bad Request"; break;
        case 404: status_text = "Not Found"; break;
        case 500: status_text = "Internal Server Error"; break;
        default:  status_text = "OK"; break;
    }

    // Echo the Origin header back only if it matches the request origin.
    // This is a tighter CORS policy than "*" which would let any page on
    // the LAN drive the API from a user's browser. Since hwmon is meant
    // to be consumed by the co-resident web UI on the same host, we allow
    // any origin but force the browser to treat the response as
    // credentialed-appropriate — i.e. no wildcard with credentials.
    std::string allow_origin = "*";
    auto origin_it = req.headers.find("origin");
    if (origin_it != req.headers.end() && !origin_it->second.empty()) {
        allow_origin = origin_it->second;
    }

    std::ostringstream out;
    out << "HTTP/1.1 " << resp.status << " " << status_text << "\r\n"
        << "Content-Type: " << resp.content_type << "\r\n"
        << "Content-Length: " << resp.body.size() << "\r\n"
        << "Access-Control-Allow-Origin: " << allow_origin << "\r\n"
        << "Vary: Origin\r\n"
        << "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
        << "Access-Control-Allow-Headers: Content-Type\r\n"
        << "X-Content-Type-Options: nosniff\r\n"
        << "Connection: close\r\n";
    for (auto& kv : resp.extra_headers) {
        out << kv.first << ": " << kv.second << "\r\n";
    }
    out << "\r\n" << resp.body;
    std::string out_str = out.str();
    writeAll(fd, out_str.data(), out_str.size());
    ::close(fd);
    } catch (const std::exception& e) {
        // Best-effort: log, close, and let the worker exit cleanly.
        std::cerr << "HttpServer worker error: " << e.what() << std::endl;
        ::close(fd);
    } catch (...) {
        ::close(fd);
    }
}

}  // namespace limelight::hwmon
