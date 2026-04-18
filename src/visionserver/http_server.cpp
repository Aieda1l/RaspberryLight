// Hand-rolled HTTP/1.1 server used by the visionserver REST API.
//
// Mirrors the socket-setup pattern recovered from FUN_0019e9e0 (MJPEG stream
// server) — getaddrinfo / socket / SO_REUSEADDR / bind / listen(backlog=5) —
// and wraps it with a route table and request/response parser. See
// src/visionserver/include/visionserver/http_server.h for the design notes.

#include "visionserver/http_server.h"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <sstream>

namespace limelight {

namespace {

// Hard cap on request body size.  Large uploads (pipeline bundles, python
// scripts, ChArUco images) go through /uploadPipeline etc.; anything beyond
// 32 MiB is almost certainly a mistake or an attack.  Returns 413.
constexpr size_t kMaxContentLength = 32 * 1024 * 1024;

// Per-connection idle timeout.  Guards against slow-loris attacks — a
// client that sends 1 byte/minute would otherwise pin a worker thread
// forever.  Applied to both header read and body read.
constexpr int kSocketTimeoutSec = 10;

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

// Read from fd until the buffer contains "\r\n\r\n". Returns the byte offset
// immediately past the header terminator, or 0 on error/close. Guards against
// runaway header sizes (max 64KiB).
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
        dest.append(chunk.data(), static_cast<size_t>(n));
    }
    return true;
}

const char* statusText(int code) {
    switch (code) {
        case 200: return "OK";
        case 201: return "Created";
        case 204: return "No Content";
        case 400: return "Bad Request";
        case 404: return "Not Found";
        case 405: return "Method Not Allowed";
        case 413: return "Payload Too Large";
        case 500: return "Internal Server Error";
        default:  return "OK";
    }
}

}  // namespace

HttpServer::HttpServer() = default;

HttpServer::~HttpServer() {
    stop();
}

void HttpServer::route(const std::string& method, const std::string& path, HttpHandler h) {
    Route r;
    r.method = method;
    r.pattern = path;
    r.handler = std::move(h);
    if (path.size() >= 2 && path[path.size() - 1] == '*' && path[path.size() - 2] == '/') {
        r.wildcard = true;
        r.parts = splitPath(path.substr(0, path.size() - 2));
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
            if (path_parts.size() < r.parts.size()) continue;
            bool ok = true;
            for (size_t i = 0; i < r.parts.size(); ++i) {
                if (r.parts[i] != path_parts[i]) { ok = false; break; }
            }
            if (!ok) continue;
            std::string rest;
            for (size_t i = r.parts.size(); i < path_parts.size(); ++i) {
                if (!rest.empty()) rest.push_back('/');
                rest += path_parts[i];
            }
            req.params["*"] = std::move(rest);
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

bool HttpServer::start(uint16_t port) {
    port_ = port;
    stopping_.store(false);

    addrinfo hints{};
    hints.ai_flags    = AI_PASSIVE;
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    char port_str[16];
    std::snprintf(port_str, sizeof(port_str), "%u", static_cast<unsigned>(port));

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

    accept_thread_ = std::thread([this]() { acceptLoop(); });
    return true;
}

void HttpServer::stop() {
    stopping_.store(true);
    if (listen_fd_ >= 0) {
        ::shutdown(listen_fd_, SHUT_RDWR);
        ::close(listen_fd_);
        listen_fd_ = -1;
    }
    if (accept_thread_.joinable()) accept_thread_.join();
}

void HttpServer::acceptLoop() {
    while (!stopping_.load()) {
        sockaddr_storage peer{};
        socklen_t peer_len = sizeof(peer);
        int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&peer), &peer_len);
        if (fd < 0) {
            if (stopping_.load()) break;
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            continue;
        }
        std::thread([this, fd]() { handleConnection(fd); }).detach();
    }
}

void HttpServer::handleConnection(int fd) {
    // Idle timeout on every recv — kernel-enforced, so a stuck client
    // doesn't leak a thread.  Applies to both the header read below and
    // the body read inside this function.
    struct timeval tv{kSocketTimeoutSec, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    std::string buf;
    size_t header_end = readHttpHeaders(fd, buf);
    if (header_end == 0) {
        ::close(fd);
        return;
    }

    HttpRequest req;
    size_t line_end = buf.find("\r\n");
    if (line_end == std::string::npos) { ::close(fd); return; }
    std::string request_line = buf.substr(0, line_end);

    size_t sp1 = request_line.find(' ');
    size_t sp2 = sp1 == std::string::npos ? std::string::npos
                                          : request_line.find(' ', sp1 + 1);
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

    // Pull the body if Content-Length is set. Tolerate extra bytes already in
    // the header buffer past the end of the headers.
    auto clen_it = req.headers.find("content-length");
    if (clen_it != req.headers.end()) {
        size_t clen = 0;
        try { clen = static_cast<size_t>(std::stoul(clen_it->second)); }
        catch (...) { clen = 0; }
        if (clen > kMaxContentLength) {
            // Refuse oversized requests BEFORE allocating the body.  Reply
            // 413 so the client gets a useful error instead of a dropped
            // connection.
            HttpResponse too_large;
            too_large.status = 413;
            too_large.content_type = "application/json";
            too_large.body = "{\"error\":\"payload too large\"}";
            std::string reply = "HTTP/1.1 413 Payload Too Large\r\n"
                                "Content-Type: application/json\r\n"
                                "Content-Length: " +
                                std::to_string(too_large.body.size()) +
                                "\r\nConnection: close\r\n\r\n" + too_large.body;
            ::send(fd, reply.data(), reply.size(), 0);
            ::close(fd);
            return;
        }
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

    Route* route = nullptr;
    HttpResponse resp;
    if (req.method == "OPTIONS") {
        // Answer CORS preflights uniformly — the web UI relies on this.
        resp.status = 204;
        resp.content_type = "text/plain";
    } else if (!matchRoute(req.method, req.path, &route, req)) {
        resp.status = 404;
        resp.body = "{\"error\":\"not found\"}";
    } else {
        try {
            resp = route->handler(req);
        } catch (const std::exception& e) {
            resp.status = 500;
            resp.body = std::string("{\"error\":\"") + e.what() + "\"}";
        }
    }

    std::ostringstream out;
    out << "HTTP/1.1 " << resp.status << " " << statusText(resp.status) << "\r\n"
        << "Content-Type: " << resp.content_type << "\r\n"
        << "Content-Length: " << resp.body.size() << "\r\n"
        << "Access-Control-Allow-Origin: *\r\n"
        << "Access-Control-Allow-Methods: GET, POST, PATCH, DELETE, OPTIONS\r\n"
        << "Access-Control-Allow-Headers: Content-Type\r\n"
        << "Connection: close\r\n";
    for (auto& kv : resp.extra_headers) {
        out << kv.first << ": " << kv.second << "\r\n";
    }
    out << "\r\n";
    std::string header_str = out.str();
    writeAll(fd, header_str.data(), header_str.size());
    if (!resp.body.empty()) {
        writeAll(fd, resp.body.data(), resp.body.size());
    }
    ::close(fd);
}

}  // namespace limelight
