#include "http_server.h"

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

// Minimal SHA1 + base64 implementations for the WebSocket handshake. We avoid
// pulling in a full crypto dependency for this one use.
namespace {

// --- SHA-1 -----------------------------------------------------------------

struct Sha1 {
    uint32_t h[5];
    uint64_t length;
    uint8_t  buffer[64];
    size_t   buffer_len;
};

inline uint32_t rotl(uint32_t x, int n) { return (x << n) | (x >> (32 - n)); }

void sha1Init(Sha1& s) {
    s.h[0] = 0x67452301u;
    s.h[1] = 0xEFCDAB89u;
    s.h[2] = 0x98BADCFEu;
    s.h[3] = 0x10325476u;
    s.h[4] = 0xC3D2E1F0u;
    s.length = 0;
    s.buffer_len = 0;
}

void sha1ProcessBlock(Sha1& s, const uint8_t* block) {
    uint32_t w[80];
    for (int i = 0; i < 16; ++i) {
        w[i] = (uint32_t(block[4 * i]) << 24) | (uint32_t(block[4 * i + 1]) << 16) |
               (uint32_t(block[4 * i + 2]) << 8) | uint32_t(block[4 * i + 3]);
    }
    for (int i = 16; i < 80; ++i) {
        w[i] = rotl(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }
    uint32_t a = s.h[0], b = s.h[1], c = s.h[2], d = s.h[3], e = s.h[4];
    for (int i = 0; i < 80; ++i) {
        uint32_t f, k;
        if (i < 20) {
            f = (b & c) | ((~b) & d);
            k = 0x5A827999u;
        } else if (i < 40) {
            f = b ^ c ^ d;
            k = 0x6ED9EBA1u;
        } else if (i < 60) {
            f = (b & c) | (b & d) | (c & d);
            k = 0x8F1BBCDCu;
        } else {
            f = b ^ c ^ d;
            k = 0xCA62C1D6u;
        }
        uint32_t tmp = rotl(a, 5) + f + e + k + w[i];
        e = d;
        d = c;
        c = rotl(b, 30);
        b = a;
        a = tmp;
    }
    s.h[0] += a;
    s.h[1] += b;
    s.h[2] += c;
    s.h[3] += d;
    s.h[4] += e;
}

void sha1Update(Sha1& s, const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    s.length += len;
    while (len > 0) {
        size_t take = std::min<size_t>(64 - s.buffer_len, len);
        std::memcpy(s.buffer + s.buffer_len, p, take);
        s.buffer_len += take;
        p += take;
        len -= take;
        if (s.buffer_len == 64) {
            sha1ProcessBlock(s, s.buffer);
            s.buffer_len = 0;
        }
    }
}

void sha1Final(Sha1& s, uint8_t out[20]) {
    uint64_t bitlen = s.length * 8;
    uint8_t pad = 0x80;
    sha1Update(s, &pad, 1);
    uint8_t zero = 0;
    while (s.buffer_len != 56) sha1Update(s, &zero, 1);
    uint8_t lenbuf[8];
    for (int i = 0; i < 8; ++i) {
        lenbuf[i] = static_cast<uint8_t>((bitlen >> (56 - 8 * i)) & 0xff);
    }
    sha1Update(s, lenbuf, 8);
    for (int i = 0; i < 5; ++i) {
        out[4 * i]     = static_cast<uint8_t>(s.h[i] >> 24);
        out[4 * i + 1] = static_cast<uint8_t>(s.h[i] >> 16);
        out[4 * i + 2] = static_cast<uint8_t>(s.h[i] >> 8);
        out[4 * i + 3] = static_cast<uint8_t>(s.h[i]);
    }
}

std::string sha1String(const std::string& in) {
    Sha1 s;
    sha1Init(s);
    sha1Update(s, in.data(), in.size());
    uint8_t out[20];
    sha1Final(s, out);
    return std::string(reinterpret_cast<char*>(out), 20);
}

// --- Base64 ----------------------------------------------------------------

const char* kBase64Chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64Encode(const std::string& in) {
    std::string out;
    out.reserve(((in.size() + 2) / 3) * 4);
    size_t i = 0;
    while (i + 3 <= in.size()) {
        uint32_t n = (uint8_t(in[i]) << 16) | (uint8_t(in[i + 1]) << 8) | uint8_t(in[i + 2]);
        out.push_back(kBase64Chars[(n >> 18) & 0x3f]);
        out.push_back(kBase64Chars[(n >> 12) & 0x3f]);
        out.push_back(kBase64Chars[(n >> 6) & 0x3f]);
        out.push_back(kBase64Chars[n & 0x3f]);
        i += 3;
    }
    size_t rem = in.size() - i;
    if (rem == 1) {
        uint32_t n = uint8_t(in[i]) << 16;
        out.push_back(kBase64Chars[(n >> 18) & 0x3f]);
        out.push_back(kBase64Chars[(n >> 12) & 0x3f]);
        out.push_back('=');
        out.push_back('=');
    } else if (rem == 2) {
        uint32_t n = (uint8_t(in[i]) << 16) | (uint8_t(in[i + 1]) << 8);
        out.push_back(kBase64Chars[(n >> 18) & 0x3f]);
        out.push_back(kBase64Chars[(n >> 12) & 0x3f]);
        out.push_back(kBase64Chars[(n >> 6) & 0x3f]);
        out.push_back('=');
    }
    return out;
}

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
        // only watch for close frames.
        std::vector<uint8_t> payload(plen);
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
    std::lock_guard<std::mutex> lk(workers_mu_);
    for (auto& t : workers_) {
        if (t.joinable()) t.detach();
    }
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
}

void HttpServer::acceptLoop() {
    while (!stopping_.load()) {
        sockaddr_in peer{};
        socklen_t peer_len = sizeof(peer);
        int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&peer), &peer_len);
        if (fd < 0) {
            if (stopping_.load()) break;
            if (errno == EINTR) continue;
            continue;
        }
        // One thread per connection — hwmon's endpoints are low-volume and
        // the original uWebSockets event loop also handles them serialized
        // per connection. Detached so we don't have to join.
        std::thread([this, fd]() { handleConnection(fd); }).detach();
    }
}

void HttpServer::handleConnection(int fd) {
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

    // If there's a body (POST with Content-Length), pull it in.
    auto clen_it = req.headers.find("content-length");
    if (clen_it != req.headers.end()) {
        size_t clen = static_cast<size_t>(std::stoul(clen_it->second));
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

    std::ostringstream out;
    out << "HTTP/1.1 " << resp.status << " " << status_text << "\r\n"
        << "Content-Type: " << resp.content_type << "\r\n"
        << "Content-Length: " << resp.body.size() << "\r\n"
        << "Access-Control-Allow-Origin: *\r\n"
        << "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
        << "Access-Control-Allow-Headers: Content-Type\r\n"
        << "Connection: close\r\n";
    for (auto& kv : resp.extra_headers) {
        out << kv.first << ": " << kv.second << "\r\n";
    }
    out << "\r\n" << resp.body;
    std::string out_str = out.str();
    writeAll(fd, out_str.data(), out_str.size());
    ::close(fd);
}

}  // namespace limelight::hwmon
