#pragma once

// Minimal HTTP/1.1 server for the Limelight visionserver.
//
// The original visionserver binary registers ~41 REST routes in a single
// contiguous .rodata region (0x003191b0..0x003194e0 — see
// .ghidra_work/notes/visionserver_rest_endpoints.md). The HTTP implementation
// itself is not a big external dependency; recovered strings look consistent
// with a hand-rolled `std::thread`-per-connection HTTP/1.0 server (same
// pattern as the MJPEG stream server in FUN_0019e9e0).
//
// This mirror is a minimal HTTP/1.1 server with:
//   - method + path routing with `:param` captures and `/prefix/*` wildcards
//   - OPTIONS-preflight CORS handling
//   - binary request/response bodies (for pipeline/NN/field-map uploads)
//
// It intentionally shares no headers with src/hwmon/http_server.h: the two
// daemons are independent binaries and hwmon's HttpServer is already in a
// different namespace (`limelight::hwmon`). Keeping them separate avoids a
// cross-target refactor and mirrors the original firmware, which also links
// two different HTTP implementations into the two daemons.

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace limelight {

struct HttpRequest {
    std::string method;  // GET, POST, PATCH, DELETE, OPTIONS, ...
    std::string path;    // Path portion, no query.
    std::string query;   // Query string, without leading '?'.
    std::string body;    // Raw request body bytes (may be binary).
    std::unordered_map<std::string, std::string> headers;
    // Populated by the router for `:param` captures. For `/prefix/*`
    // wildcards the remainder is placed under the key `"*"`.
    std::unordered_map<std::string, std::string> params;
};

struct HttpResponse {
    int status = 200;
    std::string content_type = "application/json";
    std::string body;  // Raw body bytes (may be binary — e.g. tar/zip).
    std::unordered_map<std::string, std::string> extra_headers;
};

using HttpHandler = std::function<HttpResponse(const HttpRequest&)>;

// Hand-rolled HTTP server used by the REST API. One accept thread and one
// detached worker thread per connection. Connections are handled in the same
// one-shot close-after-response style as the original binary — the web UI
// never pipelines requests.
class HttpServer {
public:
    HttpServer();
    ~HttpServer();

    HttpServer(const HttpServer&) = delete;
    HttpServer& operator=(const HttpServer&) = delete;

    // Register a route. Path may contain `:name` captures and a trailing
    // `/*` wildcard (e.g. `"/recording/*"`).
    void route(const std::string& method, const std::string& path, HttpHandler h);

    // Convenience wrappers for the common verbs used by the web UI.
    void get(const std::string& path, HttpHandler h)     { route("GET",    path, std::move(h)); }
    void post(const std::string& path, HttpHandler h)    { route("POST",   path, std::move(h)); }
    void patch(const std::string& path, HttpHandler h)   { route("PATCH",  path, std::move(h)); }
    void del(const std::string& path, HttpHandler h)     { route("DELETE", path, std::move(h)); }
    void options(const std::string& path, HttpHandler h) { route("OPTIONS",path, std::move(h)); }

    // Bind to port and start the accept thread. Returns false on failure.
    bool start(uint16_t port);

    // Signal the accept loop to exit, close the listening socket, and detach
    // any in-flight worker threads.
    void stop();

    uint16_t port() const { return port_; }

private:
    struct Route {
        std::string method;
        std::string pattern;
        std::vector<std::string> parts;
        bool wildcard = false;
        HttpHandler handler;
    };

    void acceptLoop();
    void handleConnection(int fd);
    bool matchRoute(const std::string& method, const std::string& path,
                    Route** out_route, HttpRequest& req);

    uint16_t port_ = 0;
    int listen_fd_ = -1;
    std::atomic<bool> stopping_{false};
    std::thread accept_thread_;
    std::vector<Route> routes_;
};

}  // namespace limelight
