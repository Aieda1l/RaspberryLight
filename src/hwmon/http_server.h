#pragma once

// Minimal HTTP/1.1 + WebSocket server for the Limelight hwmon daemon.
//
// The original binary uses uWebSockets with zlib for permessage-deflate. We
// don't need permessage-deflate for the recreation — the HTTP REST endpoints
// and a plain-text log-streaming WebSocket are enough to match the observable
// behavior on the wire. This tiny server implements just that.

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace limelight::hwmon {

struct HttpRequest {
    std::string method;           // GET, POST, OPTIONS, ...
    std::string path;             // Path portion, no query.
    std::string query;            // Query string, without leading '?'.
    std::string body;
    std::unordered_map<std::string, std::string> headers;
    // Populated by the router for :param captures.
    std::unordered_map<std::string, std::string> params;
};

struct HttpResponse {
    int status = 200;
    std::string content_type = "application/json";
    std::string body;
    std::unordered_map<std::string, std::string> extra_headers;
};

using HttpHandler = std::function<HttpResponse(const HttpRequest&)>;

// WebSocket session for /logs/* endpoints. The session owns its own reader
// thread that tails the log stream and calls send() with each line. The
// server's event loop only handles accept+handshake+close; once upgraded,
// the session runs independently until the peer closes or send() fails.
class WebSocketSession {
public:
    explicit WebSocketSession(int fd);
    ~WebSocketSession();

    // Send a text frame. Thread-safe.
    bool sendText(const std::string& msg);

    // Wait for the peer to send a close frame or disconnect.
    void awaitClose();

    // Force-close from another thread.
    void shutdown();

    int fd() const { return fd_; }

private:
    int fd_;
    std::mutex send_mu_;
    std::atomic<bool> closed_{false};
};

using WsHandshakeHandler =
    std::function<void(std::shared_ptr<WebSocketSession>, const HttpRequest&)>;

class HttpServer {
public:
    HttpServer();
    ~HttpServer();

    // Register an HTTP route. Path may contain `:name` captures, which are
    // placed into req.params. A prefix wildcard of the form "/foo/*" matches
    // any path beginning with "/foo/".
    void get(const std::string& path, HttpHandler h);
    void post(const std::string& path, HttpHandler h);
    void options(const std::string& path, HttpHandler h);

    // Register a WebSocket route. Matches the same pattern rules.
    void websocket(const std::string& path, WsHandshakeHandler h);

    // Start listening on the given port. Returns false on bind/listen failure.
    bool listen(uint16_t port);

    // Block the caller in the accept loop until stop() is called.
    void run();

    // Signal the accept loop to exit, then close the listening socket.
    void stop();

private:
    struct Route {
        std::string method;   // "GET", "POST", "OPTIONS", or "WS"
        std::string pattern;  // Original pattern string (may contain : or *)
        std::vector<std::string> parts;
        bool wildcard = false;
        HttpHandler http_handler;
        WsHandshakeHandler ws_handler;
    };

    void acceptLoop();
    void handleConnection(int fd);
    bool matchRoute(const std::string& method, const std::string& path,
                    Route** out_route, HttpRequest& req);

    int listen_fd_ = -1;
    std::atomic<bool> stopping_{false};
    std::vector<Route> routes_;

    // Active worker threads are stored in a map keyed by a monotonically
    // increasing id. Each worker, on exit, pushes its own id onto
    // finished_workers_ so the accept loop (and stop()) can reap and join
    // the corresponding entry. This gives us a bounded thread count without
    // detaching — which is what protects us from the use-after-free that
    // the previous destructor risked.
    std::unordered_map<uint64_t, std::thread> workers_;
    std::vector<uint64_t> finished_workers_;
    uint64_t next_worker_id_ = 0;
    std::mutex workers_mu_;
};

}  // namespace limelight::hwmon
