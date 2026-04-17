// Limelight Network Responder — 1:1 recreation of the original binary
// (`usr/local/bin/visionserver/networkresponder` in the Limelight 4 firmware).
//
// Reverse-engineered from the Ghidra decompilation of the stripped aarch64
// ELF. Every observable side effect — log lines, response byte format,
// port numbers, filter rules — was copied from the decompiled C to keep
// the Orange Pi 5 build behaviorally indistinguishable from the original.

#include "responder.h"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace limelight {

namespace {

// Wire-format discovery magic strings. The decompiled binary compared these
// using fixed-width loads (uint64 + uint32 for V1, uint64 + uint32 + uint16
// for V2) against constants whose high bytes were zero — which is equivalent
// to an exact memcmp against a NUL-terminated string of the same total
// length. We replicate that using the full-length (including trailing NUL)
// memcmp so only well-formed requests are accepted.
constexpr const char kV1Magic[] = "LLPhoneHome";      // 11 chars
constexpr const char kV2Magic[] = "LLPhoneHomeV2";    // 13 chars
constexpr std::size_t kV1CompareLen = sizeof(kV1Magic);  // 12 (incl. NUL)
constexpr std::size_t kV2CompareLen = sizeof(kV2Magic);  // 14 (incl. NUL)

// Request buffer size from the decompiled recvfrom() call (0x7ff).
constexpr std::size_t kRequestBufSize = 0x7ff;

// Global stop flag flipped by signal handlers via requestStop().
std::atomic<bool> g_stop_requested{false};

// Helpers for printing an inbound sender address in "<ip>:<port>" form,
// matching the original "V1 request from ..." / "V2 request from ..." logs.
void printPeer(std::ostream& out, const sockaddr_in& from) {
    const char* ip = ::inet_ntoa(from.sin_addr);
    if (ip) {
        out << ip;
    }
    out << ':' << ntohs(from.sin_port);
}

}  // namespace

NetworkResponder::NetworkResponder() = default;

NetworkResponder::~NetworkResponder() {
    shutdown();
}

void NetworkResponder::requestStop() {
    g_stop_requested.store(true, std::memory_order_relaxed);
}

bool NetworkResponder::populateInterfaces() {
    // The original uses a fixed 256-byte hostname buffer.
    char host_buf[256] = {0};
    if (::gethostname(host_buf, sizeof(host_buf) - 1) < 0) {
        std::cerr << "Failed to get hostname" << std::endl;
        return false;
    }
    hostname_.assign(host_buf);

    interfaces_.clear();

    ifaddrs* head = nullptr;
    if (::getifaddrs(&head) < 0) {
        std::cerr << "Failed to get interfaces" << std::endl;
        return false;
    }

    for (ifaddrs* ifa = head; ifa != nullptr; ifa = ifa->ifa_next) {
        // Filter rules, from the decompiled FUN_00102c80:
        //   - ifa_addr != NULL
        //   - (ifa_flags & IFF_LOOPBACK) == 0
        //   - (ifa_flags & IFF_UP)       != 0
        //   - ifa_addr->sa_family == AF_INET
        if (ifa->ifa_addr == nullptr) continue;
        if ((ifa->ifa_flags & IFF_LOOPBACK) != 0) continue;
        if ((ifa->ifa_flags & IFF_UP) == 0) continue;
        if (ifa->ifa_addr->sa_family != AF_INET) continue;

        NetInterface iface;
        iface.name = ifa->ifa_name ? ifa->ifa_name : "";
        iface.flags = static_cast<std::uint32_t>(ifa->ifa_flags);

        {
            const auto* sa = reinterpret_cast<const sockaddr_in*>(ifa->ifa_addr);
            char buf[INET_ADDRSTRLEN] = {0};
            ::inet_ntop(AF_INET, &sa->sin_addr, buf, sizeof(buf));
            iface.ip = buf;
        }

        if (ifa->ifa_netmask != nullptr) {
            const auto* sa = reinterpret_cast<const sockaddr_in*>(ifa->ifa_netmask);
            char buf[INET_ADDRSTRLEN] = {0};
            ::inet_ntop(AF_INET, &sa->sin_addr, buf, sizeof(buf));
            iface.netmask = buf;
        }

        if ((ifa->ifa_flags & IFF_BROADCAST) != 0 && ifa->ifa_broadaddr != nullptr) {
            const auto* sa = reinterpret_cast<const sockaddr_in*>(ifa->ifa_broadaddr);
            char buf[INET_ADDRSTRLEN] = {0};
            ::inet_ntop(AF_INET, &sa->sin_addr, buf, sizeof(buf));
            iface.broadcast = buf;
        }

        // Discovery log line — format matches original "Found interface: ..."
        std::cout << "Found interface: " << iface.name
                  << " IP: " << iface.ip
                  << " Netmask: " << iface.netmask
                  << " Broadcast: " << iface.broadcast << std::endl;

        interfaces_.push_back(std::move(iface));
    }

    ::freeifaddrs(head);
    return !interfaces_.empty();
}

bool NetworkResponder::createBoundSocket(int* fd, std::uint16_t port) {
    // socket(AF_INET, SOCK_DGRAM, 0) — matches socket(2, 2, 0) in the original.
    *fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (*fd < 0) {
        std::cerr << "Failed to create socket for port " << port << std::endl;
        return false;
    }

    int reuse = 1;
    if (::setsockopt(*fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        // Original only warns on SO_REUSEADDR failure; bind is still attempted.
        std::cerr << "Failed to set SO_REUSEADDR" << std::endl;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (::bind(*fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "Failed to bind socket on port " << port << std::endl;
        ::close(*fd);
        *fd = -1;
        return false;
    }

    std::cout << "Listening on port " << port << std::endl;
    return true;
}

bool NetworkResponder::init() {
    if (!populateInterfaces()) {
        std::cerr << "No network interfaces found" << std::endl;
        return false;
    }

    // Original order: V1 socket first (warn-only on failure), then V2 socket
    // (required). Both may share state via INADDR_ANY with SO_REUSEADDR.
    if (!createBoundSocket(&v1_fd_, kV1Port)) {
        std::cerr << "Warning: Could not create v1 socket" << std::endl;
        // Fall through — V2 is still attempted.
    }

    if (!createBoundSocket(&v2_fd_, kV2Port)) {
        std::cerr << "Failed to create v2 socket" << std::endl;
        return false;
    }

    return true;
}

void NetworkResponder::handleV1Request(const sockaddr_in& from) {
    std::cout << "V1 request from ";
    printPeer(std::cout, from);
    std::cout << std::endl;

    // Refresh interfaces so the reported IP reflects the current link state.
    populateInterfaces();
    if (interfaces_.empty()) {
        return;
    }

    std::ostringstream oss;
    oss << "h:" << hostname_ << ":ip:" << interfaces_.front().ip;
    const std::string payload = oss.str();

    if (v1_fd_ >= 0 && !payload.empty()) {
        ::sendto(v1_fd_, payload.data(), payload.size(), 0,
                 reinterpret_cast<const sockaddr*>(&from), sizeof(from));
    }
}

void NetworkResponder::handleV2Request(const sockaddr_in& from) {
    std::cout << "V2 request from ";
    printPeer(std::cout, from);
    std::cout << std::endl;

    populateInterfaces();

    // Response format from decompilation:
    //   v2:h:<hostname>:interfaces:<name>=<ip>/<mask>[/<bcast>];...
    // The trailing broadcast + slash is only emitted when broadcast is set.
    std::ostringstream oss;
    oss << "v2:h:" << hostname_ << ":interfaces:";
    for (const auto& iface : interfaces_) {
        oss << iface.name << '=' << iface.ip << '/' << iface.netmask;
        if (!iface.broadcast.empty()) {
            oss << '/' << iface.broadcast;
        }
        oss << ';';
    }
    const std::string payload = oss.str();

    if (v2_fd_ >= 0) {
        ::sendto(v2_fd_, payload.data(), payload.size(), 0,
                 reinterpret_cast<const sockaddr*>(&from), sizeof(from));
    }
}

void NetworkResponder::run() {
    std::cout << "Discovery server running..." << std::endl;
    std::cout << "V1 API (single IP) on port " << kV1Port << std::endl;
    std::cout << "V2 API (all interfaces) on port " << kV2Port << std::endl;

    while (!g_stop_requested.load(std::memory_order_relaxed)) {
        fd_set rfds;
        FD_ZERO(&rfds);
        int max_fd = -1;
        if (v1_fd_ >= 0) {
            FD_SET(v1_fd_, &rfds);
            if (v1_fd_ > max_fd) max_fd = v1_fd_;
        }
        if (v2_fd_ >= 0) {
            FD_SET(v2_fd_, &rfds);
            if (v2_fd_ > max_fd) max_fd = v2_fd_;
        }
        if (max_fd < 0) return;

        // 1-second timeout so we can observe the stop flag even when there's
        // no traffic. The original uses an untimed select, but a bounded
        // wait is strictly nicer and shutdown-safe.
        timeval tv{};
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int ready = ::select(max_fd + 1, &rfds, nullptr, nullptr, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            std::cerr << "Select error" << std::endl;
            continue;
        }
        if (ready == 0) continue;

        // Recv path: a single buffer shared between V1/V2, matching the
        // stack layout of the original. The magic string comparisons use the
        // exact byte values observed in the decompiled code.
        char buf[kRequestBufSize + 1];
        sockaddr_in from{};
        socklen_t from_len = sizeof(from);

        // The original binary writes a trailing NUL at buf[n], then does
        // fixed-width word compares that require the byte at offset
        // kV*CompareLen-1 to be zero. We do the same: cap n to the buffer,
        // NUL-terminate once per packet, and then do pure memcmps below.
        auto matches = [](const char* payload, std::size_t n,
                          const char* magic, std::size_t compare_len) {
            const std::size_t magic_len = compare_len - 1;
            if (n < magic_len) return false;
            return std::memcmp(payload, magic, compare_len) == 0;
        };

        if (v1_fd_ >= 0 && FD_ISSET(v1_fd_, &rfds)) {
            from_len = sizeof(from);
            ssize_t n = ::recvfrom(v1_fd_, buf, kRequestBufSize, 0,
                                   reinterpret_cast<sockaddr*>(&from), &from_len);
            if (n > 0) {
                const auto nz = static_cast<std::size_t>(
                    std::min<ssize_t>(n, kRequestBufSize));
                buf[nz] = '\0';
                if (matches(buf, nz, kV1Magic, kV1CompareLen)) {
                    handleV1Request(from);
                }
            }
        }

        if (v2_fd_ >= 0 && FD_ISSET(v2_fd_, &rfds)) {
            from_len = sizeof(from);
            ssize_t n = ::recvfrom(v2_fd_, buf, kRequestBufSize, 0,
                                   reinterpret_cast<sockaddr*>(&from), &from_len);
            if (n > 0) {
                const auto nz = static_cast<std::size_t>(
                    std::min<ssize_t>(n, kRequestBufSize));
                buf[nz] = '\0';
                // Decompiled FUN_00103460 exits the V2 recv loop when the
                // payload matches either "LLPhoneHome" OR "LLPhoneHomeV2",
                // then unconditionally replies in V2 format — so the V2
                // socket happily accepts a V1 magic packet but always sends
                // back a V2 response.
                if (matches(buf, nz, kV2Magic, kV2CompareLen) ||
                    matches(buf, nz, kV1Magic, kV1CompareLen)) {
                    handleV2Request(from);
                }
            }
        }
    }
}

void NetworkResponder::shutdown() {
    if (v1_fd_ >= 0) {
        ::close(v1_fd_);
        v1_fd_ = -1;
    }
    if (v2_fd_ >= 0) {
        ::close(v2_fd_);
        v2_fd_ = -1;
    }
    interfaces_.clear();
}

}  // namespace limelight
