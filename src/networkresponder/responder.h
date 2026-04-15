#pragma once

// Limelight Network Responder
//
// UDP discovery daemon recreated 1:1 from Ghidra decompilation of the
// original `networkresponder` binary (aarch64 ELF, ~22 KB stripped).
//
// Protocol:
//   - V1 ("single IP") UDP port 5809 — listens for "LLPhoneHome" packet,
//     replies with "h:<hostname>:ip:<first_interface_ip>".
//   - V2 ("all interfaces") UDP port 5808 — listens for "LLPhoneHomeV2"
//     packet, replies with
//     "v2:h:<hostname>:interfaces:<n>=<ip>/<mask>[/<bcast>];...".
//
// Interface enumeration is refreshed on every request so DHCP changes
// propagate immediately. The original re-reads the link state through
// `getifaddrs(3)` and filters out interfaces that are loopback or down.

#include <cstdint>
#include <string>
#include <vector>

namespace limelight {

// Mirrors the 136-byte Interface struct observed in the original binary.
// (Four std::string members + one uint32 flags field + padding.)
struct NetInterface {
    std::string name;
    std::string ip;
    std::string netmask;
    std::string broadcast;
    std::uint32_t flags = 0;
};

class NetworkResponder {
public:
    // Matches the original listen ports.
    static constexpr std::uint16_t kV1Port = 5809;  // 0x16b1
    static constexpr std::uint16_t kV2Port = 5808;  // 0x16b0

    NetworkResponder();
    ~NetworkResponder();

    // No copies — owns file descriptors.
    NetworkResponder(const NetworkResponder&) = delete;
    NetworkResponder& operator=(const NetworkResponder&) = delete;

    // Populate hostname + interface list, then create both listen sockets.
    // Returns true when at least the V2 socket (required) is open.
    bool init();

    // Blocking main loop: select() on both sockets, handle discovery requests
    // until `shouldStop()` is set by a signal handler. Returns when the loop
    // is told to stop or on fatal error.
    void run();

    // Asynchronously request the main loop to exit (signal-safe).
    static void requestStop();

    void shutdown();

private:
    // Refreshes hostname_ and interfaces_ from the kernel. Returns true iff
    // any usable AF_INET, non-loopback, IFF_UP interface was found.
    bool populateInterfaces();

    // Create a UDP socket, enable SO_REUSEADDR, and bind to INADDR_ANY:port.
    // On failure, closes the socket and leaves *fd == -1. Returns true on
    // success, matching the original FUN_001021c0 contract.
    bool createBoundSocket(int* fd, std::uint16_t port);

    void handleV1Request(const struct sockaddr_in& from);
    void handleV2Request(const struct sockaddr_in& from);

    std::string hostname_;
    std::vector<NetInterface> interfaces_;
    int v1_fd_ = -1;
    int v2_fd_ = -1;
};

}  // namespace limelight
