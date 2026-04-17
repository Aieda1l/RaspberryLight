#pragma once

// SHA-1 + Base64 helpers used by the WebSocket handshake (RFC 6455). Both the
// hwmon and visionserver WebSocket endpoints need the same `accept` key
// derivation, so the primitives live here in limelight-common to keep the two
// daemons byte-for-byte identical on the wire.

#include <string>

namespace limelight {

// SHA-1 of `in`, returned as a 20-byte raw digest string.
std::string sha1String(const std::string& in);

// Standard Base64 (not URL-safe). Pads output with '=' to a 4-char multiple.
std::string base64Encode(const std::string& in);

}  // namespace limelight
