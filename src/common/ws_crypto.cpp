#include "ws_crypto.h"

#include <algorithm>
#include <cstdint>
#include <cstring>

namespace limelight {

namespace {

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
        w[i] = (uint32_t(block[4 * i]) << 24) |
               (uint32_t(block[4 * i + 1]) << 16) |
               (uint32_t(block[4 * i + 2]) << 8) |
                uint32_t(block[4 * i + 3]);
    }
    for (int i = 16; i < 80; ++i) {
        w[i] = rotl(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }
    uint32_t a = s.h[0], b = s.h[1], c = s.h[2], d = s.h[3], e = s.h[4];
    for (int i = 0; i < 80; ++i) {
        uint32_t f, k;
        if (i < 20)      { f = (b & c) | ((~b) & d);        k = 0x5A827999u; }
        else if (i < 40) { f = b ^ c ^ d;                   k = 0x6ED9EBA1u; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDCu; }
        else             { f = b ^ c ^ d;                   k = 0xCA62C1D6u; }
        uint32_t tmp = rotl(a, 5) + f + e + k + w[i];
        e = d; d = c; c = rotl(b, 30); b = a; a = tmp;
    }
    s.h[0] += a; s.h[1] += b; s.h[2] += c; s.h[3] += d; s.h[4] += e;
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

const char* kBase64Chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

}  // namespace

std::string sha1String(const std::string& in) {
    Sha1 s;
    sha1Init(s);
    sha1Update(s, in.data(), in.size());
    uint8_t out[20];
    sha1Final(s, out);
    return std::string(reinterpret_cast<char*>(out), 20);
}

std::string base64Encode(const std::string& in) {
    std::string out;
    out.reserve(((in.size() + 2) / 3) * 4);
    size_t i = 0;
    while (i + 3 <= in.size()) {
        uint32_t n = (uint8_t(in[i]) << 16) | (uint8_t(in[i + 1]) << 8) |
                      uint8_t(in[i + 2]);
        out.push_back(kBase64Chars[(n >> 18) & 0x3f]);
        out.push_back(kBase64Chars[(n >> 12) & 0x3f]);
        out.push_back(kBase64Chars[(n >> 6)  & 0x3f]);
        out.push_back(kBase64Chars[ n        & 0x3f]);
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
        out.push_back(kBase64Chars[(n >> 6)  & 0x3f]);
        out.push_back('=');
    }
    return out;
}

}  // namespace limelight
