// BNO085 SH-2 I2C driver for the Orange Pi 5 hat.
//
// The BNO08x family speaks Sensor Hub Transport Protocol (SH-2) over
// I2C.  Every transfer is prefixed by a 4-byte SHTP header:
//
//   byte 0: length LSB (includes the 4 header bytes)
//   byte 1: length MSB, bit 15 = continuation
//   byte 2: channel
//   byte 3: sequence number (per-channel, wraps at 256)
//
// Channels used here:
//   0 SHTP command           (advertisement, CRST, etc.)
//   1 Executable             (reset ack, DFU)
//   2 Sensor hub control     (feature enable)
//   3 Input sensor reports   (Game Rotation Vector etc.)
//
// We configure the Game Rotation Vector feature (report 0x08) at 100 Hz
// and read each report in an INT-driven thread.  The chip asserts INT
// low any time it has data available; we block on the line's falling
// edge, read one packet, decode if it's a Game RV report, and push a
// timestamped Orientation into a lock-free ring buffer for the fiducial
// pipeline to interpolate against.

#include "opi5_imu_bno085.h"

#include <fcntl.h>
#include <gpiod.h>
#include <linux/i2c-dev.h>
#include <spdlog/spdlog.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace limelight::hal {

namespace {

// ---- SH-2 constants ------------------------------------------------------

constexpr uint8_t  kChannelCommand     = 0;
constexpr uint8_t  kChannelExecutable  = 1;
constexpr uint8_t  kChannelControl     = 2;
constexpr uint8_t  kChannelReport      = 3;

constexpr uint8_t  kReportIdGameRv     = 0x08;
constexpr uint8_t  kSetFeatureCommand  = 0xFD;

constexpr int      kSampleIntervalUs   = 10000;  // 100 Hz
constexpr size_t   kRingSlots          = 256;
constexpr size_t   kMaxPacketLen       = 1024;

// Q-point scales.  See BNO080 datasheet section 6.5.4 "Game Rotation
// Vector Output Report".  Quaternion components are i16 with Q14
// scaling; accuracy estimate is i16 Q12 (radians).
constexpr double   kQ14Scale           = 1.0 / (1 << 14);
constexpr double   kQ12Scale           = 1.0 / (1 << 12);

// ---- env helpers ---------------------------------------------------------

std::string envOr(const char* name, const char* fallback) {
    const char* v = std::getenv(name);
    return (v && *v) ? std::string(v) : std::string(fallback);
}

int envIntOr(const char* name, int fallback) {
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;
    try { return std::stoi(v); } catch (...) { return fallback; }
}

// ---- time helper ---------------------------------------------------------

uint64_t monotonicNs() {
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull
         + static_cast<uint64_t>(ts.tv_nsec);
}

// ---- quaternion -> yaw/pitch/roll (ZYX Tait-Bryan) -----------------------

struct Euler { double yaw, pitch, roll; };

Euler quatToEuler(double qi, double qj, double qk, double qr) {
    // ZYX intrinsic: yaw about Z, pitch about Y, roll about X.
    const double sinr_cosp = 2.0 * (qr * qi + qj * qk);
    const double cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj);
    const double roll      = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (qr * qj - qk * qi);
    if (sinp >  1.0) sinp =  1.0;
    if (sinp < -1.0) sinp = -1.0;
    const double pitch = std::asin(sinp);

    const double siny_cosp = 2.0 * (qr * qk + qi * qj);
    const double cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk);
    const double yaw       = std::atan2(siny_cosp, cosy_cosp);

    return { yaw, pitch, roll };
}

}  // namespace

// ---- pimpl ---------------------------------------------------------------

struct Bno085State {
    int                              i2c_fd       = -1;
    uint8_t                          i2c_addr     = 0x4A;
    gpiod_chip*                      int_chip     = nullptr;
    gpiod_line*                      int_line     = nullptr;
    gpiod_chip*                      rst_chip     = nullptr;
    gpiod_line*                      rst_line     = nullptr;

    uint8_t                          seq[6]       = {0, 0, 0, 0, 0, 0};

    // Lock-free single-writer / multi-reader ring buffer.  Size is a
    // power of two so head/tail can wrap via bitmask.
    Orientation                      ring[kRingSlots]{};
    std::atomic<uint64_t>            ring_head{0};  // next slot to write

    std::atomic<double>              yaw_tare{0.0};
    std::atomic<bool>                run{false};
    std::thread                      thread;
    mutable std::mutex               latest_mu;
    Orientation                      latest_sample{};
};

static Bno085State& state(void* p) { return *static_cast<Bno085State*>(p); }

// ---- low-level I2C helpers ------------------------------------------------

static bool i2cWrite(int fd, const uint8_t* data, size_t len) {
    ssize_t n = ::write(fd, data, len);
    return n == static_cast<ssize_t>(len);
}

static bool i2cRead(int fd, uint8_t* buf, size_t len) {
    size_t got = 0;
    while (got < len) {
        ssize_t n = ::read(fd, buf + got, len - got);
        if (n <= 0) return false;
        got += static_cast<size_t>(n);
    }
    return true;
}

// Reads one SHTP packet into `buf` (up to kMaxPacketLen).  Returns the
// total packet length (including header), or 0 if there is nothing to
// read, or -1 on I/O error.
static int readShtpPacket(int fd, uint8_t* buf) {
    // First read the 4-byte header to learn the length.
    uint8_t hdr[4];
    if (!i2cRead(fd, hdr, 4)) return -1;
    const uint16_t raw_len = static_cast<uint16_t>(hdr[0]) |
                             (static_cast<uint16_t>(hdr[1] & 0x7F) << 8);
    if (raw_len == 0) return 0;            // no data available
    if (raw_len > kMaxPacketLen) return -1;
    std::memcpy(buf, hdr, 4);
    if (raw_len <= 4) return raw_len;

    // Second transfer reads header + payload in one shot.  The BNO085
    // re-sends the header with each read, so we read the whole thing
    // including a fresh header and drop it.
    std::vector<uint8_t> tmp(raw_len);
    if (!i2cRead(fd, tmp.data(), raw_len)) return -1;
    std::memcpy(buf, tmp.data(), raw_len);
    return raw_len;
}

// Sends an SHTP packet on `channel` with `payload` (not including the
// SHTP header, which this function prepends).  Increments the sequence
// counter for that channel.
static bool sendShtpPacket(Bno085State& s, uint8_t channel,
                           const uint8_t* payload, size_t len) {
    if (len + 4 > kMaxPacketLen) return false;
    std::vector<uint8_t> pkt(len + 4);
    const uint16_t total = static_cast<uint16_t>(len + 4);
    pkt[0] = static_cast<uint8_t>(total & 0xFF);
    pkt[1] = static_cast<uint8_t>((total >> 8) & 0x7F);
    pkt[2] = channel;
    pkt[3] = s.seq[channel]++;
    std::memcpy(pkt.data() + 4, payload, len);
    return i2cWrite(s.i2c_fd, pkt.data(), pkt.size());
}

// ---- feature enable ------------------------------------------------------

static bool enableGameRotationVector(Bno085State& s, int interval_us) {
    uint8_t cmd[17] = {};
    cmd[0]  = kSetFeatureCommand;
    cmd[1]  = kReportIdGameRv;
    cmd[2]  = 0;  // feature flags
    cmd[3]  = 0;  // change sensitivity LSB
    cmd[4]  = 0;  // change sensitivity MSB
    cmd[5]  = static_cast<uint8_t>( interval_us        & 0xFF);
    cmd[6]  = static_cast<uint8_t>((interval_us >>  8) & 0xFF);
    cmd[7]  = static_cast<uint8_t>((interval_us >> 16) & 0xFF);
    cmd[8]  = static_cast<uint8_t>((interval_us >> 24) & 0xFF);
    // bytes 9..12 batch interval (0 = no batching)
    // bytes 13..16 sensor-specific config (0)
    return sendShtpPacket(s, kChannelControl, cmd, sizeof(cmd));
}

// ---- report decode -------------------------------------------------------

static bool decodeGameRv(const uint8_t* payload, size_t len, Orientation& out) {
    // Offsets per BNO080 datasheet 6.5.4.  The first 5 bytes of an input
    // report are the timebase header (report id, sequence, status,
    // delay, reserved) -- but channel 3 packets include a 5-byte prefix
    // before the report body, so we first skip it.  Layout after skip:
    //   [0]    report id (0x08)
    //   [1]    sequence
    //   [2]    status (lower 2 bits = accuracy)
    //   [3]    delay
    //   [4..5] quat i
    //   [6..7] quat j
    //   [8..9] quat k
    //   [10..11] quat real
    //   [12..13] accuracy estimate
    if (len < 14) return false;
    if (payload[0] != kReportIdGameRv) return false;

    auto i16 = [&](size_t off) -> int16_t {
        return static_cast<int16_t>(
            static_cast<uint16_t>(payload[off]) |
            (static_cast<uint16_t>(payload[off + 1]) << 8));
    };

    const double qi = i16(4)  * kQ14Scale;
    const double qj = i16(6)  * kQ14Scale;
    const double qk = i16(8)  * kQ14Scale;
    const double qr = i16(10) * kQ14Scale;
    const double acc_rad = i16(12) * kQ12Scale;

    const Euler e = quatToEuler(qi, qj, qk, qr);
    out.yaw_rad      = e.yaw;
    out.pitch_rad    = e.pitch;
    out.roll_rad     = e.roll;
    out.yaw_rate_rps = 0.0;  // Game RV does not include rate; host-derive if needed
    out.accuracy     = (acc_rad < 0.05) ? 3 : (acc_rad < 0.15) ? 2 : (acc_rad < 0.3) ? 1 : 0;
    out.valid        = true;
    return true;
}

// ---- advertisement drain -------------------------------------------------

// After reset the chip emits an advertisement packet plus an initial
// reset-complete message on the executable channel.  Reading a few
// packets until INT stays high (no more data) clears the queue.
static void drainAdvertisement(Bno085State& s) {
    std::vector<uint8_t> buf(kMaxPacketLen);
    for (int i = 0; i < 16; ++i) {
        const int n = readShtpPacket(s.i2c_fd, buf.data());
        if (n <= 0) break;
        spdlog::debug("Opi5Imu: drained {} bytes, channel={}",
                      n, n > 3 ? buf[2] : -1);
    }
}

// ---- read thread ---------------------------------------------------------

static void readLoop(Bno085State* sp) {
    auto& s = *sp;
    std::vector<uint8_t> buf(kMaxPacketLen);

    while (s.run.load()) {
        // Block until INT asserts (falling edge).  Timeout to let us
        // notice `run=false` promptly.
        timespec timeout{ .tv_sec = 0, .tv_nsec = 200'000'000 };
        const int ev = gpiod_line_event_wait(s.int_line, &timeout);
        if (!s.run.load()) break;
        if (ev <= 0) continue;  // timeout or error; loop

        gpiod_line_event event{};
        if (gpiod_line_event_read(s.int_line, &event) != 0) continue;

        const uint64_t ts = monotonicNs();

        const int n = readShtpPacket(s.i2c_fd, buf.data());
        if (n <= 4) continue;  // empty or error

        const uint8_t channel = buf[2];
        if (channel != kChannelReport) {
            spdlog::debug("Opi5Imu: ignoring channel {} packet ({} bytes)",
                          channel, n);
            continue;
        }

        // Input sensor reports have a 5-byte timebase header before the
        // actual report body.
        if (n < 4 + 5 + 1) continue;
        const uint8_t* body     = buf.data() + 4 + 5;
        const size_t   body_len = static_cast<size_t>(n) - 4 - 5;

        if (body[0] != kReportIdGameRv) continue;

        Orientation sample{};
        if (!decodeGameRv(body, body_len, sample)) continue;
        sample.timestamp_ns = ts;
        sample.yaw_rad -= s.yaw_tare.load(std::memory_order_relaxed);
        // Wrap yaw to -pi..+pi after the tare subtraction.
        while (sample.yaw_rad >  M_PI) sample.yaw_rad -= 2 * M_PI;
        while (sample.yaw_rad < -M_PI) sample.yaw_rad += 2 * M_PI;

        const uint64_t head = s.ring_head.load(std::memory_order_relaxed);
        s.ring[head & (kRingSlots - 1)] = sample;
        s.ring_head.store(head + 1, std::memory_order_release);

        {
            std::lock_guard<std::mutex> lk(s.latest_mu);
            s.latest_sample = sample;
        }
    }
}

// ---- lifecycle -----------------------------------------------------------

bool Opi5Imu::init() {
    if (!impl_) impl_ = new Bno085State();
    auto& s = state(impl_);

    const std::string i2c_dev     = envOr("LL_IMU_I2C",       "/dev/i2c-3");
    const int         i2c_addr    = envIntOr("LL_IMU_ADDR",   0x4A);
    const std::string int_chip_p  = envOr("LL_IMU_INT_CHIP",  "/dev/gpiochip1");
    const int         int_line_n  = envIntOr("LL_IMU_INT_LINE", 27);
    const std::string rst_chip_p  = envOr("LL_IMU_RST_CHIP",  "/dev/gpiochip1");
    const int         rst_line_n  = envIntOr("LL_IMU_RST_LINE", 28);

    s.i2c_fd = ::open(i2c_dev.c_str(), O_RDWR);
    if (s.i2c_fd < 0) {
        spdlog::warn("Opi5Imu: unable to open {}: {}",
                     i2c_dev, std::strerror(errno));
        return false;
    }
    s.i2c_addr = static_cast<uint8_t>(i2c_addr);
    if (::ioctl(s.i2c_fd, I2C_SLAVE, s.i2c_addr) < 0) {
        spdlog::warn("Opi5Imu: I2C_SLAVE ioctl failed for 0x{:x}: {}",
                     i2c_addr, std::strerror(errno));
        ::close(s.i2c_fd); s.i2c_fd = -1;
        return false;
    }

    s.int_chip = gpiod_chip_open(int_chip_p.c_str());
    s.rst_chip = gpiod_chip_open(rst_chip_p.c_str());
    if (!s.int_chip || !s.rst_chip) {
        spdlog::warn("Opi5Imu: unable to open gpio chips for INT/RST");
        shutdown();
        return false;
    }
    s.int_line = gpiod_chip_get_line(s.int_chip, int_line_n);
    s.rst_line = gpiod_chip_get_line(s.rst_chip, rst_line_n);
    if (!s.int_line || !s.rst_line) {
        spdlog::warn("Opi5Imu: unable to get INT/RST lines");
        shutdown();
        return false;
    }
    if (gpiod_line_request_falling_edge_events(s.int_line, "ll_imu_int") < 0) {
        spdlog::warn("Opi5Imu: unable to request INT falling-edge: {}",
                     std::strerror(errno));
        shutdown();
        return false;
    }
    if (gpiod_line_request_output(s.rst_line, "ll_imu_rst", 1) < 0) {
        spdlog::warn("Opi5Imu: unable to request RST as output: {}",
                     std::strerror(errno));
        shutdown();
        return false;
    }

    // Pulse reset.
    gpiod_line_set_value(s.rst_line, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    gpiod_line_set_value(s.rst_line, 1);

    // Wait for the first INT (advertisement arrives ~100 ms after reset).
    {
        timespec timeout{ .tv_sec = 0, .tv_nsec = 500'000'000 };
        if (gpiod_line_event_wait(s.int_line, &timeout) <= 0) {
            spdlog::warn("Opi5Imu: no INT after reset; sensor may be absent");
            shutdown();
            return false;
        }
        gpiod_line_event ev{};
        gpiod_line_event_read(s.int_line, &ev);
    }

    drainAdvertisement(s);

    if (!enableGameRotationVector(s, kSampleIntervalUs)) {
        spdlog::warn("Opi5Imu: feature enable write failed");
        shutdown();
        return false;
    }

    spdlog::info("Opi5Imu: BNO085 on {} @ 0x{:x}, GameRV @ 100 Hz",
                 i2c_dev, i2c_addr);

    s.run.store(true);
    s.thread = std::thread(readLoop, &s);
    return true;
}

void Opi5Imu::shutdown() {
    if (!impl_) return;
    auto& s = state(impl_);

    s.run.store(false);
    if (s.thread.joinable()) s.thread.join();

    if (s.rst_line) {
        gpiod_line_set_value(s.rst_line, 0);
        gpiod_line_release(s.rst_line);
    }
    if (s.int_line) gpiod_line_release(s.int_line);
    if (s.rst_chip) gpiod_chip_close(s.rst_chip);
    if (s.int_chip && s.int_chip != s.rst_chip) gpiod_chip_close(s.int_chip);
    s.int_chip = s.rst_chip = nullptr;
    s.int_line = s.rst_line = nullptr;

    if (s.i2c_fd >= 0) ::close(s.i2c_fd);
    s.i2c_fd = -1;
}

Opi5Imu::~Opi5Imu() {
    if (!impl_) return;
    shutdown();
    delete static_cast<Bno085State*>(impl_);
    impl_ = nullptr;
}

// ---- public API ----------------------------------------------------------

Orientation Opi5Imu::latest() const {
    if (!impl_) return {};
    auto& s = state(impl_);
    std::lock_guard<std::mutex> lk(s.latest_mu);
    return s.latest_sample;
}

Orientation Opi5Imu::sampleAt(uint64_t timestamp_ns) const {
    if (!impl_) return {};
    auto& s = state(impl_);

    const uint64_t head = s.ring_head.load(std::memory_order_acquire);
    if (head == 0) return {};

    const size_t count = std::min<size_t>(head, kRingSlots);

    // Walk newest -> oldest until we find the first sample older than
    // `timestamp_ns`; the one after it is the newer straddling sample.
    Orientation newer{};
    Orientation older{};
    bool have_newer = false;
    for (size_t i = 0; i < count; ++i) {
        const size_t idx = (head - 1 - i) & (kRingSlots - 1);
        const Orientation& cur = s.ring[idx];
        if (!cur.valid) continue;
        if (cur.timestamp_ns >= timestamp_ns) {
            newer      = cur;
            have_newer = true;
        } else {
            older = cur;
            break;
        }
    }

    if (!have_newer) return older.valid ? older : Orientation{};
    if (!older.valid) return newer;
    if (newer.timestamp_ns == older.timestamp_ns) return newer;

    const double t = static_cast<double>(timestamp_ns - older.timestamp_ns) /
                     static_cast<double>(newer.timestamp_ns - older.timestamp_ns);
    Orientation out = newer;
    auto lerp = [t](double a, double b) { return a + t * (b - a); };
    // Shortest-path lerp on yaw to avoid wraparound artifacts.
    double dyaw = newer.yaw_rad - older.yaw_rad;
    if (dyaw >  M_PI) dyaw -= 2 * M_PI;
    if (dyaw < -M_PI) dyaw += 2 * M_PI;
    out.yaw_rad   = older.yaw_rad + t * dyaw;
    out.pitch_rad = lerp(older.pitch_rad, newer.pitch_rad);
    out.roll_rad  = lerp(older.roll_rad,  newer.roll_rad);
    out.timestamp_ns = timestamp_ns;
    return out;
}

void Opi5Imu::tareYaw() {
    if (!impl_) return;
    auto& s = state(impl_);
    const Orientation cur = latest();
    if (!cur.valid) return;
    s.yaw_tare.store(s.yaw_tare.load() + cur.yaw_rad,
                     std::memory_order_relaxed);
}

}  // namespace limelight::hal
