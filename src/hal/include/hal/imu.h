#pragma once

// Orientation / IMU abstraction.
//
// The original Limelight 4 reads a BNO08x sensor over I2C with the SH-2
// protocol.  On the Orange Pi 5 port we expose the same semantics behind
// this interface so the fiducial pipeline can do yaw-constrained PnP
// without caring which IMU is under the hat.
//
// Thread-safety: implementations must make latest() / sampleAt() safe to
// call from the vision thread while the driver's own read thread writes
// new samples.  They are read-only with respect to the ring buffer.

#include <array>
#include <cstdint>
#include <memory>

namespace limelight::hal {

struct Orientation {
    double   yaw_rad      = 0.0;  // -pi .. +pi, right-hand about +Z (up)
    double   pitch_rad    = 0.0;  // -pi/2 .. +pi/2
    double   roll_rad     = 0.0;  // -pi .. +pi
    double   yaw_rate_rps = 0.0;  // positive CCW looking down
    uint64_t timestamp_ns = 0;    // CLOCK_MONOTONIC at sample time
    uint8_t  accuracy     = 0;    // 0 (unreliable) .. 3 (high)
    bool     valid        = false;
};

class Imu {
public:
    virtual ~Imu() = default;

    // Brings up the sensor: open bus, reset, enable the fusion report.
    // Returns false if the sensor is absent or unresponsive; callers are
    // expected to log and continue without an IMU in that case.
    virtual bool init() = 0;

    // Stops the read thread and drives RST low.  Idempotent.
    virtual void shutdown() = 0;

    // Most recent sample.  {valid=false} until the first report arrives.
    virtual Orientation latest() const = 0;

    // Returns the sample closest to `timestamp_ns`, linearly interpolated
    // between the two straddling samples when they exist.  Used to align
    // yaw with frame capture time so a slow vision loop doesn't get
    // "stale" yaw values.
    virtual Orientation sampleAt(uint64_t timestamp_ns) const = 0;

    // Software tare: zeroes the yaw reference without touching the chip.
    // Subsequent latest()/sampleAt() calls return yaw relative to the
    // value that was current at the moment tareYaw() was invoked.
    virtual void tareYaw() = 0;

    static std::unique_ptr<Imu> create();
};

} // namespace limelight::hal
