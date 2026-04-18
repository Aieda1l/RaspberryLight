#pragma once

#include "hal/imu.h"

namespace limelight::hal {

// BNO085 / BNO080 / BNO086 driver over I2C using the SH-2 protocol.
// State (SH-2 sequence numbers, ring buffer, read thread, gpiod lines,
// I2C fd) lives behind an opaque pimpl so the header does not need to
// pull in <linux/i2c-dev.h>, <gpiod.h>, or the SH-2 constants.
class Opi5Imu : public Imu {
public:
    Opi5Imu() = default;
    ~Opi5Imu() override;

    bool        init() override;
    void        shutdown() override;
    Orientation latest() const override;
    Orientation sampleAt(uint64_t timestamp_ns) const override;
    void        tareYaw() override;

private:
    void* impl_ = nullptr;
};

} // namespace limelight::hal
