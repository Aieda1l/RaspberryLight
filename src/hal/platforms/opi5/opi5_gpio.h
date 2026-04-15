#pragma once

#include "hal/gpio.h"

namespace limelight::hal {

// RK3588 GPIO via libgpiod.  State is held in an opaque pimpl
// (Opi5GpioState) so that this header does not leak <gpiod.h>.
class Opi5Gpio : public Gpio {
public:
    Opi5Gpio() = default;
    ~Opi5Gpio() override;

    bool init() override;
    void setLed(LedColor color, int brightness_pct = 100) override;
    void setLedBlink(LedColor color, int on_ms, int off_ms) override;
    void setLedOff() override;
    bool readPin(int pin) override;
    void writePin(int pin, bool value) override;

    // The .cpp file accesses this via a friend cast; it points to the
    // internal Opi5GpioState struct defined in opi5_gpio.cpp.
    void* opaque_ = nullptr;
};

}  // namespace limelight::hal
