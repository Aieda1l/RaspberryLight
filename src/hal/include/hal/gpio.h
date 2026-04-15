#pragma once

#include <memory>
#include <string>
#include <cstdint>

namespace limelight::hal {

enum class LedColor : uint8_t {
    OFF = 0,
    GREEN,
    RED,
    BLUE,
    WHITE,
    YELLOW,
    PURPLE,
    ORANGE
};

class Gpio {
public:
    virtual ~Gpio() = default;

    virtual bool init() = 0;

    // LED control
    virtual void setLed(LedColor color, int brightness_pct = 100) = 0;
    virtual void setLedBlink(LedColor color, int on_ms, int off_ms) = 0;
    virtual void setLedOff() = 0;

    // Generic GPIO
    virtual bool readPin(int pin) = 0;
    virtual void writePin(int pin, bool value) = 0;

    static std::unique_ptr<Gpio> create();
};

} // namespace limelight::hal
