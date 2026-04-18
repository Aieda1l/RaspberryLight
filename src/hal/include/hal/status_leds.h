#pragma once

// Status LEDs: three indicator LEDs on the Orange Pi 5 hat driven by
// GPIO.  Separate from the user-facing WS2812 strip; these are for
// device status (power on, vision loop healthy, NT connected).
//
// Blink cadences are handled by a single shared worker thread inside
// the implementation so callers can stop, start, and re-enter setters
// from anywhere without spawning per-LED threads.

#include <cstdint>
#include <memory>

namespace limelight::hal {

enum class StatusLed : uint8_t {
    POWER = 0,
    HEARTBEAT,
    LINK,
    COUNT   // must be last
};

enum class StatusState : uint8_t {
    OFF = 0,
    ON,
    BLINK_SLOW,   // 1 Hz,  500 ms on / 500 ms off
    BLINK_FAST    // 5 Hz,  100 ms on / 100 ms off
};

class StatusLeds {
public:
    virtual ~StatusLeds() = default;

    // Acquires the 3 GPIO lines.  Returns true if the backend is usable,
    // false if every line failed to acquire (typical on dev boards
    // without the hat).  Callers should treat this as non-fatal.
    virtual bool init() = 0;

    // Thread-safe; takes effect on the next worker tick (<= 20 ms).
    virtual void set(StatusLed which, StatusState state) = 0;

    static std::unique_ptr<StatusLeds> create();
};

} // namespace limelight::hal
