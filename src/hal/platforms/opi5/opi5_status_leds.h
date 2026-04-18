#pragma once

#include "hal/status_leds.h"

namespace limelight::hal {

class Opi5StatusLeds : public StatusLeds {
public:
    Opi5StatusLeds() = default;
    ~Opi5StatusLeds() override;

    bool init() override;
    void set(StatusLed which, StatusState state) override;

private:
    void* impl_ = nullptr;
};

} // namespace limelight::hal
