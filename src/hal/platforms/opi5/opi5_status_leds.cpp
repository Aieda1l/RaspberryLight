// Orange Pi 5 status LED backend: three GPIO-driven indicator LEDs
// (power, heartbeat, link) handled by a single shared worker thread.
//
// The worker wakes every 10 ms, reads each LED's atomic StatusState,
// and drives the line high/low based on a monotonic tick counter so
// BLINK_SLOW / BLINK_FAST phases are consistent across LEDs.
//
// Per-line gpiochip/line numbers are overridable via env vars so the
// hat layout can change without recompilation.  Missing lines are
// logged as warnings but do not fail init() -- on a dev board without
// the hat we simply run with no status indicators.

#include "opi5_status_leds.h"

#include <gpiod.h>
#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

namespace limelight::hal {

namespace {

constexpr const char* kConsumer = "ll_status";

struct LineCfg {
    const char* chip_env;
    const char* line_env;
    const char* default_chip;
    int         default_line;
};

// Defaults chosen to match the hat pin contract in the design spec.
constexpr LineCfg kLines[static_cast<int>(StatusLed::COUNT)] = {
    { "LL_STATUS_PWR_CHIP",  "LL_STATUS_PWR_LINE",  "/dev/gpiochip4", 0 },
    { "LL_STATUS_HB_CHIP",   "LL_STATUS_HB_LINE",   "/dev/gpiochip4", 1 },
    { "LL_STATUS_LINK_CHIP", "LL_STATUS_LINK_LINE", "/dev/gpiochip4", 2 },
};

std::string envOr(const char* name, const char* fallback) {
    const char* v = std::getenv(name);
    return (v && *v) ? std::string(v) : std::string(fallback);
}

int envIntOr(const char* name, int fallback) {
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;
    try { return std::stoi(v); } catch (...) { return fallback; }
}

}  // namespace

struct StatusLedsState {
    gpiod_chip*                      chips[static_cast<int>(StatusLed::COUNT)] = {};
    gpiod_line*                      lines[static_cast<int>(StatusLed::COUNT)] = {};
    std::atomic<StatusState>         states[static_cast<int>(StatusLed::COUNT)] = {};

    std::atomic<bool>                run{false};
    std::thread                      thread;
};

static StatusLedsState& st(void* p) {
    return *static_cast<StatusLedsState*>(p);
}

Opi5StatusLeds::~Opi5StatusLeds() {
    if (!impl_) return;
    auto& s = st(impl_);
    s.run.store(false);
    if (s.thread.joinable()) s.thread.join();
    for (int i = 0; i < static_cast<int>(StatusLed::COUNT); ++i) {
        if (s.lines[i]) {
            gpiod_line_set_value(s.lines[i], 0);
            gpiod_line_release(s.lines[i]);
        }
    }
    for (int i = 0; i < static_cast<int>(StatusLed::COUNT); ++i) {
        if (s.chips[i]) {
            // Only close each unique chip once (multiple LEDs may share one).
            bool already = false;
            for (int j = 0; j < i; ++j) if (s.chips[j] == s.chips[i]) already = true;
            if (!already) gpiod_chip_close(s.chips[i]);
        }
    }
    delete static_cast<StatusLedsState*>(impl_);
    impl_ = nullptr;
}

bool Opi5StatusLeds::init() {
    if (!impl_) impl_ = new StatusLedsState();
    auto& s = st(impl_);

    int acquired = 0;
    for (int i = 0; i < static_cast<int>(StatusLed::COUNT); ++i) {
        const auto&       cfg  = kLines[i];
        const std::string chip = envOr(cfg.chip_env, cfg.default_chip);
        const int         line = envIntOr(cfg.line_env, cfg.default_line);

        // Reuse chip handle if a previous LED already opened the same path.
        gpiod_chip* c = nullptr;
        for (int j = 0; j < i; ++j) {
            if (s.chips[j]) {
                const char* p = gpiod_chip_name(s.chips[j]);
                if (p && chip.find(p) != std::string::npos) { c = s.chips[j]; break; }
            }
        }
        if (!c) c = gpiod_chip_open(chip.c_str());
        if (!c) {
            spdlog::warn("Opi5StatusLeds: unable to open {} for LED {}: {}",
                         chip, i, std::strerror(errno));
            continue;
        }
        s.chips[i] = c;

        gpiod_line* l = gpiod_chip_get_line(c, line);
        if (!l) {
            spdlog::warn("Opi5StatusLeds: chip {} missing line {}", chip, line);
            continue;
        }
        if (gpiod_line_request_output(l, kConsumer, 0) < 0) {
            spdlog::warn("Opi5StatusLeds: cannot request line {}:{} as output: {}",
                         chip, line, std::strerror(errno));
            continue;
        }
        s.lines[i]  = l;
        s.states[i] = StatusState::OFF;
        ++acquired;
    }

    if (acquired == 0) {
        spdlog::warn("Opi5StatusLeds: no status LEDs available (dev board?)");
        return false;
    }

    s.run.store(true);
    s.thread = std::thread([&s]() {
        using namespace std::chrono;
        uint64_t tick = 0;
        while (s.run.load()) {
            for (int i = 0; i < static_cast<int>(StatusLed::COUNT); ++i) {
                if (!s.lines[i]) continue;
                int v = 0;
                switch (s.states[i].load(std::memory_order_relaxed)) {
                    case StatusState::OFF:        v = 0; break;
                    case StatusState::ON:         v = 1; break;
                    case StatusState::BLINK_SLOW: v = (tick % 100) < 50 ? 1 : 0; break;  // 500/500 ms
                    case StatusState::BLINK_FAST: v = (tick %  20) < 10 ? 1 : 0; break;  // 100/100 ms
                }
                gpiod_line_set_value(s.lines[i], v);
            }
            std::this_thread::sleep_for(milliseconds(10));
            ++tick;
        }
    });

    spdlog::info("Opi5StatusLeds: {} LED(s) active", acquired);
    return true;
}

void Opi5StatusLeds::set(StatusLed which, StatusState state) {
    if (!impl_) return;
    auto& s = st(impl_);
    const int i = static_cast<int>(which);
    if (i < 0 || i >= static_cast<int>(StatusLed::COUNT)) return;
    s.states[i].store(state, std::memory_order_relaxed);
}

}  // namespace limelight::hal
