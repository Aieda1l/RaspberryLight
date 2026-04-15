// Orange Pi 5 GPIO HAL — libgpiod-based LED / GPIO control for RK3588.
//
// The RK3588 exposes five GPIO banks via /dev/gpiochip0..4.  The Limelight 4
// firmware opens /dev/gpiochip0 directly and requests a vector of output
// lines tagged "ll4_led" (seen in visionserver 0x2a1ea0 and 0x2a1c80).  On
// the Raspberry Pi 5 / CM4 these are physical pins driving the LED rail.
// We keep the same consumer tag and a configurable (chip, lines) tuple so
// that an Orange Pi 5 port can map whichever pins are wired to the LED
// board.
//
// Configuration precedence:
//   1. Environment variables  LL_LED_CHIP  (e.g. /dev/gpiochip1)
//                             LL_LED_LINES (e.g. "24,25,26")
//   2. Compile-time defaults  gpiochip4, line 14 (GPIO4_A6 on OPi5 header 7)
//
// LED color is best-effort: on Limelight hardware the LEDs are a single
// green rail, so brightness > 0 == driven high.  Multi-color boards can
// wire separate lines per color by extending the `color_lines_` map.

#include "opi5_gpio.h"

#include <gpiod.h>
#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace limelight::hal {

namespace {

constexpr const char* kConsumer      = "ll4_led";
constexpr const char* kDefaultChip   = "/dev/gpiochip4";
constexpr const char* kDefaultLines  = "14";  // GPIO4_A6 on OPi5 40-pin header

std::vector<unsigned int> parseLines(const std::string& csv) {
    std::vector<unsigned int> out;
    std::stringstream ss(csv);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (tok.empty()) continue;
        try {
            out.push_back(static_cast<unsigned int>(std::stoul(tok)));
        } catch (...) { /* ignore malformed entries */ }
    }
    return out;
}

std::string envOr(const char* name, const char* fallback) {
    const char* v = std::getenv(name);
    return (v && *v) ? std::string(v) : std::string(fallback);
}

}  // namespace

// ---------- internal state (kept out of the header to avoid leaking gpiod) --

struct Opi5GpioState {
    gpiod_chip*                     chip = nullptr;
    std::vector<gpiod_line*>        lines;       // LED rail lines
    std::string                     chip_path;

    std::mutex                      blink_mu;
    std::thread                     blink_thread;
    std::atomic<bool>               blink_run{false};
    std::atomic<int>                blink_on_ms{0};
    std::atomic<int>                blink_off_ms{0};
    std::atomic<int>                blink_level{0};  // 0/1 during off/on phase

    bool                            initialized = false;
};

// The header stores state in an opaque void*, so the .cpp reinterprets
// it back into Opi5GpioState each time.  Keeping libgpiod types out of
// the header means other HAL consumers don't have to pull in <gpiod.h>.

// ---------- lifecycle ------------------------------------------------------

bool Opi5Gpio::init() {
    if (!opaque_) {
        opaque_ = new Opi5GpioState();
    }
    auto& s = *static_cast<Opi5GpioState*>(opaque_);
    if (s.initialized) return true;

    s.chip_path = envOr("LL_LED_CHIP", kDefaultChip);
    const std::string lines_csv = envOr("LL_LED_LINES", kDefaultLines);

    s.chip = gpiod_chip_open(s.chip_path.c_str());
    if (!s.chip) {
        spdlog::warn("Opi5Gpio: unable to open {}: {}", s.chip_path, std::strerror(errno));
        return false;
    }
    spdlog::info("Opi5Gpio: using {} (label: {})",
                 s.chip_path,
                 gpiod_chip_label(s.chip) ? gpiod_chip_label(s.chip) : "?");

    for (auto line_num : parseLines(lines_csv)) {
        gpiod_line* line = gpiod_chip_get_line(s.chip, line_num);
        if (!line) {
            spdlog::warn("Opi5Gpio: unable to get line {}", line_num);
            continue;
        }
        if (gpiod_line_request_output(line, kConsumer, 0) < 0) {
            spdlog::warn("Opi5Gpio: unable to request line {} as output: {}",
                         line_num, std::strerror(errno));
            gpiod_line_release(line);
            continue;
        }
        s.lines.push_back(line);
        spdlog::debug("Opi5Gpio: configured line {} as LED output", line_num);
    }

    if (s.lines.empty()) {
        spdlog::warn("Opi5Gpio: no LED lines configured");
    }
    s.initialized = true;
    return true;
}

Opi5Gpio::~Opi5Gpio() {
    if (!opaque_) return;
    auto* s = static_cast<Opi5GpioState*>(opaque_);
    s->blink_run = false;
    if (s->blink_thread.joinable()) s->blink_thread.join();
    for (auto* line : s->lines) {
        if (line) gpiod_line_release(line);
    }
    s->lines.clear();
    if (s->chip) gpiod_chip_close(s->chip);
    delete s;
    opaque_ = nullptr;
}

// ---------- LED control ----------------------------------------------------

static void writeAll(Opi5GpioState& s, int value) {
    for (auto* line : s.lines) {
        if (line) gpiod_line_set_value(line, value);
    }
}

// Stops any in-progress blink thread. Must be called without holding blink_mu.
static void stopBlink(Opi5GpioState& s) {
    std::lock_guard<std::mutex> lk(s.blink_mu);
    if (s.blink_run.exchange(false)) {
        if (s.blink_thread.joinable()) s.blink_thread.join();
    }
}

void Opi5Gpio::setLed(LedColor color, int brightness_pct) {
    if (!opaque_) return;
    auto& s = *static_cast<Opi5GpioState*>(opaque_);
    stopBlink(s);
    const int on = (color != LedColor::OFF && brightness_pct > 0) ? 1 : 0;
    writeAll(s, on);
}

void Opi5Gpio::setLedBlink(LedColor color, int on_ms, int off_ms) {
    if (!opaque_) return;
    auto& s = *static_cast<Opi5GpioState*>(opaque_);

    stopBlink(s);
    if (color == LedColor::OFF || on_ms <= 0) {
        writeAll(s, 0);
        return;
    }

    std::lock_guard<std::mutex> lk(s.blink_mu);
    s.blink_on_ms  = on_ms;
    s.blink_off_ms = off_ms > 0 ? off_ms : on_ms;
    s.blink_run    = true;
    s.blink_thread = std::thread([&s]() {
        while (s.blink_run.load()) {
            writeAll(s, 1);
            s.blink_level = 1;
            for (int t = 0; t < s.blink_on_ms.load() && s.blink_run.load(); t += 10) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            if (!s.blink_run.load()) break;
            writeAll(s, 0);
            s.blink_level = 0;
            for (int t = 0; t < s.blink_off_ms.load() && s.blink_run.load(); t += 10) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        writeAll(s, 0);
    });
}

void Opi5Gpio::setLedOff() {
    if (!opaque_) return;
    auto& s = *static_cast<Opi5GpioState*>(opaque_);
    stopBlink(s);
    writeAll(s, 0);
}

// ---------- generic pin access (one-shot, outside the LED rail) ------------

bool Opi5Gpio::readPin(int pin) {
    if (!opaque_) return false;
    auto& s = *static_cast<Opi5GpioState*>(opaque_);
    if (!s.chip) return false;
    gpiod_line* line = gpiod_chip_get_line(s.chip, static_cast<unsigned>(pin));
    if (!line) return false;
    if (gpiod_line_request_input(line, kConsumer) < 0) {
        gpiod_line_release(line);
        return false;
    }
    int v = gpiod_line_get_value(line);
    gpiod_line_release(line);
    return v > 0;
}

void Opi5Gpio::writePin(int pin, bool value) {
    if (!opaque_) return;
    auto& s = *static_cast<Opi5GpioState*>(opaque_);
    if (!s.chip) return;
    gpiod_line* line = gpiod_chip_get_line(s.chip, static_cast<unsigned>(pin));
    if (!line) return;
    if (gpiod_line_request_output(line, kConsumer, value ? 1 : 0) < 0) {
        gpiod_line_release(line);
        return;
    }
    gpiod_line_set_value(line, value ? 1 : 0);
    gpiod_line_release(line);
}

}  // namespace limelight::hal
