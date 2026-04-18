// Orange Pi 5 GPIO HAL -- two LED backends behind one interface.
//
// The original Limelight 4 LED rail is a single green string switched by
// one GPIO line (consumer "ll4_led").  We keep that 1:1 path as the
// default, and add an addressable WS2812 / SK6812 strip backend that
// speaks the same Gpio interface so main.cpp and the pipeline config
// don't have to care which rail is wired up.
//
// Mode is selected by env var LL_LED_MODE:
//
//   gpio       -- (default) libgpiod output lines driving a MOSFET gate.
//                 Config: LL_LED_CHIP   (default /dev/gpiochip4)
//                         LL_LED_LINES  (default "14", CSV supported)
//
//   ws2812     -- Addressable strip clocked out through SPI MOSI.  Works
//                 with WS2812B / WS2813 / SK6812 (RGB and RGBW).  Wiring:
//                         SPI MOSI pin  -> strip DIN  (through a 3.3V->5V
//                                          level shifter such as 74HCT245
//                                          if the strip is 5 V powered)
//                         Strip GND    -> common GND with the OPi5
//                         Strip +5 V   -> external 5 V supply sized for
//                                          ~60 mA per LED at full white
//                 Config: LL_LED_SPI    (default /dev/spidev1.0)
//                         LL_LED_SPI_HZ (default 2400000)
//                         LL_LED_COUNT  (default 8)
//                         LL_LED_ORDER  (default "grb"; also rgb/grbw/rgbw)
//
// LedColor values are mapped to RGB triples and scaled by the pipeline's
// brightness_pct before being written.  The existing blink thread works
// for both modes; in ws2812 mode "on" writes the active color, "off"
// writes black and re-flushes so the strip actually latches.

#include "opi5_gpio.h"
#include "ws2812_driver.h"

#include <gpiod.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace limelight::hal {

namespace {

constexpr const char* kConsumer      = "ll4_led";
constexpr const char* kDefaultChip   = "/dev/gpiochip4";
constexpr const char* kDefaultLines  = "14";
constexpr const char* kDefaultSpi    = "/dev/spidev1.0";
constexpr uint32_t    kDefaultSpiHz  = 2'400'000;
constexpr int         kDefaultCount  = 8;

enum class LedMode { GPIOD, WS2812 };

std::string envOr(const char* name, const char* fallback) {
    const char* v = std::getenv(name);
    return (v && *v) ? std::string(v) : std::string(fallback);
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return s;
}

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

Ws2812Order parseOrder(const std::string& s) {
    const std::string k = lower(s);
    if (k == "rgb")  return Ws2812Order::RGB;
    if (k == "grbw") return Ws2812Order::GRBW;
    if (k == "rgbw") return Ws2812Order::RGBW;
    return Ws2812Order::GRB;
}

// Map an enum color + brightness (0-100) to a 24-bit WS2812 triple.  The
// white channel of SK6812 RGBW strips is driven for LedColor::WHITE so the
// rail looks like actual white light rather than mixed R+G+B.
Ws2812Color colorToRgb(LedColor c, int brightness_pct, bool rgbw) {
    if (c == LedColor::OFF || brightness_pct <= 0) return {};
    const int pct = std::clamp(brightness_pct, 0, 100);
    auto scale = [pct](int v) -> uint8_t {
        return static_cast<uint8_t>((v * pct + 50) / 100);
    };

    switch (c) {
        case LedColor::GREEN:  return { scale(0),   scale(255), scale(0),   0 };
        case LedColor::RED:    return { scale(255), scale(0),   scale(0),   0 };
        case LedColor::BLUE:   return { scale(0),   scale(0),   scale(255), 0 };
        case LedColor::YELLOW: return { scale(255), scale(255), scale(0),   0 };
        case LedColor::PURPLE: return { scale(160), scale(0),   scale(255), 0 };
        case LedColor::ORANGE: return { scale(255), scale(90),  scale(0),   0 };
        case LedColor::WHITE:
            return rgbw
                ? Ws2812Color{ 0, 0, 0, scale(255) }
                : Ws2812Color{ scale(255), scale(255), scale(255), 0 };
        case LedColor::OFF:
        default:
            return {};
    }
}

}  // namespace

// ---------- internal state (kept out of the header to avoid leaking gpiod) --

struct Opi5GpioState {
    LedMode                          mode = LedMode::GPIOD;

    // -- GPIOD backend --
    gpiod_chip*                      chip = nullptr;
    std::vector<gpiod_line*>         lines;
    std::string                      chip_path;

    // -- WS2812 backend --
    std::unique_ptr<Ws2812Driver>    ws2812;
    std::mutex                       spi_mu;
    bool                             ws2812_rgbw = false;

    // -- Blink thread (shared) --
    std::mutex                       blink_mu;
    std::thread                      blink_thread;
    std::atomic<bool>                blink_run{false};
    std::atomic<int>                 blink_on_ms{0};
    std::atomic<int>                 blink_off_ms{0};
    std::atomic<int>                 blink_level{0};

    // Current target color/brightness (so the blink thread and re-entrant
    // setLed() calls know what to paint on the "on" phase).
    std::atomic<int>                 active_color_int{static_cast<int>(LedColor::OFF)};
    std::atomic<int>                 active_brightness{0};

    bool                             initialized = false;
};

// ---------- helpers --------------------------------------------------------

static void writeGpiodAll(Opi5GpioState& s, int value) {
    for (auto* line : s.lines) {
        if (line) gpiod_line_set_value(line, value);
    }
}

static void writeWs2812(Opi5GpioState& s, LedColor color, int brightness_pct) {
    if (!s.ws2812 || !s.ws2812->isOpen()) return;
    const Ws2812Color c = colorToRgb(color, brightness_pct, s.ws2812_rgbw);
    std::lock_guard<std::mutex> lk(s.spi_mu);
    s.ws2812->fill(c);
    s.ws2812->flush();
}

// Writes the current "active" state.  In GPIOD mode this is a simple
// high/low based on color+brightness; in WS2812 mode it repaints the strip.
static void writeActive(Opi5GpioState& s, bool on_phase) {
    const auto color = static_cast<LedColor>(s.active_color_int.load());
    const int  pct   = s.active_brightness.load();

    if (s.mode == LedMode::GPIOD) {
        const int val = (on_phase && color != LedColor::OFF && pct > 0) ? 1 : 0;
        writeGpiodAll(s, val);
    } else {
        if (on_phase) {
            writeWs2812(s, color, pct);
        } else {
            writeWs2812(s, LedColor::OFF, 0);
        }
    }
}

// Stops any in-progress blink thread. Must be called without holding blink_mu.
static void stopBlink(Opi5GpioState& s) {
    std::lock_guard<std::mutex> lk(s.blink_mu);
    if (s.blink_run.exchange(false)) {
        if (s.blink_thread.joinable()) s.blink_thread.join();
    }
}

// ---------- lifecycle ------------------------------------------------------

static bool initGpiod(Opi5GpioState& s) {
    s.chip_path              = envOr("LL_LED_CHIP",  kDefaultChip);
    const std::string lines_csv = envOr("LL_LED_LINES", kDefaultLines);

    s.chip = gpiod_chip_open(s.chip_path.c_str());
    if (!s.chip) {
        spdlog::warn("Opi5Gpio: unable to open {}: {}",
                     s.chip_path, std::strerror(errno));
        return false;
    }
    spdlog::info("Opi5Gpio[gpio]: using {} (label: {})",
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
    return true;
}

static bool initWs2812(Opi5GpioState& s) {
    const std::string path  = envOr("LL_LED_SPI",    kDefaultSpi);
    const std::string hz    = envOr("LL_LED_SPI_HZ", "");
    const std::string count = envOr("LL_LED_COUNT",  "");
    const std::string order = envOr("LL_LED_ORDER",  "grb");

    uint32_t speed    = kDefaultSpiHz;
    int      num_leds = kDefaultCount;
    try { if (!hz.empty())    speed    = static_cast<uint32_t>(std::stoul(hz)); } catch (...) {}
    try { if (!count.empty()) num_leds = std::stoi(count); } catch (...) {}

    const Ws2812Order parsed_order = parseOrder(order);
    s.ws2812_rgbw = (parsed_order == Ws2812Order::GRBW ||
                     parsed_order == Ws2812Order::RGBW);
    s.ws2812 = std::make_unique<Ws2812Driver>();
    if (!s.ws2812->open(path, speed, parsed_order, num_leds)) {
        spdlog::warn("Opi5Gpio[ws2812]: strip init failed; LEDs will be inert");
        s.ws2812.reset();
        return false;
    }
    // Start with a clean black frame so the strip isn't showing garbage
    // from whatever was last on the SPI wire.
    std::lock_guard<std::mutex> lk(s.spi_mu);
    s.ws2812->fill({});
    s.ws2812->flush();
    return true;
}

bool Opi5Gpio::init() {
    if (!opaque_) opaque_ = new Opi5GpioState();
    auto& s = *static_cast<Opi5GpioState*>(opaque_);
    if (s.initialized) return true;

    const std::string mode_s = lower(envOr("LL_LED_MODE", "gpio"));
    if (mode_s == "ws2812" || mode_s == "neopixel" || mode_s == "strip") {
        s.mode = LedMode::WS2812;
        initWs2812(s);
    } else {
        s.mode = LedMode::GPIOD;
        initGpiod(s);
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

    // Ws2812Driver destructor paints black and closes the fd.
    s->ws2812.reset();

    delete s;
    opaque_ = nullptr;
}

// ---------- LED control ----------------------------------------------------

void Opi5Gpio::setLed(LedColor color, int brightness_pct) {
    if (!opaque_) return;
    auto& s = *static_cast<Opi5GpioState*>(opaque_);
    stopBlink(s);

    s.active_color_int  = static_cast<int>(color);
    s.active_brightness = brightness_pct;
    writeActive(s, /*on_phase=*/true);
}

void Opi5Gpio::setLedBlink(LedColor color, int on_ms, int off_ms) {
    if (!opaque_) return;
    auto& s = *static_cast<Opi5GpioState*>(opaque_);

    stopBlink(s);
    if (color == LedColor::OFF || on_ms <= 0) {
        s.active_color_int  = static_cast<int>(LedColor::OFF);
        s.active_brightness = 0;
        writeActive(s, /*on_phase=*/false);
        return;
    }

    std::lock_guard<std::mutex> lk(s.blink_mu);
    s.active_color_int  = static_cast<int>(color);
    s.active_brightness = 100;
    s.blink_on_ms       = on_ms;
    s.blink_off_ms      = off_ms > 0 ? off_ms : on_ms;
    s.blink_run         = true;
    s.blink_thread = std::thread([&s]() {
        while (s.blink_run.load()) {
            writeActive(s, true);
            s.blink_level = 1;
            for (int t = 0; t < s.blink_on_ms.load() && s.blink_run.load(); t += 10) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            if (!s.blink_run.load()) break;
            writeActive(s, false);
            s.blink_level = 0;
            for (int t = 0; t < s.blink_off_ms.load() && s.blink_run.load(); t += 10) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        writeActive(s, false);
    });
}

void Opi5Gpio::setLedOff() {
    if (!opaque_) return;
    auto& s = *static_cast<Opi5GpioState*>(opaque_);
    stopBlink(s);
    s.active_color_int  = static_cast<int>(LedColor::OFF);
    s.active_brightness = 0;
    writeActive(s, /*on_phase=*/false);
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
