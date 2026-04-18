// WS2812 / SK6812 SPI-based LED strip driver for the Orange Pi 5.
//
// The WS2812B protocol is 800 kHz NRZ: each bit is 1.25 us, encoded by the
// duty cycle of a high pulse.  Bit-banging from Linux userspace can't meet
// the ~150 ns timing tolerance, so we re-encode the waveform onto SPI MOSI
// and let the RK3588 SPI controller clock it out in hardware.  With a SPI
// clock of 2.4 MHz, one SPI bit = 417 ns, so 3 SPI bits = 1.25 us = one
// WS2812 bit:
//
//   WS2812 "0"  = SPI 0b100   (T0H ~417 ns, T0L ~833 ns)
//   WS2812 "1"  = SPI 0b110   (T1H ~833 ns, T1L ~417 ns)
//
// All four of those timings land squarely inside the datasheet windows
// (T0H 400 +/- 150, T0L 850 +/- 150, T1H 800 +/- 150, T1L 450 +/- 150).
//
// A reset pulse of >= 50 us low is required to latch the frame; we pad the
// buffer with enough trailing zero bytes to cover that (120 SPI bits at
// 2.4 MHz = 50 us, so we use 24 bytes = 80 us to be safe).
//
// The strip is configurable via env vars -- see opi5_gpio.cpp -- so the
// user can pick the SPI device, speed, color order, and LED count without
// recompiling.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace limelight::hal {

enum class Ws2812Order {
    GRB,    // WS2812B, WS2813, SK6812 RGB  (most common)
    RGB,    // WS2811 and some clones
    GRBW,   // SK6812 RGBW  (4 bytes per pixel)
    RGBW,   // rare RGBW variants
};

struct Ws2812Color {
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    uint8_t w = 0;  // only written for *W orders
};

class Ws2812Driver {
public:
    Ws2812Driver() = default;
    ~Ws2812Driver();

    // Opens /dev/spidev* at the given clock rate, configures mode 0, and
    // allocates encoding buffers for `num_leds` pixels.  Returns false on
    // any error (device missing, ioctl failure, unsupported speed).
    bool open(const std::string& spi_path,
              uint32_t           speed_hz,
              Ws2812Order        order,
              int                num_leds);

    void close();
    bool isOpen() const { return fd_ >= 0; }

    int  count() const { return num_leds_; }

    // Per-pixel / bulk setters.  These only mutate the pixel cache; call
    // flush() to push the frame to the strip.
    void fill(const Ws2812Color& c);
    void setPixel(int index, const Ws2812Color& c);

    // Encodes pixels_ into the SPI buffer and writes it in one transfer.
    bool flush();

private:
    int                        fd_            = -1;
    uint32_t                   speed_hz_      = 2'400'000;
    Ws2812Order                order_         = Ws2812Order::GRB;
    int                        num_leds_      = 0;
    int                        bytes_per_led_ = 3;   // 3 for GRB/RGB, 4 for *W
    std::string                path_;
    std::vector<Ws2812Color>   pixels_;
    std::vector<uint8_t>       spi_buf_;             // encoded SPI MOSI

    void encodeByte(uint8_t src, uint8_t* dst);      // 1 color byte -> 3 SPI bytes
    void encodeFrame();                              // pixels_ -> spi_buf_
};

}  // namespace limelight::hal
