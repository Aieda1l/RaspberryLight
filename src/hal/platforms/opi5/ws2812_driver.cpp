#include "ws2812_driver.h"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <spdlog/spdlog.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace limelight::hal {

namespace {

// 1 SPI bit at 2.4 MHz = ~417 ns.
// WS2812 "0": 0b100 (T0H=417 ns, T0L=833 ns)  -- spec T0H=400+-150, T0L=850+-150
// WS2812 "1": 0b110 (T1H=833 ns, T1L=417 ns)  -- spec T1H=800+-150, T1L=450+-150
//
// Pack 8 WS2812 bits into 3 SPI bytes (24 bits).  Each WS2812 bit expands
// to 3 SPI bits, so the table below maps every possible nibble to its
// 12-bit SPI expansion; we look up the high and low nibble of each color
// byte and concatenate.
//
// nibble bits b3 b2 b1 b0 -> 12-bit SPI:
//   each bit: 1 -> 0b110, 0 -> 0b100
constexpr uint16_t kNibbleLut[16] = {
    0b100'100'100'100,  // 0x0
    0b100'100'100'110,  // 0x1
    0b100'100'110'100,  // 0x2
    0b100'100'110'110,  // 0x3
    0b100'110'100'100,  // 0x4
    0b100'110'100'110,  // 0x5
    0b100'110'110'100,  // 0x6
    0b100'110'110'110,  // 0x7
    0b110'100'100'100,  // 0x8
    0b110'100'100'110,  // 0x9
    0b110'100'110'100,  // 0xA
    0b110'100'110'110,  // 0xB
    0b110'110'100'100,  // 0xC
    0b110'110'100'110,  // 0xD
    0b110'110'110'100,  // 0xE
    0b110'110'110'110,  // 0xF
};

// Reset / latch padding.  WS2812B datasheet says >=50 us low; SK6812 and
// many WS2812 clones want >=80 us.  At 2.4 MHz one byte = 3.33 us, so 32
// bytes of 0x00 = ~106 us — enough headroom for every strip I've tested
// (SK6812 RGBW, WS2812B V5, WS2813, BTF-Lighting clones) without causing
// visible flicker from overly long resets.
constexpr int kResetBytes = 32;

}  // namespace

Ws2812Driver::~Ws2812Driver() { close(); }

bool Ws2812Driver::open(const std::string& spi_path,
                        uint32_t           speed_hz,
                        Ws2812Order        order,
                        int                num_leds) {
    close();

    if (num_leds <= 0) {
        spdlog::warn("Ws2812Driver: refusing to open with num_leds={}", num_leds);
        return false;
    }
    if (speed_hz < 1'800'000 || speed_hz > 3'200'000) {
        spdlog::warn("Ws2812Driver: speed {} Hz is outside the safe 2.0-3.0 MHz "
                     "window for 3-bits-per-symbol encoding; continuing anyway",
                     speed_hz);
    }

    fd_ = ::open(spi_path.c_str(), O_WRONLY);
    if (fd_ < 0) {
        spdlog::warn("Ws2812Driver: unable to open {}: {}",
                     spi_path, std::strerror(errno));
        return false;
    }

    uint8_t  mode  = SPI_MODE_0;     // CPOL=0, CPHA=0
    uint8_t  bits  = 8;
    uint32_t speed = speed_hz;

    if (::ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
        ::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        spdlog::warn("Ws2812Driver: SPI ioctl failed on {}: {}",
                     spi_path, std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    path_          = spi_path;
    speed_hz_      = speed;
    order_         = order;
    num_leds_      = num_leds;
    bytes_per_led_ = (order == Ws2812Order::GRBW || order == Ws2812Order::RGBW) ? 4 : 3;

    pixels_.assign(num_leds, Ws2812Color{});
    // 3 SPI bytes encode 1 color byte; plus leading + trailing reset padding.
    spi_buf_.assign(kResetBytes + num_leds * bytes_per_led_ * 3 + kResetBytes, 0);

    spdlog::info("Ws2812Driver: {} opened @ {} Hz, order={}, {} LEDs, buf={} bytes",
                 path_, speed_hz_,
                 order_ == Ws2812Order::GRB  ? "GRB"  :
                 order_ == Ws2812Order::RGB  ? "RGB"  :
                 order_ == Ws2812Order::GRBW ? "GRBW" : "RGBW",
                 num_leds_, spi_buf_.size());
    return true;
}

void Ws2812Driver::close() {
    if (fd_ >= 0) {
        // Try to leave the strip dark rather than stuck on the last frame.
        std::fill(pixels_.begin(), pixels_.end(), Ws2812Color{});
        flush();
        ::close(fd_);
    }
    fd_ = -1;
    pixels_.clear();
    spi_buf_.clear();
    num_leds_ = 0;
}

void Ws2812Driver::fill(const Ws2812Color& c) {
    for (auto& p : pixels_) p = c;
}

void Ws2812Driver::setPixel(int index, const Ws2812Color& c) {
    if (index < 0 || index >= num_leds_) return;
    pixels_[index] = c;
}

void Ws2812Driver::encodeByte(uint8_t src, uint8_t* dst) {
    // High nibble -> first 12 SPI bits, low nibble -> next 12 SPI bits.
    // Pack the 24 SPI bits into 3 big-endian bytes (MSB shifted out first).
    const uint16_t hi = kNibbleLut[(src >> 4) & 0x0F];
    const uint16_t lo = kNibbleLut[ src       & 0x0F];
    const uint32_t word = (static_cast<uint32_t>(hi) << 12) | lo;
    dst[0] = static_cast<uint8_t>((word >> 16) & 0xFF);
    dst[1] = static_cast<uint8_t>((word >>  8) & 0xFF);
    dst[2] = static_cast<uint8_t>( word        & 0xFF);
}

void Ws2812Driver::encodeFrame() {
    uint8_t* cursor = spi_buf_.data() + kResetBytes;
    for (const auto& p : pixels_) {
        uint8_t color_bytes[4];
        switch (order_) {
            case Ws2812Order::GRB:
                color_bytes[0] = p.g; color_bytes[1] = p.r; color_bytes[2] = p.b;
                break;
            case Ws2812Order::RGB:
                color_bytes[0] = p.r; color_bytes[1] = p.g; color_bytes[2] = p.b;
                break;
            case Ws2812Order::GRBW:
                color_bytes[0] = p.g; color_bytes[1] = p.r;
                color_bytes[2] = p.b; color_bytes[3] = p.w;
                break;
            case Ws2812Order::RGBW:
                color_bytes[0] = p.r; color_bytes[1] = p.g;
                color_bytes[2] = p.b; color_bytes[3] = p.w;
                break;
        }
        for (int i = 0; i < bytes_per_led_; ++i) {
            encodeByte(color_bytes[i], cursor);
            cursor += 3;
        }
    }
    // Trailing reset bytes were pre-zeroed at open() time.
}

bool Ws2812Driver::flush() {
    if (fd_ < 0) return false;

    encodeFrame();

    spi_ioc_transfer tr{};
    tr.tx_buf        = reinterpret_cast<__u64>(spi_buf_.data());
    tr.rx_buf        = 0;
    tr.len           = static_cast<__u32>(spi_buf_.size());
    tr.speed_hz      = speed_hz_;
    tr.bits_per_word = 8;
    tr.cs_change     = 0;
    tr.delay_usecs   = 0;

    if (::ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
        spdlog::debug("Ws2812Driver: SPI_IOC_MESSAGE failed: {}",
                      std::strerror(errno));
        return false;
    }
    return true;
}

}  // namespace limelight::hal
