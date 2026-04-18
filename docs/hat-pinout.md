# RaspberryLight Hat — pin contract

This is the contract between the RaspberryLight firmware and the custom
Orange Pi 5 hat PCB.  Lines listed below are *defaults*; every line can be
overridden at runtime with the environment variables noted in the table so
a PCB revision that reshuffles pins doesn't require a rebuild.

The hat plugs onto the Orange Pi 5's 40-pin header.  Everything on the hat
runs off a wide-input (7–24 V) supply that's fed straight into the GPIO
header and regulated by an on-hat buck-boost controller (bypassing the
USB-C input).

## Hat block summary

| Block        | Function                                              |
|--------------|-------------------------------------------------------|
| Power stage  | TVS + LC filter + reverse-polarity FET + buck-boost   |
| IMU          | Socketed Adafruit BNO085 breakout (SH-2 over I²C)     |
| Status LEDs  | 3× GPIO-driven indicator LEDs (power / heartbeat / link) |
| Addressable  | WS2812/SK6812 strip header with 74AHCT125 3.3→5 V shifter |
| Fan          | 4-pin PWM fan header (25 kHz PWM)                     |
| M.2 passthru | Hailo-8 sits in the OPi5's M-key slot, not on the hat |
| Camera       | Arducam OV9281 connects directly to the OPi5 MIPI CSI |

## Default pin contract

All lines are addressed as `(gpiochip, line)` pairs via **libgpiod**.  Chip
and line numbers on the RK3588 don't map to a published naming scheme —
confirm with `gpiodetect` and `gpioinfo` after a kernel upgrade, which is
why the env-var overrides exist.

| Signal            | Default chip     | Default line | Env override (chip / line)                     |
|-------------------|------------------|--------------|------------------------------------------------|
| Pipeline LED (on-board vision ring) | /dev/gpiochip1 | 8  | `LL_LED_CHIP` / `LL_LED_LINE`                |
| Pipeline LED PWM mode (SPI WS2812) | —                | —           | `LL_LED_MODE=WS2812`, `LL_LED_SPI=/dev/spidev0.0` |
| Status LED — power      | /dev/gpiochip4 | 0 | `LL_STATUS_PWR_CHIP`  / `LL_STATUS_PWR_LINE`    |
| Status LED — heartbeat  | /dev/gpiochip4 | 1 | `LL_STATUS_HB_CHIP`   / `LL_STATUS_HB_LINE`     |
| Status LED — link       | /dev/gpiochip4 | 2 | `LL_STATUS_LINK_CHIP` / `LL_STATUS_LINK_LINE`   |
| IMU INT (BNO085 H_INTN) | /dev/gpiochip1 | 27 | `LL_IMU_INT_CHIP`    / `LL_IMU_INT_LINE`       |
| IMU RST (BNO085 nRESET) | /dev/gpiochip1 | 28 | `LL_IMU_RST_CHIP`    / `LL_IMU_RST_LINE`       |
| Fan PWM                 | —              | — | `LL_FAN_PWM=/sys/class/pwm/pwmchipN/pwm0`       |

The IMU I²C bus address is the Adafruit breakout's default (0x4A when ADR
is floating, 0x4B when pulled high).  Override with `LL_IMU_I2C` for the
bus device node and `LL_IMU_ADDR` for the 7-bit address.

## Power stage

- **Input**: up to 24 V, polarity-protected by a P-channel MOSFET
- **Transient suppression**: high-current TVS diode on the input
- **Input filter**: series inductor + X7R ceramic cap for EMI
- **Regulation**: buck-boost controller producing the 5 V hat rail
- The OPi5's USB-C *is not* powered by the hat; the hat feeds the 5 V pin
  on the GPIO header directly.  Don't plug both sources in simultaneously.

## First-boot verification

After flashing a fresh image, before trusting the pin contract:

```sh
gpiodetect                                   # enumerate gpiochip names
gpioinfo gpiochip1 | head -40                # inspect line 8 / 27 / 28
gpioinfo gpiochip4 | head -10                # inspect status-LED lines
ls /sys/class/pwm                            # verify hw PWM chip present
i2cdetect -y 3                               # BNO085 should ack at 0x4A
cat /sys/class/thermal/thermal_zone*/type    # confirm RK3588 zone names
```

If any of the defaults disagree with what's actually present, set the env
override in `/etc/default/visionserver` (a shell-sourced file the systemd
unit pulls in via `EnvironmentFile=`).

## Pipeline LED — two modes

The on-camera indicator ring can be driven as either:

1. **Plain GPIO** (default): a single line toggled on/off for targeting
   feedback.  Cheapest option, no bill-of-materials impact.
2. **WS2812/SK6812 addressable**: set `LL_LED_MODE=WS2812` and point
   `LL_LED_SPI` at the SPI-MOSI-connected strip.  Firmware re-encodes the
   color onto SPI at 2.4 MHz (3 SPI bits per WS2812 symbol) and pads with
   a 24-byte reset latch.  The hat includes a 74AHCT125 to shift MOSI from
   3.3 V to 5 V for reliable WS2812 timing.

## IMU hookup details

- Socketed on 0.1" female headers so an Adafruit BNO085 breakout drops in
- Communication is I²C (400 kHz).  UART-RVC mode is *not* used — we run
  the full SH-2 protocol to get the Game Rotation Vector (feature 0x08)
  at 100 Hz, which is gyro+accel fused and immune to magnetic anomalies
- H_INTN is wired to a dedicated GPIO (not a bus shared with the LEDs) so
  the driver can use `gpiod_line_request_falling_edge_events` instead of
  polling the I²C bus at a fixed cadence
- nRESET is pulled by the driver at startup for a clean boot

The driver keeps a 256-slot lock-free ring buffer of `(timestamp_ns, yaw,
pitch, roll)` samples so the fiducial pipeline can align the fused yaw
against the camera capture timestamp (important because 100 Hz IMU vs a
60–90 Hz vision pipeline introduces up to ~10 ms of phase offset).

## Fan control

The hat's 4-pin fan header is driven off one of the RK3588 hardware PWM
channels (usually `pwmchip0` or `pwmchip1`; confirm per-board).  At init
the thermal HAL exports `pwm0`, writes `period=40000 ns` (25 kHz — the
standard 4-pin PC-fan spec), sets `duty_cycle=0`, and enables the output.
`setFanSpeed(pct)` writes `duty_cycle = period * pct / 100`.

If no hardware PWM is discoverable, the thermal HAL falls back to the
kernel hwmon/cooling-device nodes in step mode.

## Hailo-8 addon

The Hailo-8 sits in the OPi5's M.2 M-key slot (underside of the board).
The firmware talks to it via `libhailort`, same as on the original
Limelight.  The hat doesn't carry any Hailo-related traces.
