# IMU-Fused AprilTag Localization + OPi5 Hat Integration

**Date:** 2026-04-17
**Status:** Approved — ready for implementation plan
**Scope:** Add a BNO085-based IMU HAL and MegaTag2-style yaw-fused pose solver to the vision server, plus the hat-side hardware integration (fan PWM, status LEDs, pin contract).

---

## Goal

Bring the Orange Pi 5 port to 1:1 feature parity with Limelight 4 for IMU-fused AprilTag localization. The real firmware publishes `botpose_orb*` NetworkTables topics that use a fused yaw (robot gyro + internal IMU, blended per `imu_mode`) to disambiguate single-tag PnP. Our codebase already has the NT/REST scaffolding for this; it does not have an IMU driver, a solver, or the hat pinout contract.

Secondary goal: lock down the hat-side hardware contract (pin assignments, bus choices, LED/fan wiring) so the custom PCB can be routed without waiting for the firmware to settle.

---

## Non-goals

- Multi-tag bundle adjustment (tier B/C of MegaTag2). Tier A (single-tag yaw-constrained PnP with weighted averaging across visible tags) is the initial implementation; tier B is deferred.
- Hat-identification EEPROM / revision enforcement. Nice to have, not required for v1.
- Push-buttons. The Limelight 4 has none; the hat mirrors that.
- Gyro-Integrated Rotation Vector (high-rate yaw) — deferred; adding it later does not break the HAL interface.
- Battery fuel gauge / RTC backup.
- Buildroot OS image changes. That is Phase 5 of the overarching project plan and is out of scope here.

---

## Hardware summary

The hat is a custom PCB that mounts on the OPi5 40-pin header and carries:

| Component | Interface | Notes |
|-----------|-----------|-------|
| Adafruit BNO085 breakout | I²C-3 + INT + RST | Socketed on 0.1" headers so the module is replaceable |
| WS2812B / SK6812 LED strip header | SPI1 MOSI via 74AHCT125 level shifter | 5V strip supply, 1000 µF reservoir cap, 500 Ω series R |
| Fan header | RK3588 hardware PWM | Optional tach input; PWM is sufficient for closed loop |
| Status LEDs | 3× GPIOs (power / heartbeat / link) | Dumb LEDs, direct drive through current-limit resistors |
| Buck-boost DC-DC | Pure hardware | Feeds 5V into the 40-pin header, bypassing USB-C PD |
| OV9281 | CSI ribbon direct to OPi5 | Not on the hat; listed for completeness |
| Hailo-8 | M.2 M-Key on OPi5 | Not on the hat; listed for completeness |

No buttons (matching the LL4).

### Pin contract

The firmware reads the pins from env vars with the defaults below, so the hat layout can change in rev B without a recompile. These defaults are what the PCB should be routed to for "zero config" out of the box.

| Signal | OPi5 40-pin | GPIO chip/line (default) | Env var |
|--------|-------------|--------------------------|---------|
| I²C-3 SDA | pin 3 | n/a (i2c-3 kernel device) | `LL_IMU_I2C=/dev/i2c-3` |
| I²C-3 SCL | pin 5 | n/a | — |
| IMU INT | pin 11 | gpiochip1 line 27 | `LL_IMU_INT_CHIP`, `LL_IMU_INT_LINE` |
| IMU RST | pin 13 | gpiochip1 line 28 | `LL_IMU_RST_CHIP`, `LL_IMU_RST_LINE` |
| SPI1 MOSI (LED data) | pin 19 | /dev/spidev1.0 | `LL_LED_SPI` (already implemented) |
| Fan PWM | pin 7 | pwmchip1 channel 0 | `LL_FAN_PWM=/sys/class/pwm/pwmchip1/pwm0` |
| Status LED: power | pin 29 | gpiochip4 line 0 | `LL_STATUS_PWR_CHIP`, `LL_STATUS_PWR_LINE` |
| Status LED: heartbeat | pin 31 | gpiochip4 line 1 | `LL_STATUS_HB_CHIP`, `LL_STATUS_HB_LINE` |
| Status LED: link | pin 33 | gpiochip4 line 2 | `LL_STATUS_LINK_CHIP`, `LL_STATUS_LINK_LINE` |

(Exact gpiochip/line numbers will be finalized when the first hat rev is booted; env vars are the safety net.)

---

## Software architecture

### New HAL interfaces

#### `src/hal/include/hal/imu.h`

```cpp
namespace limelight::hal {

struct Orientation {
    double   yaw_rad;     // -pi .. +pi, right-hand rule about Z
    double   pitch_rad;   // -pi/2 .. +pi/2
    double   roll_rad;    // -pi .. +pi
    double   yaw_rate_rps;
    uint64_t timestamp_ns;  // CLOCK_MONOTONIC
    uint8_t  accuracy;      // 0 (unreliable) .. 3 (high), from BNO085
    bool     valid;
};

class Imu {
public:
    virtual ~Imu() = default;

    virtual bool init() = 0;
    virtual void shutdown() = 0;

    // Latest sample. Returns {valid=false} if the driver has not produced
    // a sample yet (e.g. during first calibration after boot).
    virtual Orientation latest() const = 0;

    // Returns the sample whose timestamp is closest to the requested one,
    // optionally linearly interpolated between the two straddling samples.
    // This is what the fiducial pipeline uses to align yaw with frame
    // capture time.
    virtual Orientation sampleAt(uint64_t timestamp_ns) const = 0;

    // Zeroes the yaw reference without rebooting the chip. Equivalent of
    // the Limelight "tare" button in the web UI.
    virtual void tareYaw() = 0;

    static std::unique_ptr<Imu> create();
};

} // namespace limelight::hal
```

#### `src/hal/include/hal/status_leds.h`

```cpp
namespace limelight::hal {

enum class StatusLed : uint8_t { POWER, HEARTBEAT, LINK };
enum class StatusState : uint8_t { OFF, ON, BLINK_SLOW, BLINK_FAST };

class StatusLeds {
public:
    virtual ~StatusLeds() = default;
    virtual bool init() = 0;
    virtual void set(StatusLed which, StatusState state) = 0;
    static std::unique_ptr<StatusLeds> create();
};

} // namespace limelight::hal
```

### New OPi5 backend: BNO085 SH-2 I²C driver

**File:** `src/hal/platforms/opi5/opi5_imu_bno085.{h,cpp}`

The BNO085 speaks SH-2 (Sensor Hub Transport Protocol) over I²C. Key protocol properties:

- Host reads only when INT is asserted low.
- Every transfer begins with a 4-byte SHTP header (length LSB, length MSB, channel, sequence).
- Channel 2 carries Sensor Hub reports; we subscribe to Game Rotation Vector (report ID `0x08`).
- Feature enable is a 17-byte control command on channel 2 specifying the report ID, sample period in microseconds, and options.
- DFU and advertisement packets are ignored after the initial boot sequence.

**Initialization sequence:**
1. Open `/dev/i2c-3`, set slave address `0x4A` (Adafruit breakout default; also `0x4B` configurable).
2. Drive RST GPIO low for 10 ms, release, wait for the advertisement packet on INT (up to 500 ms).
3. Drain any pending advertisement / executable-channel messages.
4. Send `Set Feature Command` for report 0x08 with period 10000 µs (100 Hz) on channel 2.
5. Wait for the first Game Rotation Vector report to confirm the feature is active.
6. Spawn a read thread.

**Read loop (INT-driven):**
- Block on `gpiod_line_event_wait` for a falling edge with a 200 ms timeout.
- On edge: read header (4 bytes), then read `length - 4` bytes of payload.
- Dispatch by channel + report ID. For report 0x08:
  - Unpack i16 quaternion components (i, j, k, real), Q-point 14.
  - Unpack i16 accuracy estimate (radians, Q-point 12).
  - Convert quaternion → yaw/pitch/roll (ZYX Tait-Bryan).
  - Timestamp with `clock_gettime(CLOCK_MONOTONIC)` at the moment of INT assertion (read from `/sys/class/gpio/.../edge_time` if available, else just-now).
  - Push into a 256-slot lock-free ring buffer keyed by timestamp.

The driver is intentionally isolated: SH-2 framing, CRC-less packet semantics, and all magic numbers live in a single file. Tests can stub the I²C fd and feed canned SH-2 packets without touching real hardware.

### Thermal HAL extension

**File:** `src/hal/platforms/opi5/opi5_thermal.cpp`

Current `setFanSpeed` writes `pwm1` on the hwmon cooling device. On the hat the fan is driven by RK3588 hardware PWM via `/sys/class/pwm/pwmchipN/pwmM`. Changes:

1. Detect preferred path: if `LL_FAN_PWM` is set or `/sys/class/pwm/pwmchip*/pwm0` exists with a writable `duty_cycle`, use it.
2. First call exports the channel (write channel index to `export`), sets period to 40000 ns (25 kHz, standard 4-pin fan), enables the channel.
3. `setFanSpeed(pct)` writes `duty_cycle = period * pct / 100`.
4. Fallback to the existing hwmon path if PWM sysfs is absent, so the software still works on dev boards without the hat.

### Status LEDs backend

**File:** `src/hal/platforms/opi5/opi5_status_leds.{h,cpp}`

Three GPIO lines driven via libgpiod (reusing the pattern in `opi5_gpio.cpp`). Blink states are handled by a single shared worker thread that wakes every 10 ms, not a thread per LED — cheap, deterministic, matches the existing LED blink pattern. Each state:

| State | Behavior |
|-------|----------|
| OFF | Drive low |
| ON | Drive high |
| BLINK_SLOW | 500 ms on / 500 ms off (1 Hz) |
| BLINK_FAST | 100 ms on / 100 ms off (5 Hz) |

`main.cpp` drives them: POWER=ON at startup, HEARTBEAT=BLINK_SLOW when the vision loop is healthy, LINK=ON when NT is connected, BLINK_FAST on any error path.

### Vision server changes

#### `fiducial_pipeline.{h,cpp}`

Add four entry points that match the existing NT/REST wiring:

```cpp
void setRobotOrientation(const std::array<double, 6>& o);  // roll, pitch, yaw, rates
void setImuMode(int mode);                                 // 0=ext, 1=int, 2=assist
void setImuAssistAlpha(double alpha);                      // 0..1
void setImu(hal::Imu* imu);                                // non-owning, may be null
```

Plus the MegaTag2 tier-A solver, invoked from `process()`:

1. For each detected tag with a known field-map pose:
   - Run classic `solvePnP(SOLVEPNP_ITERATIVE)` with the tag's object-space corners → gives an initial estimate for `botpose` (this is what we already publish).
2. Fuse yaw:
   - `yaw_ext` = `robot_orientation[2]` from NT (radians).
   - `yaw_int` = `imu->sampleAt(frame_capture_ts).yaw_rad` if available.
   - Blend: `yaw_fused = mode==0 ? yaw_ext : mode==1 ? yaw_int : alpha*yaw_ext + (1-alpha)*yaw_int`.
3. Yaw-constrained refinement per tag:
   - Rotate the tag's object-space corners by `yaw_fused` in the field frame.
   - Run `solvePnPRefineLM` from the initial estimate, then **project-and-clamp** yaw: reconstruct the rotation matrix with the pre/post refinement, decompose, force yaw back to `yaw_fused`, rebuild R. This is the cheap substitute for a true constrained LM and is what the MegaTag2 white paper describes.
4. Aggregate across tags: weight each per-tag pose by `tag_area_px / reprojection_error_px²`, take the weighted mean of translations, yaw is the input `yaw_fused`.
5. Publish as `botpose_orb`, plus the red/blue alliance variants. Alliance transforms mirror the existing classic `botpose_wpired`/`botpose_wpiblue` logic (180° rotation about the field center, not a mirror) so downstream consumers don't have to special-case the orb variants.

The existing classic `botpose` publish path stays untouched so teams that only want MegaTag1 don't regress.

#### `main.cpp`

Additions:
- Construct `auto imu = hal::Imu::create();` after the existing GPIO/camera setup. If `imu->init()` fails, log a warning and continue — the external mode (robot gyro) still works.
- Construct `auto status = hal::StatusLeds::create();` and wire four transitions: boot, vision-loop-running, vision-error, NT-connected.
- Wire `pipeline_mgr` → `fiducial_pipeline::setImu(imu.get())` so every fiducial pipeline in any slot sees the same IMU.
- Wire the existing NT callbacks `on_robot_orientation`, `on_imu_mode`, `on_imu_assist_alpha` through the pipeline manager down to the fiducial pipeline.

**IMU mode IDs:** symbolic constants in `fiducial_pipeline.h` (`kImuModeExternal = 0`, `kImuModeInternal = 1`, `kImuModeAssist = 2`). Final numeric values must match whatever the Limelight web UI publishes on `imumode_set`; if the web UI turns out to use a different enumeration (to be confirmed during first integration test with the original UI bundle), the constants are updated in one place.

### HAL factory

`src/hal/hal_factory.cpp` gets two new `create()` entry points returning `Opi5Imu` / `Opi5StatusLeds` under `#ifdef PLATFORM_OPI5`. RPi stubs can be added later (the platform dispatches the same way as the existing camera/gpio/thermal factories).

---

## Data flow

```
 BNO085 (INT) ─▶ opi5_imu_bno085 read thread ─▶ ring buffer
                                                     │
                                                     ▼
 roboRIO NT ──▶ on_robot_orientation ──▶ fiducial_pipeline ◀── sampleAt(capture_ts)
                                               │
                                               ▼
                                    yaw-constrained PnP
                                               │
                                               ▼
                                     nt_publisher ──▶ botpose_orb*
```

---

## Testing

### Unit-level (host machine, no hardware)

- SH-2 frame parser: feed canned BNO085 advertisement + Game RV packets (captured from a dev board with logic analyzer), assert yaw/pitch/roll decode matches expected.
- Quaternion → Euler converter: property tests on random unit quaternions round-tripping through the converter.
- MegaTag2 tier-A solver: synthetic scene — place a virtual tag at known pose, reproject corners with noise, solve, assert `botpose_orb` error is bounded and monotonically better than unconstrained PnP when tag is near-frontal (the known failure mode of classic PnP).
- Ring buffer `sampleAt`: assert interpolation is correct at boundaries and outside range.

### Integration (requires the hat or a dev rig)

- Bench test: place a printed AprilTag at known world coords, rotate the camera slowly by hand, verify `botpose_orb` stays stable while classic `botpose` swings.
- Bench test: disconnect the IMU (pull RST low indefinitely), verify the pipeline falls back to external mode without crashing.
- Stress: 1 hour continuous run with the vision loop at full rate, assert no missed IMU samples, no ring buffer underruns.

### Regression

- Existing fiducial pipeline tests must still pass — the classic `botpose` path is untouched.
- WS2812 driver regression: confirm LED behavior is unchanged with `LL_LED_MODE` unset.

---

## Risk & mitigations

| Risk | Mitigation |
|------|------------|
| BNO085 SH-2 corner cases (executable channel resets mid-operation, DFU advertisements) | Drain and ignore anything that isn't a channel-2 input report for the first 500 ms after reset; log-once on unexpected channels |
| INT edge detection races with I²C read latency | Use `gpiod_line_event_wait` with a timeout and re-sync by draining any queued reports on timeout |
| Yaw-constrained LM doesn't converge when tag is tilted >45° | Fall back to the tag's classic `solvePnP` result and mark that sample as low-confidence in the aggregation |
| Hat PWM and SPI pin contention | SPI1 and PWM1 are on separate controllers — verified against RK3588 TRM |
| Status LEDs thread starves vision loop | One shared 10 ms worker for all 3 LEDs, `std::atomic` state, no blocking |

---

## Out of scope (for this spec, not forever)

- Full multi-tag bundle-adjusted MegaTag2 (tier B/C). Interface is forward-compatible.
- High-rate GIRV layering on top of Game RV.
- Hat EEPROM revision enforcement.
- Push-buttons of any kind.
- Buildroot image integration — deferred to the Phase 5 project plan.
