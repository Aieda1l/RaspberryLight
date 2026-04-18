# Implementation Plan: IMU-Fused MegaTag + Hat Integration

**Spec:** `2026-04-17-imu-fused-megatag-hat.md`
**Execution mode:** single branch on `main` (existing working style), commit per task, push at the end.
**Verification:** each task finishes with a compile + self-check. We can't run the full binary on Windows (targets aarch64 Linux), so verification is "configures cleanly / no missing symbols / static analysis passes". End-to-end validation is deferred to bench testing on the OPi5 hat.

---

## Task 1 — HAL interfaces and factory wiring

**Files:**
- `src/hal/include/hal/imu.h` (new) — `Orientation` struct + `Imu` abstract base + `create()` factory.
- `src/hal/include/hal/status_leds.h` (new) — `StatusLed`/`StatusState` enums + `StatusLeds` abstract base + `create()` factory.
- `src/hal/hal_factory.cpp` — add platform-dispatched factories for the two new interfaces.

**Steps:**
1. Write `imu.h` matching the spec signature (yaw/pitch/roll, timestamp, accuracy, `latest()`, `sampleAt()`, `tareYaw()`).
2. Write `status_leds.h` matching the spec.
3. Open `hal_factory.cpp`, add `Imu::create()` and `StatusLeds::create()` dispatch functions under `#ifdef PLATFORM_OPI5` forwarding to `std::make_unique<Opi5Imu>()` / `std::make_unique<Opi5StatusLeds>()`. Both classes are forward-declared only — the full includes come in Tasks 2 and 3.
4. Add both header files to `CMakeLists.txt` public include dir coverage (they are already under `src/hal/include` which is already on the include path; verify no extra changes needed).

**Verification:** `hal_factory.cpp` compiles (symbolically — defer until Task 2/3). No regression in the existing `limelight-hal` target sources list.

---

## Task 2 — BNO085 SH-2 I²C driver

**Files:**
- `src/hal/platforms/opi5/opi5_imu_bno085.h` (new)
- `src/hal/platforms/opi5/opi5_imu_bno085.cpp` (new)
- `CMakeLists.txt` — add new source to `HAL_SOURCES` under `OPI5` branch.

**Steps:**
1. Header declares `Opi5Imu : public Imu` with opaque `void* impl_` pimpl (SH-2 state + ring buffer kept out of header).
2. In the .cpp, define the SH-2 state struct:
   - I²C fd, address.
   - INT gpiod line, RST gpiod line.
   - Sequence numbers per channel (0..5).
   - Lock-free 256-slot ring buffer of `Orientation` indexed by atomic head/tail.
   - Read thread + stop flag.
3. Implement `init()`:
   - Open `/dev/i2c-3` (env: `LL_IMU_I2C`), `I2C_SLAVE` ioctl to `0x4A` (env: `LL_IMU_ADDR`).
   - Acquire INT + RST lines via libgpiod (env: `LL_IMU_INT_CHIP/LINE`, `LL_IMU_RST_CHIP/LINE`). INT as `gpiod_line_request_falling_edge_events`, RST as output starting high.
   - Pulse RST low 10 ms → high, wait for INT falling edge up to 500 ms.
   - Drain advertisement: while INT low, read SHTP header, read remaining payload, discard.
   - Send Set Feature Command for report 0x08 (Game Rotation Vector), sample interval 10 000 µs, zero flags. 21-byte control payload on channel 2.
   - Wait for first Game RV report to confirm activation.
   - Spawn read thread; return true.
4. Implement read loop:
   - `gpiod_line_event_wait` on INT with 200 ms timeout.
   - On edge: read 4-byte header (little-endian length + flags + channel + sequence). Validate length ≤ 32768. If 0, no data — continue.
   - Read `length - 4` bytes.
   - Dispatch by channel: channel 3 (Input sensor reports) with report ID 0x08 → decode quaternion (i16 i, j, k, real; Q14), accuracy (i16, Q12), status byte. Convert to yaw/pitch/roll via ZYX Tait-Bryan. Timestamp with `CLOCK_MONOTONIC` at entry.
   - Push to ring buffer.
   - Other channels: log once per (channel, report_id) pair and ignore.
5. Implement `latest()`: copy tail slot under acquire fence.
6. Implement `sampleAt(ts_ns)`: binary search ring buffer for samples straddling `ts_ns`, return linear interpolation (quaternions slerp would be ideal but Euler lerp is sufficient at 100 Hz given <10 ms gap).
7. Implement `tareYaw()`: atomic double offset that `latest()`/`sampleAt()` subtract before returning yaw. Not a chip-side tare — simpler and reversible.
8. Destructor: stop thread, drive RST low, release lines, close fd.

**Verification:** `cmake --build` completes on target (symbolic check: compile-only). Inspect code for obvious FRAMING bugs: header endian, Q-point conversion, radian vs degree.

---

## Task 3 — Status LEDs backend

**Files:**
- `src/hal/platforms/opi5/opi5_status_leds.h` (new)
- `src/hal/platforms/opi5/opi5_status_leds.cpp` (new)
- `CMakeLists.txt` — add to `HAL_SOURCES`.

**Steps:**
1. Header declares `Opi5StatusLeds : public StatusLeds` with opaque pimpl.
2. cpp allocates 3 gpiod lines (chip/line pairs from env vars with defaults), one shared worker thread waking every 10 ms.
3. Worker reads `std::atomic<StatusState>` per LED and a monotonic tick counter to drive blink phase. No per-LED thread.
4. `init()` requests all 3 lines as outputs low; returns true if at least one succeeds (missing hat should not fail the daemon).
5. `set(which, state)` atomically writes state slot; the worker picks it up on the next tick.
6. Destructor stops thread, drives all low, releases lines.

**Verification:** Compiles clean. Thread shutdown path cannot deadlock (stop flag is atomic bool, worker polls).

---

## Task 4 — Thermal fan PWM via `/sys/class/pwm`

**Files:**
- `src/hal/platforms/opi5/opi5_thermal.cpp` (modify)
- `src/hal/platforms/opi5/opi5_thermal.h` (modify — add `pwm_period_ns_` member)

**Steps:**
1. In `init()`, after the existing fan-path discovery, also check `LL_FAN_PWM` env var or the first existing `/sys/class/pwm/pwmchip*/pwm0/duty_cycle`.
2. If PWM sysfs path chosen:
   - If `/sys/class/pwm/pwmchipN/pwm0` does not exist, write `0` to `/sys/class/pwm/pwmchipN/export`.
   - Write `40000` (25 kHz period) to `period`.
   - Write `1` to `enable`.
   - Store path to `duty_cycle` and period to `pwm_period_ns_`.
3. `setFanSpeed(pct)`: if PWM path is active, write `pwm_period_ns_ * pct / 100` to `duty_cycle`. Else fall through to the existing hwmon/cooling_device path.
4. Log which path was selected once at init.

**Verification:** Compiles clean. Existing hwmon path still works (no regression: the new code is additive and behind a presence check).

---

## Task 5 — Fiducial pipeline MegaTag2 tier-A integration

**Files:**
- `src/visionserver/include/visionserver/fiducial_pipeline.h` (modify)
- `src/visionserver/fiducial_pipeline.cpp` (modify)

**Steps:**
1. Add public API: `setImu(hal::Imu*)`, `setRobotOrientation(const std::array<double,6>&)`, `setImuMode(int)`, `setImuAssistAlpha(double)`. Constants `kImuModeExternal=0`, `kImuModeInternal=1`, `kImuModeAssist=2`.
2. Private state: non-owning `hal::Imu* imu_ = nullptr;`, `std::array<double,6> robot_orientation_{}`, `std::atomic<int> imu_mode_{0}`, `std::atomic<double> imu_assist_alpha_{1.0}`, `std::atomic<double> yaw_tare_offset_{0.0}`.
3. In `process()`, after the existing classic PnP solve for each detected tag:
   - Compute `yaw_ext = robot_orientation_[5]` (index for yaw in `robot_orientation_set`, verified against Limelight convention).
   - Compute `yaw_int`: if `imu_` non-null and `imu_->sampleAt(frame_capture_ts).valid`, use that yaw; else fall back to `yaw_ext`.
   - Blend: switch on `imu_mode_`.
4. Yaw-constrained refinement — per tag:
   - Build tag object-space corners (existing helper `tagObjectCorners`).
   - Use the classic solve's rvec/tvec as initial guess.
   - Call `cv::solvePnPRefineLM` on it.
   - Decompose rvec → R, extract yaw (atan2 of R(1,0)/R(0,0)), force yaw to `yaw_fused` by rebuilding R with the fused yaw and refined pitch/roll, re-encode rvec.
   - Recompute tvec: given fixed R, project object corners to image, solve 2-tvec-variables linear system against the observed image corners (the translation part of PnP with known rotation is linear). This is ~15 lines of OpenCV math.
   - Compute reprojection error; store `(tvec, rvec, err, tag_area_px)` for this tag.
5. Aggregate across visible tags with field-map entries:
   - `weight_i = tag_area_i / (err_i² + 1.0)`.
   - Weighted mean of tvecs, yaw = `yaw_fused`, pitch/roll from the best (lowest-err) tag.
6. Populate `PipelineResult::botpose_orb`, `botpose_orb_wpired`, `botpose_orb_wpiblue` (new fields). The red/blue transforms reuse the same helper the classic `botpose_wpired/wpiblue` path uses.
7. Preserve the classic `botpose` path unchanged.

**Verification:** Compiles. Static check: yaw-fused solve gracefully falls back to classic `botpose` if `imu_mode_` is invalid or `yaw_ext` is NaN.

---

## Task 6 — main.cpp and NT publisher wiring

**Files:**
- `src/visionserver/main.cpp` (modify)
- `src/visionserver/include/visionserver/pipeline_result.h` (modify — add 3 new fields)
- `src/visionserver/nt_publisher.cpp` (modify — publish `botpose_orb*` from the new fields)
- `src/visionserver/pipeline_manager.{h,cpp}` (modify — forward IMU + orientation to the fiducial pipeline in each slot)

**Steps:**
1. Extend `PipelineResult` with `std::array<double,6> botpose_orb{}`, `botpose_orb_wpired{}`, `botpose_orb_wpiblue{}` + `bool botpose_orb_valid=false`.
2. `nt_publisher.cpp::publish(...)` — in the existing `kOutBotposeOrb*` section, write the three arrays when `result.botpose_orb_valid`.
3. `pipeline_manager` — hold a raw `hal::Imu*` (non-owning) and fanout methods `setImu`, `setRobotOrientation`, `setImuMode`, `setImuAssistAlpha` that walk the slot vector and call the same on any `FiducialPipeline` slot.
4. `main.cpp`:
   - After `auto gpio = hal::Gpio::create();` line, add `auto imu = hal::Imu::create();` and `auto status = hal::StatusLeds::create();`. Call `init()` on each; log + continue on failure.
   - `status->set(POWER, ON)` after successful init.
   - Pass `imu.get()` to `pipeline_mgr.setImu(...)` before the vision loop.
   - Wire the existing `nt_publisher` callbacks:
     - `on_robot_orientation` → `pipeline_mgr.setRobotOrientation`
     - `on_imu_mode` → `pipeline_mgr.setImuMode`
     - `on_imu_assist_alpha` → `pipeline_mgr.setImuAssistAlpha`
   - `status->set(HEARTBEAT, BLINK_SLOW)` once the first frame is processed; `set(LINK, ON)` if `nt_publisher.isConnected()` (add accessor if missing).
   - On shutdown: `imu->shutdown()`, `status->set(POWER, OFF)`.

**Verification:** Compiles. `main.cpp` still runs on a bare dev board with no hat: IMU init fails → pipeline runs with `imu_ == nullptr` → falls through to external/classic mode only. Status LED init fails → no-op setters. No hard dependency on the hat.

---

## Task 7 — Hat pinout documentation

**Files:**
- `docs/hat-pinout.md` (new)

**Steps:**
1. Copy the pin contract table from the spec into this doc.
2. Add a second table mapping each signal to its env-var override.
3. Add a "First boot" section documenting how to identify gpiochip/line numbers on a new OPi5 kernel (`gpiodetect`, `gpioinfo`).

**Verification:** Markdown renders. Table formatting consistent.

---

## Task 8 — CMake + final sweep

**Files:**
- `CMakeLists.txt`

**Steps:**
1. Verify all three new .cpp files (`opi5_imu_bno085.cpp`, `opi5_status_leds.cpp`) are in `HAL_SOURCES` under the `OPI5` branch.
2. No new external dependencies — everything uses libc / libgpiod / OpenCV we already link.
3. Configure the project locally (`cmake -S . -B build -DLIMELIGHT_PLATFORM=OPI5`) — on Windows this will likely fail finding libcamera/rknn, but the CMake parse itself should succeed. Report the outcome.

**Verification:** `cmake --version` runs; run `cmake -P - <<< "message(\"ok\")"` to confirm CMake itself is available; attempt project configure, capturing expected-vs-unexpected errors. If the parse errors are just missing OS-side libs, that's expected on Windows.

---

## Final step

Single commit with body enumerating the above tasks, `git push`, then summarize to the user what was done, what needs bench testing, and what's still deferred (tier B/C bundle adjust, GIRV layering, hat-ID EEPROM).
