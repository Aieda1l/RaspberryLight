#include "hal/camera.h"
#include "hal/gpio.h"
#include "hal/imu.h"
#include "hal/npu.h"
#include "hal/status_leds.h"
#include "hal/thermal.h"
#include "hal/usb_gadget.h"

#if defined(PLATFORM_RPI)
#include "../platforms/rpi/rpi_camera.h"
#include "../platforms/rpi/rpi_gpio.h"
#include "../platforms/rpi/rpi_thermal.h"
#elif defined(PLATFORM_OPI5)
#include "../platforms/opi5/opi5_camera.h"
#include "../platforms/opi5/opi5_gpio.h"
#include "../platforms/opi5/opi5_imu_bno085.h"
#include "../platforms/opi5/opi5_status_leds.h"
#include "../platforms/opi5/opi5_thermal.h"
#endif

namespace limelight::hal {

std::unique_ptr<Camera> Camera::create() {
#if defined(PLATFORM_RPI)
    return std::make_unique<RpiCamera>();
#elif defined(PLATFORM_OPI5)
    return std::make_unique<Opi5Camera>();
#else
    #error "No platform defined. Set LIMELIGHT_PLATFORM to RPI or OPI5."
#endif
}

std::unique_ptr<Gpio> Gpio::create() {
#if defined(PLATFORM_RPI)
    return std::make_unique<RpiGpio>();
#elif defined(PLATFORM_OPI5)
    return std::make_unique<Opi5Gpio>();
#else
    #error "No platform defined."
#endif
}

std::unique_ptr<Thermal> Thermal::create() {
#if defined(PLATFORM_RPI)
    return std::make_unique<RpiThermal>();
#elif defined(PLATFORM_OPI5)
    return std::make_unique<Opi5Thermal>();
#else
    #error "No platform defined."
#endif
}

std::unique_ptr<Imu> Imu::create() {
#if defined(PLATFORM_OPI5)
    return std::make_unique<Opi5Imu>();
#else
    // No default IMU on RPi: the original Limelight 4 has one wired
    // directly to the Pi header, but on non-OPi5 ports we simply return
    // nullptr and let the pipeline fall back to external-gyro mode.
    return nullptr;
#endif
}

std::unique_ptr<StatusLeds> StatusLeds::create() {
#if defined(PLATFORM_OPI5)
    return std::make_unique<Opi5StatusLeds>();
#else
    return nullptr;
#endif
}

std::vector<InferenceBackend> InferenceEngine::availableBackends() {
    std::vector<InferenceBackend> backends;
#ifdef HAS_TFLITE
    backends.push_back(InferenceBackend::TFLITE_CPU);
#endif
#ifdef HAS_RKNN
    backends.push_back(InferenceBackend::RKNN);
#endif
#ifdef HAS_HAILO
    backends.push_back(InferenceBackend::HAILO);
#endif
#ifdef HAS_EDGETPU
    backends.push_back(InferenceBackend::EDGETPU);
#endif
    return backends;
}

} // namespace limelight::hal
