#pragma once

#include <opencv2/core.hpp>
#include <memory>
#include <functional>
#include <string>

namespace limelight::hal {

struct CameraControls {
    double exposure = 120.0;
    double gain = 6.0;
    double red_balance = 1200.0;
    double blue_balance = 1975.0;
    int black_level = 0;
    int flicker = 0;       // 0=off, 1=50Hz, 2=60Hz
    int flip = 0;           // 0=none, 1=h, 2=v, 3=both
    int resolution = 0;     // 0=full, 1=half, etc.
};

struct CameraInfo {
    int width;
    int height;
    int fps;
    std::string sensor_name;
    std::string driver;
};

class Camera {
public:
    virtual ~Camera() = default;

    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;

    virtual bool startCapture() = 0;
    virtual void stopCapture() = 0;

    // Get next frame. Blocks until frame available or timeout.
    // Returns empty Mat on failure/timeout.
    virtual cv::Mat getFrame(int timeout_ms = 1000) = 0;

    virtual void setControls(const CameraControls& controls) = 0;
    virtual CameraControls getControls() const = 0;
    virtual CameraInfo getInfo() const = 0;

    // Factory: creates platform-appropriate camera
    static std::unique_ptr<Camera> create();
};

} // namespace limelight::hal
