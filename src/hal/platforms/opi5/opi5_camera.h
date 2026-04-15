#pragma once

#include "hal/camera.h"
#include <string>

namespace limelight::hal {

// Orange Pi 5 camera implementation using V4L2 + Rockchip ISP2
// Designed for Arducam OV9281 (1MP, global shutter, monochrome)
// Sensor: 1280x800 Y8/Y10 over MIPI CSI-2
// Kernel driver: ov9282.ko (handles both OV9281 and OV9282)
class Opi5Camera : public Camera {
public:
    Opi5Camera();
    ~Opi5Camera() override;

    bool open() override;
    void close() override;
    bool isOpen() const override;

    bool startCapture() override;
    void stopCapture() override;

    cv::Mat getFrame(int timeout_ms = 1000) override;

    void setControls(const CameraControls& controls) override;
    CameraControls getControls() const override;
    CameraInfo getInfo() const override;

private:
    bool initV4l2();
    bool initMmap();
    void cleanupMmap();
    bool setV4l2Control(uint32_t id, int32_t value);
    int32_t getV4l2Control(uint32_t id);

    int fd_ = -1;
    std::string device_path_;

    // Memory-mapped buffers for zero-copy capture
    struct MmapBuffer {
        void* start = nullptr;
        size_t length = 0;
    };
    static constexpr int NUM_BUFFERS = 4;
    MmapBuffer buffers_[NUM_BUFFERS];
    int num_buffers_ = 0;

    int width_ = 1280;
    int height_ = 800;
    int fps_ = 60;
    bool capturing_ = false;
    CameraControls controls_;
};

} // namespace limelight::hal
