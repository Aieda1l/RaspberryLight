// Orange Pi 5 Camera HAL - V4L2 implementation for OV9281
//
// The OV9281 is a 1MP global shutter monochrome CMOS sensor.
// On Orange Pi 5 (RK3588), it connects via MIPI CSI-2 to the Rockchip ISP2.
// The kernel driver is ov9282.ko (handles both OV9281/9282 variants).
//
// Device tree overlay required for OV9281 on CSI-2 port.
// The ISP presents the processed output at /dev/video0 (or discoverable via media-ctl).

#include "opi5_camera.h"

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <iostream>
#include <dirent.h>

namespace limelight::hal {

Opi5Camera::Opi5Camera() = default;

Opi5Camera::~Opi5Camera() {
    close();
}

bool Opi5Camera::open() {
    // Try common V4L2 device paths
    // On RK3588, the ISP output is typically /dev/video0 or discoverable
    const char* candidates[] = {
        "/dev/video0", "/dev/video1", "/dev/video2", "/dev/video3"
    };

    for (const char* path : candidates) {
        struct stat st;
        if (stat(path, &st) != 0 || !S_ISCHR(st.st_mode)) continue;

        int fd = ::open(path, O_RDWR | O_NONBLOCK);
        if (fd < 0) continue;

        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
            if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) &&
                (cap.capabilities & V4L2_CAP_STREAMING)) {
                fd_ = fd;
                device_path_ = path;
                std::cout << "Opened camera: " << path << " ("
                          << cap.card << ")" << std::endl;
                return initV4l2();
            }
        }
        ::close(fd);
    }

    std::cerr << "No suitable V4L2 camera found" << std::endl;
    return false;
}

bool Opi5Camera::initV4l2() {
    // Set format: OV9281 outputs Y8 (8-bit mono) at 1280x800
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width_;
    fmt.fmt.pix.height = height_;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;  // Y8 monochrome
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        // Try YUYV as fallback (color cameras)
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
            std::cerr << "Cannot set video format" << std::endl;
            return false;
        }
    }

    // Update actual dimensions (driver may adjust)
    width_ = fmt.fmt.pix.width;
    height_ = fmt.fmt.pix.height;

    // Set frame rate
    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = fps_;
    ioctl(fd_, VIDIOC_S_PARM, &parm);

    return initMmap();
}

bool Opi5Camera::initMmap() {
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = NUM_BUFFERS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        std::cerr << "VIDIOC_REQBUFS failed" << std::endl;
        return false;
    }

    num_buffers_ = req.count;

    for (int i = 0; i < num_buffers_; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            std::cerr << "VIDIOC_QUERYBUF failed for buffer " << i << std::endl;
            return false;
        }

        buffers_[i].length = buf.length;
        buffers_[i].start = mmap(nullptr, buf.length,
                                  PROT_READ | PROT_WRITE, MAP_SHARED,
                                  fd_, buf.m.offset);

        if (buffers_[i].start == MAP_FAILED) {
            std::cerr << "mmap failed for buffer " << i << std::endl;
            return false;
        }
    }

    return true;
}

void Opi5Camera::cleanupMmap() {
    for (int i = 0; i < num_buffers_; i++) {
        if (buffers_[i].start && buffers_[i].start != MAP_FAILED) {
            munmap(buffers_[i].start, buffers_[i].length);
            buffers_[i].start = nullptr;
        }
    }
    num_buffers_ = 0;
}

void Opi5Camera::close() {
    stopCapture();
    cleanupMmap();
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool Opi5Camera::isOpen() const {
    return fd_ >= 0;
}

bool Opi5Camera::startCapture() {
    if (capturing_) return true;

    // Queue all buffers
    for (int i = 0; i < num_buffers_; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            std::cerr << "VIDIOC_QBUF failed for buffer " << i << std::endl;
            return false;
        }
    }

    // Start streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        std::cerr << "VIDIOC_STREAMON failed" << std::endl;
        return false;
    }

    capturing_ = true;
    return true;
}

void Opi5Camera::stopCapture() {
    if (!capturing_) return;

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd_, VIDIOC_STREAMOFF, &type);
    capturing_ = false;
}

cv::Mat Opi5Camera::getFrame(int timeout_ms) {
    if (!capturing_) return cv::Mat();

    struct pollfd pfd;
    pfd.fd = fd_;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, timeout_ms);
    if (ret <= 0) return cv::Mat();  // Timeout or error

    // Dequeue buffer
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        return cv::Mat();
    }

    // Create cv::Mat from buffer (Y8 mono -> CV_8UC1)
    // Clone to own the data before re-queuing the buffer
    cv::Mat frame(height_, width_, CV_8UC1, buffers_[buf.index].start);
    cv::Mat result = frame.clone();

    // Re-queue buffer
    ioctl(fd_, VIDIOC_QBUF, &buf);

    return result;
}

bool Opi5Camera::setV4l2Control(uint32_t id, int32_t value) {
    struct v4l2_control ctrl;
    ctrl.id = id;
    ctrl.value = value;
    return ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == 0;
}

int32_t Opi5Camera::getV4l2Control(uint32_t id) {
    struct v4l2_control ctrl;
    ctrl.id = id;
    if (ioctl(fd_, VIDIOC_G_CTRL, &ctrl) < 0) return -1;
    return ctrl.value;
}

void Opi5Camera::setControls(const CameraControls& controls) {
    controls_ = controls;

    if (fd_ < 0) return;

    // Map Limelight exposure (0-480 range in .vpr) to V4L2 exposure
    setV4l2Control(V4L2_CID_EXPOSURE, static_cast<int32_t>(controls.exposure));

    // Map gain
    setV4l2Control(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(controls.gain));

    // Flip
    if (controls.flip & 1) setV4l2Control(V4L2_CID_HFLIP, 1);
    if (controls.flip & 2) setV4l2Control(V4L2_CID_VFLIP, 1);

    // Note: red_balance and blue_balance are ISP white balance controls.
    // For the OV9281 (monochrome), these are not applicable.
    // For color cameras, map to V4L2_CID_RED_BALANCE / V4L2_CID_BLUE_BALANCE
}

CameraControls Opi5Camera::getControls() const {
    return controls_;
}

CameraInfo Opi5Camera::getInfo() const {
    return CameraInfo{
        .width = width_,
        .height = height_,
        .fps = fps_,
        .sensor_name = "OV9281",
        .driver = "v4l2-rkisp"
    };
}

} // namespace limelight::hal
