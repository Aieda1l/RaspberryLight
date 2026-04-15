# RaspberryLight

RaspberryLight is a high-performance vision server and coprocessor framework designed for FRC (FIRST Robotics Competition). Inspired by the Limelight smart camera, it is built to run on readily available SBCs (Single Board Computers) like the Orange Pi 5 and Raspberry Pi lineups.

## Overview

At its core, RaspberryLight recreates the familiar REST API, NetworkTables interfaces, and hardware-accelerated computer vision pipelines found in elite FRC vision systems, making it possible to deploy high-throughput AprilTag, object detection, and color blob tracking to your own hardware. 

### Key Features

* **Hardware-Accelerated Inference**: Dispatches neural network workloads to the best available backend at runtime:
  * Rockchip NPU (RKNN) specifically tailored for the Orange Pi 5.
  * Hailo AI accelerators.
  * Google Coral EdgeTPU.
  * TensorFlow Lite CPU fallback for universal compatibility.
* **First-Class NetworkTables Integration**: Seamlessly syncs 3D targeting data, poses, and diagnostics back to the RoboRIO.
* **REST & Streaming API**: Standardized JSON-based configuration management and MJPEG video streaming server.
* **Modular Pipelines**: Built-in support for fiducial tracking (AprilTags), deep-learning object detectors/classifiers, and standard color thresholds.

## Directory Structure
* `src/visionserver/` - The core HTTP daemon, NT publisher, pipeline manager, and inference dispatch routing.
* `src/hal/` - Abstractions for specific Neural Processing Units and camera interfaces.
* `web/` - Front-end assets / UI to match the hardware pipeline logic.

## Building and Running
This project utilizes CMake. From the project root, general build instructions are as follows:

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

Make sure your specific SBC has the correct dependencies installed (OpenCV, ntcore, TFLite/RKNN/EdgeTPU SDKs) before compiling. 
