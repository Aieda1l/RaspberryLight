cmake_minimum_required(VERSION 3.20)
project(limelight-vision VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Platform selection: RPI or OPI5
set(LIMELIGHT_PLATFORM "OPI5" CACHE STRING "Target platform: RPI or OPI5")
set_property(CACHE LIMELIGHT_PLATFORM PROPERTY STRINGS "RPI" "OPI5")

message(STATUS "Building for platform: ${LIMELIGHT_PLATFORM}")

# Common dependencies
find_package(PkgConfig REQUIRED)
find_package(OpenCV 4.10 REQUIRED COMPONENTS
    core imgproc imgcodecs videoio video calib3d features2d
    dnn objdetect aruco highgui
)
find_package(Python3 3.11 REQUIRED COMPONENTS Development)
find_package(Threads REQUIRED)

pkg_check_modules(SPDLOG REQUIRED spdlog)
pkg_check_modules(APRILTAG REQUIRED apriltag)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)
pkg_check_modules(GPIOD REQUIRED libgpiod libgpiodcxx)
pkg_check_modules(PROTOBUF REQUIRED protobuf)
pkg_check_modules(GSTREAMER REQUIRED
    gstreamer-1.0
    gstreamer-app-1.0
    gstreamer-video-1.0
)

# ZXing (may not have pkg-config)
find_library(ZXING_LIB ZXing REQUIRED)
find_path(ZXING_INCLUDE ZXing/ReadBarcode.h)

# WPILib NetworkTables
find_library(NTCORE_LIB ntcore REQUIRED)
find_library(WPINET_LIB wpinet REQUIRED)
find_library(WPIUTIL_LIB wpiutil REQUIRED)
find_path(WPILIB_INCLUDE ntcore.h PATH_SUFFIXES wpilib)

# JSON (header-only, bundled)
include(FetchContent)
FetchContent_Declare(json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(json)

# Platform-specific dependencies
if(LIMELIGHT_PLATFORM STREQUAL "RPI")
    pkg_check_modules(LIBCAMERA REQUIRED libcamera)
    find_library(RPICAM_APP_LIB camera_app HINTS /usr/lib/aarch64-linux-gnu)
    find_library(HAILORT_LIB hailort)
    find_library(EDGETPU_LIB edgetpu)
    add_compile_definitions(PLATFORM_RPI)
elseif(LIMELIGHT_PLATFORM STREQUAL "OPI5")
    pkg_check_modules(V4L2 REQUIRED libv4l2)
    find_library(RKNN_LIB rknnrt HINTS /usr/lib)
    find_library(HAILORT_LIB hailort)  # Optional M.2
    find_library(EDGETPU_LIB edgetpu)  # Optional Coral USB
    add_compile_definitions(PLATFORM_OPI5)
endif()

# TFLite (always available as CPU fallback)
find_library(TFLITE_LIB tensorflow-lite)

# Optional backends
if(HAILORT_LIB)
    add_compile_definitions(HAS_HAILO)
    message(STATUS "Hailo backend: ENABLED")
else()
    message(STATUS "Hailo backend: disabled (libhailort not found)")
endif()

if(EDGETPU_LIB)
    add_compile_definitions(HAS_EDGETPU)
    message(STATUS "EdgeTPU backend: ENABLED")
else()
    message(STATUS "EdgeTPU backend: disabled (libedgetpu not found)")
endif()

if(RKNN_LIB)
    add_compile_definitions(HAS_RKNN)
    message(STATUS "RKNN backend: ENABLED")
else()
    message(STATUS "RKNN backend: disabled (librknnrt not found)")
endif()

if(TFLITE_LIB)
    add_compile_definitions(HAS_TFLITE)
    message(STATUS "TFLite backend: ENABLED")
else()
    message(STATUS "TFLite backend: disabled (libtensorflow-lite not found)")
endif()

# Common library (config parsing, logging, shared types)
add_library(limelight-common STATIC
    src/common/config.cpp
    src/common/pipeline_config.cpp
    src/common/field_map.cpp
    src/common/ws_crypto.cpp
)
target_include_directories(limelight-common PUBLIC
    src/common/include
    ${SPDLOG_INCLUDE_DIRS}
)
target_link_libraries(limelight-common PUBLIC
    nlohmann_json::nlohmann_json
    ${SPDLOG_LIBRARIES}
)

# HAL library
set(HAL_SOURCES src/hal/hal_factory.cpp)
if(LIMELIGHT_PLATFORM STREQUAL "RPI")
    list(APPEND HAL_SOURCES
        src/hal/platforms/rpi/rpi_camera.cpp
        src/hal/platforms/rpi/rpi_gpio.cpp
        src/hal/platforms/rpi/rpi_thermal.cpp
        src/hal/platforms/rpi/rpi_npu.cpp
    )
elseif(LIMELIGHT_PLATFORM STREQUAL "OPI5")
    list(APPEND HAL_SOURCES
        src/hal/platforms/opi5/opi5_camera.cpp
        src/hal/platforms/opi5/opi5_gpio.cpp
        src/hal/platforms/opi5/opi5_thermal.cpp
        src/hal/platforms/opi5/opi5_npu.cpp
    )
endif()

add_library(limelight-hal STATIC ${HAL_SOURCES})
target_include_directories(limelight-hal PUBLIC
    src/hal/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(limelight-hal PUBLIC
    limelight-common
    ${OpenCV_LIBS}
    ${GPIOD_LIBRARIES}
    Threads::Threads
)

if(LIMELIGHT_PLATFORM STREQUAL "RPI")
    target_link_libraries(limelight-hal PUBLIC ${LIBCAMERA_LIBRARIES})
    if(RPICAM_APP_LIB)
        target_link_libraries(limelight-hal PUBLIC ${RPICAM_APP_LIB})
    endif()
elseif(LIMELIGHT_PLATFORM STREQUAL "OPI5")
    target_link_libraries(limelight-hal PUBLIC ${V4L2_LIBRARIES})
    if(RKNN_LIB)
        target_link_libraries(limelight-hal PUBLIC ${RKNN_LIB})
    endif()
endif()

# Vision Server
add_executable(visionserver
    src/visionserver/main.cpp
    src/visionserver/pipeline.cpp
    src/visionserver/pipeline_manager.cpp
    src/visionserver/fiducial_pipeline.cpp
    src/visionserver/color_pipeline.cpp
    src/visionserver/detector_pipeline.cpp
    src/visionserver/classifier_pipeline.cpp
    src/visionserver/depth_pipeline.cpp
    src/visionserver/python_pipeline.cpp
    src/visionserver/barcode_pipeline.cpp
    src/visionserver/viewfinder_pipeline.cpp
    src/visionserver/calibration_pipeline.cpp
    src/visionserver/stream_server.cpp
    src/visionserver/websocket_server.cpp
    src/visionserver/nt_publisher.cpp
    src/visionserver/modbus_server.cpp
    src/visionserver/rest_api.cpp
    src/visionserver/http_server.cpp
    src/visionserver/inference_engine.cpp
    src/visionserver/python_runtime.cpp
)
target_include_directories(visionserver PRIVATE
    src/visionserver/include
    ${WPILIB_INCLUDE}
    ${ZXING_INCLUDE}
    ${GSTREAMER_INCLUDE_DIRS}
    ${APRILTAG_INCLUDE_DIRS}
)
target_link_libraries(visionserver PRIVATE
    limelight-common
    limelight-hal
    ${OpenCV_LIBS}
    ${APRILTAG_LIBRARIES}
    ${ZXING_LIB}
    ${NTCORE_LIB}
    ${WPINET_LIB}
    ${WPIUTIL_LIB}
    ${LIBMODBUS_LIBRARIES}
    ${GSTREAMER_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    Python3::Python
    Threads::Threads
)
if(TFLITE_LIB)
    target_link_libraries(visionserver PRIVATE ${TFLITE_LIB})
endif()
if(HAILORT_LIB)
    target_link_libraries(visionserver PRIVATE ${HAILORT_LIB})
endif()
if(EDGETPU_LIB)
    target_link_libraries(visionserver PRIVATE ${EDGETPU_LIB})
endif()

# Network Responder
add_executable(networkresponder
    src/networkresponder/main.cpp
    src/networkresponder/responder.cpp
)
target_link_libraries(networkresponder PRIVATE
    limelight-common
    Threads::Threads
)

# Hardware Monitor
#
# Despite the name, the original binary is not a thermal/fan loop — it's an
# HTTP/WebSocket API server on TCP/4802 with 21 endpoints used by the
# Limelight web UI (see FUN_00123c40 in hwmon_decompiled.c). It links
# against nlohmann::json but NOT libgpiod or libhal.
add_executable(hwmon
    src/hwmon/main.cpp
    src/hwmon/monitor.cpp
    src/hwmon/http_server.cpp
    src/hwmon/util.cpp
    src/hwmon/endpoints_disk.cpp
    src/hwmon/endpoints_kernel.cpp
    src/hwmon/endpoints_logs.cpp
    src/hwmon/endpoints_system.cpp
    src/hwmon/log_stream.cpp
)
target_link_libraries(hwmon PRIVATE
    limelight-common
    nlohmann_json::nlohmann_json
    Threads::Threads
)

# USB Gadget Configurator
add_executable(gadget
    src/gadget/main.cpp
    src/gadget/usb_gadget.cpp
)
target_link_libraries(gadget PRIVATE
    limelight-common
)

# Install targets
install(TARGETS visionserver DESTINATION /usr/local/bin/visionserver)
install(TARGETS networkresponder DESTINATION /usr/local/bin/visionserver)
install(TARGETS hwmon DESTINATION /usr/local/bin/hwmon)
install(TARGETS gadget DESTINATION /usr/local/bin/gadget)
