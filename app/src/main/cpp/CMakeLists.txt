cmake_minimum_required(VERSION 3.22)
project(sphere360 CXX)

# OpenCV Android SDK, liên kết tĩnh, chỉ lấy các module cần cho việc ghép ảnh.
# Đường dẫn do Gradle truyền vào từ local.properties (opencv.sdk=...).
if(NOT OPENCV_ANDROID_SDK)
    message(FATAL_ERROR "Chưa đặt opencv.sdk trong local.properties (thư mục OpenCV-android-sdk).")
endif()
set(OPENCV_SDK ${OPENCV_ANDROID_SDK}/sdk/native)
set(OPENCV_LIBS ${OPENCV_SDK}/staticlibs/${ANDROID_ABI})
set(OPENCV_3RD ${OPENCV_SDK}/3rdparty/libs/${ANDROID_ABI})

add_library(sphere_stitcher SHARED
    alignment.cpp
    bridge.cpp
    epipolar.cpp
    flow.cpp
    geometry.cpp
    jni_bridge.cpp
    log.cpp
    params.cpp
    quality.cpp
    seams.cpp
    stitcher.cpp
    warp.cpp)
target_include_directories(sphere_stitcher PRIVATE ${OPENCV_SDK}/jni/include)

file(GLOB OPENCV_3RD_LIBS ${OPENCV_3RD}/*.a)
target_link_libraries(sphere_stitcher
    -Wl,--start-group
    ${OPENCV_LIBS}/libopencv_stitching.a
    ${OPENCV_LIBS}/libopencv_video.a
    ${OPENCV_LIBS}/libopencv_calib3d.a
    ${OPENCV_LIBS}/libopencv_features2d.a
    ${OPENCV_LIBS}/libopencv_flann.a
    ${OPENCV_LIBS}/libopencv_imgcodecs.a
    ${OPENCV_LIBS}/libopencv_imgproc.a
    ${OPENCV_LIBS}/libopencv_core.a
    ${OPENCV_3RD_LIBS}
    -Wl,--end-group
    z log android dl m)
