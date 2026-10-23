#pragma once
// Phần dùng chung: OpenCV, thư viện chuẩn, ghi log.

#include <android/log.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#define LOG_TAG "SphereStitcher"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)

namespace sphere {
// Toàn bộ mã ghép nằm trong namespace sphere nên dùng thẳng tên của OpenCV cho gọn.
using namespace cv;
using namespace cv::detail;
}  // namespace sphere
