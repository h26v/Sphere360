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

#define LOGI(...) ::sphere::logLine(ANDROID_LOG_INFO, __VA_ARGS__)
#define LOGW(...) ::sphere::logLine(ANDROID_LOG_WARN, __VA_ARGS__)

namespace sphere {
// Ghi một dòng log ra logcat (thẻ SphereStitcher), và ra file nếu đã đặt bằng setLogFile.
void logLine(int priority, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
// Đặt file nhận thêm log (ghi đè); chuỗi rỗng thì thôi ghi file.
void setLogFile(const std::string &path);
// Tiền tố đường dẫn cho ảnh debug (vd ".../t1"); rỗng thì không ghi. Đặt trong stitch() khi kDebugSeams.
inline std::string gDebugStem;

// Toàn bộ mã ghép nằm trong namespace sphere nên dùng thẳng tên của OpenCV cho gọn.
using namespace cv;
using namespace cv::detail;
}  // namespace sphere
