#pragma once
// Toàn bộ quy trình ghép, không phụ thuộc JNI.

#include "common.h"

namespace sphere {

// Báo tiến độ (phần trăm, tên bước) về phía gọi.
struct Progress {
    std::function<void(int, const char *)> fn;

    void report(int percent, const char *stage) const {
        if (fn) fn(percent, stage);
    }
};

// Khung cầu nối của một lượt chụp (xem bridge.h). Rỗng nếu lượt chụp không có.
struct BridgeInput {
    std::vector<std::string> paths;
    std::vector<Mat> sensorR;  // camera -> toàn cảnh theo cảm biến, CV_32F
    std::vector<int> after;    // chỉ số tấm chụp ngay trước khung này
    double focal = 0;          // tiêu cự (pixel) của khung cầu nối
    int rotationCw = 90;       // xoay theo chiều kim đồng hồ bấy nhiêu độ thì ảnh đứng
};

// Ghép các ảnh thành ảnh cầu equirect rộng outWidth và ghi ra outPath. Trả về chuỗi rỗng nếu thành công,
// ngược lại là thông báo lỗi.
std::string stitch(const std::vector<std::string> &paths, const std::vector<Mat> &sensorR,
                   double focalFull, const BridgeInput &bridges, int outWidth, int jpegQuality, bool deghost,
                   const std::string &outPath, const Progress &progress);

}  // namespace sphere
