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

// Ghép các ảnh thành ảnh cầu equirect rộng outWidth và ghi ra outPath. Trả về chuỗi rỗng nếu thành công,
// ngược lại là thông báo lỗi.
std::string stitch(const std::vector<std::string> &paths, const std::vector<Mat> &sensorR,
                   double focalFull, int outWidth, int jpegQuality, bool deghost, const std::string &outPath,
                   const Progress &progress);

}  // namespace sphere
