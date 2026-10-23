#pragma once
// Trường uốn từ optical flow ở các vùng chồng (giảm bóng mờ do thị sai).

#include "bridge.h"
#include "warp.h"

namespace sphere {

// Thị sai đo được ở vùng chồng của một cặp bản ảnh, đã đổi sang khung đường nối.
struct PairParallax {
    int p, q;  // chỉ số bản ảnh (Instance)
    Rect roi;  // vị trí trên khung đường nối
    Mat deg;   // CV_32F, độ lệch giữa hai ảnh (độ)
};

// Trường uốn cho từng bản ảnh. Nếu parallax khác null thì ghi thêm bản đồ thị sai của từng cặp, thu nhỏ theo
// tỉ lệ seamRatio (khung đường nối / khung flow). Nếu có khung cầu nối thì hai tấm chụp liền nhau được dò flow
// qua chuỗi khung đó.
std::vector<Field> computeFlowFields(const std::vector<Instance> &inst, const std::vector<Mat> &warped,
                                     const std::vector<Mat> &masks, const std::vector<Point> &corners,
                                     int width, double seamRatio = 0,
                                     std::vector<PairParallax> *parallax = nullptr,
                                     const BridgeChains *bridges = nullptr);

}  // namespace sphere
