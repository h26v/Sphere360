#pragma once
// Đo chất lượng ghép: độ lệch còn lại giữa các ảnh sau khi căn góc và uốn, để so các cách làm bằng số.

#include "warp.h"

namespace sphere {

struct ResidualStats {
    double overlapDeg = 0;  // trung bình độ lệch ở mọi chỗ có cạnh trong vùng chồng (độ)
    double seamDeg = 0;     // trung bình độ lệch ở chỗ có cạnh nằm trên đường nối (độ): cái mắt thấy
    double seamP90Deg = 0;
    double seamBroken = 0;  // tỉ lệ điểm cạnh trên đường nối mà flow hai chiều không khớp (lệch quá tầm, che khuất)
    int seamPx = 0;
};

// Nắn lại từng bản ảnh (ảnh nhỏ cỡ khung flow, ma trận nội tại K) theo phép xoay R và trường uốn của nó, rồi ở
// mỗi vùng chồng đo flow còn lại giữa hai ảnh. Chỉ lấy thành phần vuông góc với cạnh (thành phần dọc cạnh không
// đo được và cũng không lộ). Đường nối lấy từ mặt nạ đường nối ở khung rộng seamWidth.
ResidualStats measureResidual(const std::vector<Instance> &inst, const std::vector<Mat> &images,
                              const std::vector<Mat> &K, const std::vector<Mat> &R, const std::vector<Field> &fields,
                              int width, const std::vector<UMat> &seamMasks, const std::vector<Point> &seamCorners,
                              int seamWidth, const std::vector<Mat> *srcMasks = nullptr);

}  // namespace sphere
