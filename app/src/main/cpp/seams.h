#pragma once
// Đường nối quanh đỉnh/đáy.

#include "warp.h"

namespace sphere {

// Giao vùng quanh đỉnh/đáy cho tấm chụp thẳng lên/xuống.
void assignPoleCaps(const std::vector<Matx33d> &R, const std::vector<Instance> &inst,
                    const std::vector<Mat> &valid, const std::vector<Point> &corners,
                    std::vector<UMat> &seamMasks, double s);

}  // namespace sphere
