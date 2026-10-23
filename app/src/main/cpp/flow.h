#pragma once
// Trường uốn từ optical flow ở các vùng chồng (giảm bóng mờ do thị sai).

#include "warp.h"

namespace sphere {

std::vector<Field> computeFlowFields(const std::vector<Instance> &inst, const std::vector<Mat> &warped,
                                     const std::vector<Mat> &masks, const std::vector<Point> &corners,
                                     int width);

}  // namespace sphere
