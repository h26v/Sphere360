#pragma once
// Căn góc: điểm khớp giữa các cặp ảnh và tinh chỉnh hướng của mọi tấm.

#include "common.h"

namespace sphere {

struct RayPair {
    int i, j;
    Vec3d ui, uj;  // tia trong hệ camera i và j
};

// RANSAC phép xoay cho từng cặp ảnh, bỏ cặp lệch xa cảm biến; trả về các cặp tia đã lọc.
std::vector<RayPair> collectRayPairs(const std::vector<ImageFeatures> &features,
                                     const std::vector<MatchesInfo> &pairwise,
                                     const std::vector<Matx33d> &sensor, const Matx33d &Kinv,
                                     double focalWork, int &pairsUsed);

// Tinh chỉnh góc của mọi tấm cùng lúc (Levenberg–Marquardt, Huber, neo theo cảm biến).
std::vector<Matx33d> refineRotations(const std::vector<RayPair> &obs, const std::vector<Matx33d> &sensor,
                                     double focal, double huber);

}  // namespace sphere
