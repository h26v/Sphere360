#pragma once
// Căn góc: điểm khớp giữa các cặp ảnh và tinh chỉnh hướng của mọi tấm.

#include "common.h"

namespace sphere {

struct RayPair {
    int i, j;
    Vec3d ui, uj;  // tia trong hệ camera i và j
};

// Phép đo góc tương đối giữa hai tấm: A ≈ R_i^T R_j (tia trong hệ j đổi sang hệ i). weight 0..1.
struct RelRotation {
    int i, j;
    Matx33d A;
    double weight;
};

// RANSAC phép xoay cho từng cặp ảnh, bỏ cặp lệch xa cảm biến; trả về các cặp tia đã lọc. Nếu pairRots khác null
// thì ghi thêm phép xoay của từng cặp (Kabsch trên các điểm khớp đúng).
std::vector<RayPair> collectRayPairs(const std::vector<ImageFeatures> &features,
                                     const std::vector<MatchesInfo> &pairwise,
                                     const std::vector<Matx33d> &sensor, const Matx33d &Kinv,
                                     double focalWork, int &pairsUsed,
                                     std::vector<RelRotation> *pairRots = nullptr);

// Phép xoay A (a ≈ A b) từ hai tập tia tương ứng: RANSAC 2 điểm rồi Kabsch trên điểm khớp đúng. false nếu ít hơn
// minInliers điểm khớp đúng.
bool robustRotation(const std::vector<Vec3d> &a, const std::vector<Vec3d> &b, double inlierDeg, int minInliers,
                    RNG &rng, Matx33d &A, int &inliers);

// Căn góc mới: trung bình phép xoay có chống nhiễu (Cauchy) trên các phép đo của cặp ảnh và chuỗi khung cầu nối,
// neo theo góc con quay giữa hai tấm chụp liền nhau, trọng lực và (lỏng) hướng ngang.
std::vector<Matx33d> averageRotations(const std::vector<RelRotation> &pairs, const std::vector<RelRotation> &chains,
                                      const std::vector<Matx33d> &sensor);

// Ảnh có độ hiệu chỉnh so với cảm biến lệch xa mốc chung của mọi ảnh thì đặt lại theo cảm biến.
void resetDeviating(std::vector<Matx33d> &R, const std::vector<Matx33d> &sensor);

// Tinh chỉnh góc của mọi tấm cùng lúc (Levenberg–Marquardt, Huber, neo theo cảm biến).
std::vector<Matx33d> refineRotations(const std::vector<RayPair> &obs, const std::vector<Matx33d> &sensor,
                                     double focal, double huber);

}  // namespace sphere
