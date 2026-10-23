#pragma once
// Phép xoay, góc và ma trận nội tại camera.

#include "common.h"

namespace sphere {

double angleBetweenDeg(const Vec3d &a, const Vec3d &b);
double rotationAngleDeg(const Matx33d &a, const Matx33d &b);
Matx33d expRot(const Vec3d &w);
Vec3d logRot(const Matx33d &R);
Matx33d skew(const Vec3d &v);
Vec3d opticalAxis(const Matx33d &R);
// Phép xoay A tốt nhất sao cho a_k ≈ A * b_k (Kabsch).
Matx33d kabsch(const std::vector<Vec3d> &a, const std::vector<Vec3d> &b);
// Phép xoay A sao cho a ≈ A b, dựng từ 2 cặp tia (mẫu tối thiểu cho RANSAC).
bool rotationFrom2(const Vec3d &a1, const Vec3d &a2, const Vec3d &b1, const Vec3d &b2, Matx33d &A);
Mat intrinsics(double focal, Size size);

}  // namespace sphere
