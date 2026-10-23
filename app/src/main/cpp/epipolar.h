#pragma once
// Mô hình epipolar cho flow giữa hai ảnh khi tâm ống kính dịch đi (xoay người lúc chụp).

#include "common.h"

namespace sphere {

// Hướng (vector đơn vị) của điểm (gx, gy) trên khung equirect có s pixel/rad, cùng quy ước với warpSphere.
inline Vec3d panoDir(double gx, double gy, double s) {
    double th = gx / s, ph = std::max(0.0, std::min(CV_PI, gy / s));
    double sp = std::sin(ph);
    return Vec3d(sp * std::sin(th), -std::cos(ph), sp * std::cos(th));
}

// Đổi vector tiếp tuyến w (rad) tại hướng u thành độ dịch pixel trên khung equirect.
inline Vec2d tangentToPx(const Vec3d &u, const Vec3d &w, double s) {
    double xz = u[0] * u[0] + u[2] * u[2];
    if (xz < 1e-9) return Vec2d(0, 0);
    return Vec2d(s * (u[2] * w[0] - u[0] * w[2]) / xz, s * w[1] / std::sqrt(xz));
}

struct FlowSample {
    Vec3d u, v;   // hướng điểm ở ảnh p và hướng nó xuất hiện ở ảnh q
    Vec2f f;      // flow (pixel khung flow)
    float conf;
    int x, y;     // toạ độ trong vùng chồng
};

// Hướng dịch chuyển b giữa hai tâm ống kính và phần xoay dư omega của một cặp ảnh.
struct EpiModel {
    Vec3d b, omega;
    bool ok = false;
    int moving = 0, inliers = 0;
};

// Khoảng cách góc (rad) từ v (đã bỏ phần xoay dư) tới đường tròn lớn qua u và b.
inline double epiResidual(const Vec3d &b, const Vec3d &omega, const Vec3d &u, const Vec3d &v) {
    double bu = norm(b.cross(u));
    if (bu < 1e-6) return 1e9;
    return std::abs(b.dot(u.cross(v - omega.cross(u)))) / bu;
}

// RANSAC 2 điểm cho hướng dịch chuyển b, rồi tinh chỉnh xen kẽ phần xoay dư omega và b.
EpiModel fitEpipolar(const std::vector<FlowSample> &S, double s, RNG &rng);

// Flow (pixel) theo mô hình; perpPx = phần flow đo được nằm ngoài mô hình.
Vec2f constrainFlow(const EpiModel &M, const FlowSample &S, double s, float &perpPx);

}  // namespace sphere
