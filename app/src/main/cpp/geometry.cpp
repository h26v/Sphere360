#include "geometry.h"

namespace sphere {

double angleBetweenDeg(const Vec3d &a, const Vec3d &b) {
    return std::atan2(norm(a.cross(b)), a.dot(b)) * 180.0 / CV_PI;
}

double rotationAngleDeg(const Matx33d &a, const Matx33d &b) {
    Matx33d d = a.t() * b;
    double c = (d(0, 0) + d(1, 1) + d(2, 2) - 1.0) / 2.0;
    return std::acos(std::max(-1.0, std::min(1.0, c))) * 180.0 / CV_PI;
}

Matx33d expRot(const Vec3d &w) {
    Matx33d R;
    Rodrigues(w, R);
    return R;
}

Vec3d logRot(const Matx33d &R) {
    Vec3d w;
    Rodrigues(R, w);
    return w;
}

Matx33d skew(const Vec3d &v) {
    return Matx33d(0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0);
}

Vec3d opticalAxis(const Matx33d &R) {
    return Vec3d(R(0, 2), R(1, 2), R(2, 2));
}

// Phép xoay A tốt nhất sao cho a_k ≈ A * b_k (Kabsch).
Matx33d kabsch(const std::vector<Vec3d> &a, const std::vector<Vec3d> &b) {
    Matx33d H = Matx33d::zeros();
    for (size_t k = 0; k < a.size(); ++k) H += b[k] * a[k].t();
    Mat w, u, vt;
    SVD::compute(Mat(H), w, u, vt);
    Mat A = vt.t() * u.t();
    if (determinant(A) < 0) {
        Mat D = Mat::eye(3, 3, CV_64F);
        D.at<double>(2, 2) = -1;
        A = vt.t() * D * u.t();
    }
    Matx33d out;
    A.copyTo(Mat(out, false));
    return out;
}

// Phép xoay A sao cho a ≈ A b, dựng từ 2 cặp tia (mẫu tối thiểu cho RANSAC).
bool rotationFrom2(const Vec3d &a1, const Vec3d &a2, const Vec3d &b1, const Vec3d &b2, Matx33d &A) {
    Vec3d ac = a1.cross(a2), bc = b1.cross(b2);
    double na = norm(ac), nb = norm(bc);
    if (na < 1e-3 || nb < 1e-3) return false;
    ac *= 1.0 / na;
    bc *= 1.0 / nb;
    Vec3d a3 = a1.cross(ac), b3 = b1.cross(bc);
    Matx33d Ma(a1[0], ac[0], a3[0], a1[1], ac[1], a3[1], a1[2], ac[2], a3[2]);
    Matx33d Mb(b1[0], bc[0], b3[0], b1[1], bc[1], b3[1], b1[2], bc[2], b3[2]);
    A = Ma * Mb.t();
    return true;
}

Mat intrinsics(double focal, Size size) {
    Mat K = (Mat_<float>(3, 3) << focal, 0, size.width / 2.0, 0, focal, size.height / 2.0, 0, 0, 1);
    return K;
}

}  // namespace sphere
