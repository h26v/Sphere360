#include "warp.h"

#include "params.h"

namespace sphere {

// Nắn một ảnh lên mặt cầu (u = s*atan2(x, z), v = s*(pi - acos(y))). Vùng ra được xác định đúng cả
// khi ảnh chứa đỉnh/đáy hoặc vắt qua kinh tuyến 180°. Nếu có trường uốn, điểm (u, v) của khung lấy
// màu từ hướng (u + dx, v + dy) — tức ảnh được uốn cho khớp với ảnh bên cạnh.
Point warpSphere(const Mat &src, const Mat &K, const Mat &R, double s, Mat &dstImg, Mat &dstMask,
                 const Field *field, double fieldShiftPx) {
    Matx33d Kd, Rd;
    K.convertTo(Mat(Kd, false), CV_64F);
    R.convertTo(Mat(Rd, false), CV_64F);
    const Matx33d toPano = Rd * Kd.inv();
    const Matx33d toCam = Rd.t();
    const double halfW = CV_PI * s;
    const double fullH = CV_PI * s;
    const double canvasW = 2 * CV_PI * s;

    double umin = 1e30, umax = -1e30, vmin = 1e30, vmax = -1e30;
    bool wrap = false;
    double prevU = 0;
    bool havePrev = false;
    auto addBorder = [&](double x, double y) {
        Vec3d d = toPano * Vec3d(x, y, 1);
        double r = norm(d);
        double u = s * std::atan2(d[0], d[2]);
        double v = s * (CV_PI - std::acos(std::max(-1.0, std::min(1.0, d[1] / r))));
        if (havePrev && std::abs(u - prevU) > halfW) wrap = true;
        prevU = u;
        havePrev = true;
        umin = std::min(umin, u); umax = std::max(umax, u);
        vmin = std::min(vmin, v); vmax = std::max(vmax, v);
    };
    const int N = 128;
    const double W = src.cols, H = src.rows;
    for (int k = 0; k <= N; ++k) addBorder(W * k / N, 0);
    for (int k = 0; k <= N; ++k) addBorder(W, H * k / N);
    for (int k = 0; k <= N; ++k) addBorder(W * (N - k) / N, H);
    for (int k = 0; k <= N; ++k) addBorder(0, H * (N - k) / N);

    auto poleInside = [&](double py) {
        Vec3d c = toCam * Vec3d(0, py, 0);
        if (c[2] <= 1e-6) return false;
        Vec3d p = Kd * (c * (1.0 / c[2]));
        return p[0] >= 0 && p[0] < W && p[1] >= 0 && p[1] < H;
    };
    if (poleInside(-1)) { vmin = 0; wrap = true; }
    if (poleInside(1)) { vmax = fullH; wrap = true; }
    if (wrap) { umin = -halfW; umax = halfW; }
    // Chừa thêm chỗ cho phần ảnh bị uốn ra ngoài.
    if (field != nullptr && !field->empty() && !wrap) {
        // Chừa thêm một góc cố định (không phải số pixel) để mọi độ phân giải cắt cùng một vùng, nếu không
        // đường nối tính ở khung nhỏ có thể giao cho ảnh những điểm mà ở khung lớn ảnh không phủ tới.
        double pad = field->maxPx * (canvasW / field->width) + s * (1.5 * CV_PI / 180.0);
        umin -= pad; umax += pad; vmin -= pad; vmax += pad;
    }

    int x0 = std::max((int) std::floor(umin), (int) std::floor(-halfW));
    int x1 = std::min((int) std::ceil(umax) + 1, (int) std::ceil(halfW));
    int y0 = std::max((int) std::floor(vmin), 0);
    int y1 = std::min((int) std::ceil(vmax) + 1, (int) std::ceil(fullH));
    if (x1 <= x0 || y1 <= y0) {
        dstImg.release();
        dstMask.release();
        return Point(0, 0);
    }
    const int cols = x1 - x0, rows = y1 - y0;
    dstImg.create(rows, cols, src.type());
    dstMask.create(rows, cols, CV_8U);
    Mat ones(src.size(), CV_8U, Scalar::all(255));
    const bool useField = field != nullptr && !field->empty();

    const int strip = 256;
    for (int r0 = 0; r0 < rows; r0 += strip) {
        int r1 = std::min(rows, r0 + strip);
        Mat xmap(r1 - r0, cols, CV_32F), ymap(r1 - r0, cols, CV_32F);
        parallel_for_(Range(r0, r1), [&](const Range &range) {
            for (int r = range.start; r < range.end; ++r) {
                float *xm = xmap.ptr<float>(r - r0), *ym = ymap.ptr<float>(r - r0);
                for (int c = 0; c < cols; ++c) {
                    double u = x0 + c, v = y0 + r;
                    if (useField) {
                        Vec2f dd = field->sample(u + fieldShiftPx, v, canvasW);
                        u += dd[0];
                        v += dd[1];
                    }
                    double th = u / s, ph = std::max(0.0, std::min(CV_PI, v / s));
                    double sp = std::sin(ph);
                    Vec3d cam = toCam * Vec3d(sp * std::sin(th), -std::cos(ph), sp * std::cos(th));
                    if (cam[2] <= 1e-6) {
                        xm[c] = ym[c] = -1.f;
                        continue;
                    }
                    double iz = 1.0 / cam[2];
                    xm[c] = (float) (Kd(0, 0) * cam[0] * iz + Kd(0, 2));
                    ym[c] = (float) (Kd(1, 1) * cam[1] * iz + Kd(1, 2));
                }
            }
        });
        Mat di = dstImg.rowRange(r0, r1), dm = dstMask.rowRange(r0, r1);
        remap(src, di, xmap, ymap, INTER_LINEAR, BORDER_REFLECT);
        remap(ones, dm, xmap, ymap, INTER_NEAREST, BORDER_CONSTANT, Scalar::all(0));
    }
    return Point(x0, y0);
}

}  // namespace sphere
