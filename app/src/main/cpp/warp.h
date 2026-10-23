#pragma once
// Nắn ảnh lên khung equirect và trường uốn dùng khi nắn.

#include "common.h"

namespace sphere {

struct Instance {
    int img;
    int shift;  // 0 hoặc 1: dịch một vòng sang phải để phủ dải lặp ở mép phải
};

// Trường dịch chuyển (theo pixel của khung toàn cảnh rộng `width`) cho một bản ảnh.
struct Field {
    Point corner;
    Mat d;           // CV_32FC2
    double width = 0;
    double maxPx = 0;  // độ dịch lớn nhất, theo pixel của khung rộng `width`

    bool empty() const { return d.empty(); }

    // Lấy độ dịch tại toạ độ (gx, gy) của một khung toàn cảnh rộng targetWidth.
    Vec2f sample(double gx, double gy, double targetWidth) const {
        double ratio = width / targetWidth;
        double fx = gx * ratio - corner.x, fy = gy * ratio - corner.y;
        int x0 = (int) std::floor(fx), y0 = (int) std::floor(fy);
        if (x0 < 0 || y0 < 0 || x0 + 1 >= d.cols || y0 + 1 >= d.rows) return Vec2f(0, 0);
        float ax = (float) (fx - x0), ay = (float) (fy - y0);
        const Vec2f *r0 = d.ptr<Vec2f>(y0), *r1 = d.ptr<Vec2f>(y0 + 1);
        Vec2f v = (r0[x0] * (1 - ax) + r0[x0 + 1] * ax) * (1 - ay) + (r1[x0] * (1 - ax) + r1[x0 + 1] * ax) * ay;
        return v * (float) (1.0 / ratio);
    }
};

// Nắn một ảnh lên mặt cầu (u = s*atan2(x, z), v = s*(pi - acos(y))), có thể kèm trường uốn.
// Trả về toạ độ góc trên trái của ảnh ra trên khung.
Point warpSphere(const Mat &src, const Mat &K, const Mat &R, double s, Mat &dstImg, Mat &dstMask,
                 const Field *field = nullptr, double fieldShiftPx = 0);

}  // namespace sphere
