#include "seams.h"

#include "geometry.h"
#include "params.h"

namespace sphere {

// Giao vùng quanh đỉnh/đáy cho tấm chụp thẳng lên/xuống. Trên ảnh equirect, các điểm gần cực nằm
// rất xa nhau theo chiều ngang nên đường nối toả ra từ cực không được trộn mịn; giao cả chỏm cho một
// tấm thì đường nối thành một vòng tròn vĩ độ, trộn được bình thường.
void assignPoleCaps(const std::vector<Matx33d> &R, const std::vector<Instance> &inst,
                    const std::vector<Mat> &valid, const std::vector<Point> &corners,
                    std::vector<UMat> &seamMasks, double s) {
    const int n = (int) R.size();
    const int m = (int) inst.size();
    const int capRows = (int) std::floor(s * kPoleCapDeg * CV_PI / 180.0);
    const int fullH = (int) std::ceil(CV_PI * s);
    for (int pole = -1; pole <= 1; pole += 2) {
        int best = -1;
        double bestY = 0.94;
        for (int i = 0; i < n; ++i) {
            double ay = opticalAxis(R[i])[1] * pole;  // trục y của toàn cảnh hướng xuống
            if (ay > bestY) {
                bestY = ay;
                best = i;
            }
        }
        if (best < 0) continue;
        const int rowFrom = pole < 0 ? 0 : fullH - capRows;
        const int rowTo = pole < 0 ? capRows : fullH;
        auto ownerValid = [&](int gx, int gy) {
            for (int k = 0; k < m; ++k) {
                if (inst[k].img != best || valid[k].empty()) continue;
                int lx = gx - corners[k].x, ly = gy - corners[k].y;
                if (lx >= 0 && ly >= 0 && lx < valid[k].cols && ly < valid[k].rows && valid[k].at<uchar>(ly, lx))
                    return true;
            }
            return false;
        };
        for (int k = 0; k < m; ++k) {
            if (valid[k].empty()) continue;
            Mat mk = seamMasks[k].getMat(ACCESS_RW);
            const Point c = corners[k];
            for (int gy = std::max(rowFrom, c.y); gy < std::min(rowTo, c.y + mk.rows); ++gy) {
                uchar *row = mk.ptr<uchar>(gy - c.y);
                const uchar *vrow = valid[k].ptr<uchar>(gy - c.y);
                for (int lx = 0; lx < mk.cols; ++lx) {
                    if (!ownerValid(c.x + lx, gy)) continue;
                    row[lx] = (inst[k].img == best) ? vrow[lx] : 0;
                }
            }
        }
        LOGI("Ảnh %d giữ vùng %s", best, pole < 0 ? "đỉnh trời" : "mặt đất");
    }
}

}  // namespace sphere
