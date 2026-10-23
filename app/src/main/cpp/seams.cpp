#include "seams.h"

#include "geometry.h"
#include "params.h"

namespace sphere {

namespace {

inline float dist2(const Point3f &a, const Point3f &b) {
    Point3f d = a - b;
    return d.dot(d);
}

inline float norm2(const Point3f &a) { return a.dot(a); }

}  // namespace

void ParallaxSeamFinder::find(const std::vector<UMat> &src, const std::vector<Point> &corners,
                              std::vector<UMat> &masks) {
    // Độ lớn gradient của từng ảnh như OpenCV (bình phương chuẩn L2 của Sobel trên 3 kênh).
    dx_.resize(src.size());
    dy_.resize(src.size());
    Mat dx, dy;
    for (size_t i = 0; i < src.size(); ++i) {
        Sobel(src[i], dx, CV_32F, 1, 0);
        Sobel(src[i], dy, CV_32F, 0, 1);
        dx_[i].create(src[i].size(), CV_32F);
        dy_[i].create(src[i].size(), CV_32F);
        for (int y = 0; y < src[i].rows; ++y) {
            const Point3f *dxr = dx.ptr<Point3f>(y), *dyr = dy.ptr<Point3f>(y);
            float *ox = dx_[i].ptr<float>(y), *oy = dy_[i].ptr<float>(y);
            for (int x = 0; x < src[i].cols; ++x) {
                ox[x] = norm2(dxr[x]);
                oy[x] = norm2(dyr[x]);
            }
        }
    }
    PairwiseSeamFinder::find(src, corners, masks);
    if (seamLength_ > 0)
        LOGI("Đường nối: %.0f px, thị sai trung bình trên đường nối %.2f°, lớn nhất %.1f°, %.1f%% điểm ở mức trần",
             seamLength_, seamParallax_ / seamLength_, seamMaxParallax_, 100.0 * seamOverCap_ / seamLength_);
}

void ParallaxSeamFinder::findInPair(size_t first, size_t second, Rect roi) {
    const float terminalCost = 10000.f, badRegionPenalty = 1000.f, eps = 1.f;
    Mat img1 = images_[first].getMat(ACCESS_READ), img2 = images_[second].getMat(ACCESS_READ);
    Mat mask1 = masks_[first].getMat(ACCESS_RW), mask2 = masks_[second].getMat(ACCESS_RW);
    const Mat &dx1 = dx_[first], &dx2 = dx_[second], &dy1 = dy_[first], &dy2 = dy_[second];
    const Point tl1 = corners_[first], tl2 = corners_[second];
    const PairParallax *par = nullptr;
    for (const PairParallax &pp : parallax_)
        if ((pp.p == (int) first && pp.q == (int) second) || (pp.p == (int) second && pp.q == (int) first)) {
            par = &pp;
            break;
        }

    // Cắt ảnh, mặt nạ, gradient và thị sai của vùng chồng, chừa thêm một viền.
    const int gap = 10;
    const int W = roi.width + 2 * gap, H = roi.height + 2 * gap;
    Mat s1(H, W, CV_32FC3, Scalar::all(0)), s2(H, W, CV_32FC3, Scalar::all(0));
    Mat m1(H, W, CV_8U, Scalar(0)), m2(H, W, CV_8U, Scalar(0));
    Mat gx1(H, W, CV_32F, Scalar(0)), gx2(H, W, CV_32F, Scalar(0)), gy1(H, W, CV_32F, Scalar(0)),
        gy2(H, W, CV_32F, Scalar(0)), P(H, W, CV_32F, Scalar(0));
    for (int y = -gap; y < roi.height + gap; ++y)
        for (int x = -gap; x < roi.width + gap; ++x) {
            const int sy = y + gap, sx = x + gap;
            int y1 = roi.y - tl1.y + y, x1 = roi.x - tl1.x + x;
            if (y1 >= 0 && x1 >= 0 && y1 < img1.rows && x1 < img1.cols) {
                s1.at<Point3f>(sy, sx) = img1.at<Point3f>(y1, x1);
                m1.at<uchar>(sy, sx) = mask1.at<uchar>(y1, x1);
                gx1.at<float>(sy, sx) = dx1.at<float>(y1, x1);
                gy1.at<float>(sy, sx) = dy1.at<float>(y1, x1);
            }
            int y2 = roi.y - tl2.y + y, x2 = roi.x - tl2.x + x;
            if (y2 >= 0 && x2 >= 0 && y2 < img2.rows && x2 < img2.cols) {
                s2.at<Point3f>(sy, sx) = img2.at<Point3f>(y2, x2);
                m2.at<uchar>(sy, sx) = mask2.at<uchar>(y2, x2);
                gx2.at<float>(sy, sx) = dx2.at<float>(y2, x2);
                gy2.at<float>(sy, sx) = dy2.at<float>(y2, x2);
            }
            if (par) {
                int py = roi.y + y - par->roi.y, px = roi.x + x - par->roi.x;
                if (py >= 0 && px >= 0 && py < par->deg.rows && px < par->deg.cols)
                    P.at<float>(sy, sx) = par->deg.at<float>(py, px);
            }
        }

    // Chi phí màu như COST_COLOR_GRAD: khác màu chia cho độ gắt của cạnh.
    Mat hw(H, W, CV_32F, Scalar(0)), vw(H, W, CV_32F, Scalar(0));
    Mat hbad(H, W, CV_8U, Scalar(0)), vbad(H, W, CV_8U, Scalar(0));
    std::vector<float> inside;
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            const bool ok = m1.at<uchar>(y, x) && m2.at<uchar>(y, x);
            const float d = dist2(s1.at<Point3f>(y, x), s2.at<Point3f>(y, x));
            if (x < W - 1) {
                float grad = gx1.at<float>(y, x) + gx1.at<float>(y, x + 1) + gx2.at<float>(y, x) +
                             gx2.at<float>(y, x + 1) + eps;
                float w = (d + dist2(s1.at<Point3f>(y, x + 1), s2.at<Point3f>(y, x + 1))) / grad + eps;
                bool good = ok && m1.at<uchar>(y, x + 1) && m2.at<uchar>(y, x + 1);
                hw.at<float>(y, x) = w;
                hbad.at<uchar>(y, x) = !good;
                if (good) inside.push_back(w);
            }
            if (y < H - 1) {
                float grad = gy1.at<float>(y, x) + gy1.at<float>(y + 1, x) + gy2.at<float>(y, x) +
                             gy2.at<float>(y + 1, x) + eps;
                float w = (d + dist2(s1.at<Point3f>(y + 1, x), s2.at<Point3f>(y + 1, x))) / grad + eps;
                bool good = ok && m1.at<uchar>(y + 1, x) && m2.at<uchar>(y + 1, x);
                vw.at<float>(y, x) = w;
                vbad.at<uchar>(y, x) = !good;
                if (good) inside.push_back(w);
            }
        }
    // Thang chi phí màu khác nhau theo cảnh, nên quy chi phí thị sai theo chi phí màu trung vị của cặp này.
    float med = 1.f;
    if (!inside.empty()) {
        auto mid = inside.begin() + inside.size() / 2;
        std::nth_element(inside.begin(), mid, inside.end());
        med = std::max(1.f, *mid);
    }
    const float k = (float) (kSeamParallaxWeight * med * 0.5);

    GCGraph<float> graph(W * H, (H - 1) * W + (W - 1) * H);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            int v = graph.addVtx();
            graph.addTermWeights(v, m1.at<uchar>(y, x) ? terminalCost : 0.f,
                                 m2.at<uchar>(y, x) ? terminalCost : 0.f);
        }
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            const int v = y * W + x;
            const float p = P.at<float>(y, x);
            if (x < W - 1) {
                float w = hw.at<float>(y, x) + k * (p + P.at<float>(y, x + 1));
                if (hbad.at<uchar>(y, x)) w += badRegionPenalty;
                graph.addEdges(v, v + 1, w, w);
            }
            if (y < H - 1) {
                float w = vw.at<float>(y, x) + k * (p + P.at<float>(y + 1, x));
                if (vbad.at<uchar>(y, x)) w += badRegionPenalty;
                graph.addEdges(v, v + W, w, w);
            }
        }
    graph.maxFlow();

    Mat label(H, W, CV_8U, Scalar(0));
    for (int y = 0; y < roi.height; ++y)
        for (int x = 0; x < roi.width; ++x) {
            const bool src = graph.inSourceSegment((y + gap) * W + x + gap);
            label.at<uchar>(y + gap, x + gap) = src;
            if (src) {
                if (mask1.at<uchar>(roi.y - tl1.y + y, roi.x - tl1.x + x))
                    mask2.at<uchar>(roi.y - tl2.y + y, roi.x - tl2.x + x) = 0;
            } else {
                if (mask2.at<uchar>(roi.y - tl2.y + y, roi.x - tl2.x + x))
                    mask1.at<uchar>(roi.y - tl1.y + y, roi.x - tl1.x + x) = 0;
            }
        }

    // Thống kê thị sai dọc đường nối (chỗ nhãn đổi bên trong vùng hai ảnh cùng phủ).
    if (!par) return;
    const float capDeg = (float) (kSeamParallaxCapDeg - kSeamParallaxFloorDeg) * 0.95f;
    for (int y = gap; y < gap + roi.height - 1; ++y)
        for (int x = gap; x < gap + roi.width - 1; ++x) {
            if (!(m1.at<uchar>(y, x) && m2.at<uchar>(y, x))) continue;
            const uchar l = label.at<uchar>(y, x);
            if (l == label.at<uchar>(y, x + 1) && l == label.at<uchar>(y + 1, x)) continue;
            const float p = P.at<float>(y, x);
            seamLength_ += 1;
            seamParallax_ += p;
            seamMaxParallax_ = std::max(seamMaxParallax_, (double) p);
            if (p >= capDeg) ++seamOverCap_;
        }
}

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
