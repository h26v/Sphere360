#include "flow.h"

#include "epipolar.h"
#include "params.h"

namespace sphere {

namespace {

// Độ dài flow A tại (x, y) (pixel), kiểm tra ngược bằng flow B; -1 nếu hai chiều không khớp nhau.
float flowAt(const Mat &A, const Mat &B, int x, int y) {
    Vec2f f = A.at<Vec2f>(y, x);
    const float len = std::hypot(f[0], f[1]);
    int bx = cvRound(x + f[0]), by = cvRound(y + f[1]);
    if (bx < 0 || by < 0 || bx >= A.cols || by >= A.rows) return len;
    Vec2f g = B.at<Vec2f>(by, bx);
    if (std::abs(f[0] + g[0]) > 2.f || std::abs(f[1] + g[1]) > 2.f) return -1.f;
    return len;
}

// Bản đồ thị sai (độ) của vùng chồng ov cho đường nối. Hai nguồn:
//  - thị sai đo được: trung bình có trọng số (accNum / accDen) của các điểm flow mà bước uốn đã chấp nhận,
//    rải ở cả chỗ điểm nằm trong ảnh p lẫn chỗ nó tới trong ảnh q;
//  - flow hỏng dày đặc (hai chiều F, G không khớp nhau ở chỗ có góc cạnh): che khuất, hoặc vật gần lệch quá
//    tầm flow. Điểm hỏng lẻ ở mép cạnh thì bỏ qua.
PairParallax parallaxMap(const Mat &accNum, const Mat &accDen, const Mat &F, const Mat &G, const Mat &confA,
                         const Mat &confB, const Mat &both, Rect ov, int p, int q, double degPx, double ratio) {
    const float span = (float) (kSeamParallaxCapDeg - kSeamParallaxFloorDeg);
    Mat tex(ov.size(), CV_32F, Scalar(0)), bad(ov.size(), CV_32F, Scalar(0));
    const Mat *flows[2][3] = {{&F, &G, &confA}, {&G, &F, &confB}};
    for (int y = 0; y < ov.height; ++y) {
        const uchar *bm = both.ptr<uchar>(y);
        float *tr = tex.ptr<float>(y), *br = bad.ptr<float>(y);
        for (int x = 0; x < ov.width; ++x) {
            if (!bm[x]) continue;
            for (auto &d : flows) {
                if (d[2]->at<float>(y, x) < kSeamParallaxMinConf) continue;
                tr[x] += 1;
                if (flowAt(*d[0], *d[1], x, y) < 0) br[x] += 1;
            }
        }
    }
    const int win = std::max(3, cvRound(kSeamBadWindowDeg * degPx)) | 1;
    Mat texS, badS, numS, denS;
    boxFilter(tex, texS, -1, Size(win, win), Point(-1, -1), false);
    boxFilter(bad, badS, -1, Size(win, win), Point(-1, -1), false);
    const double sigma = std::max(0.5, kSeamParallaxSmoothDeg * degPx);
    GaussianBlur(accNum, numS, Size(), sigma);
    GaussianBlur(accDen, denS, Size(), sigma);
    const float minTex = 0.1f * win * win;
    const float lo = (float) kSeamBadRatioLo, hi = (float) kSeamBadRatioHi;
    const float minDen = (float) kSeamParallaxMinDensity;
    Mat P(ov.size(), CV_32F);
    for (int y = 0; y < ov.height; ++y) {
        const float *ts = texS.ptr<float>(y), *bs = badS.ptr<float>(y);
        const float *ns = numS.ptr<float>(y), *ds = denS.ptr<float>(y);
        float *pr = P.ptr<float>(y);
        for (int x = 0; x < ov.width; ++x) {
            float m = 0, b = 0;
            if (ds[x] >= minDen) m = std::min(span, std::max(0.f, ns[x] / ds[x] - (float) kSeamParallaxFloorDeg));
            if (ts[x] >= minTex) b = span * std::min(1.f, std::max(0.f, (bs[x] / ts[x] - lo) / (hi - lo)));
            pr[x] = std::max(m, b);
        }
    }
    const int r = std::max(1, cvRound(kSeamParallaxGrowDeg * degPx));
    dilate(P, P, getStructuringElement(MORPH_ELLIPSE, Size(2 * r + 1, 2 * r + 1)));
    PairParallax out{p, q};
    out.roi = Rect(cvRound(ov.x * ratio), cvRound(ov.y * ratio), std::max(1, cvRound(ov.width * ratio)),
                   std::max(1, cvRound(ov.height * ratio)));
    resize(P, out.deg, out.roi.size(), 0, 0, INTER_AREA);
    return out;
}

inline int floorDiv(int a, int b) { return a >= 0 ? a / b : -((-a + b - 1) / b); }

// Một ràng buộc của trường uốn chung: ở ô (cx, cy) của lưới toàn cảnh, bản q phải dịch hơn bản p đúng f
// (pixel khung flow) thì nội dung hai bản gặp nhau. w: mật độ điểm flow tin được trong ô (0..1).
struct FlowConstraint {
    int p, q, cx, cy;
    float w;
    Vec2f f;
};

// Trường uốn chung cho mọi bản ảnh: mỗi bản có một lưới độ dịch (ô g pixel, chung mạng lưới toàn cảnh), giải
//   min  sum_ràng buộc w |d_q - d_p - f|^2 + kJointSmooth sum_bản sum_ô kề |d_a - d_b|^2 + kJointPrior sum |d|^2
// bằng conjugate gradient. Khác cách cũ (mỗi cặp đẩy hai bản đi nửa đường rồi lấy trung bình), chỗ ba bốn
// bản chồng nhau vẫn khớp được với nhau; chỗ không có điểm flow (tường trơn, sàn) thì độ dịch được nội suy
// mượt từ xung quanh và nhạt dần ra ngoài.
void solveJointFields(const std::vector<FlowConstraint> &cons, const std::vector<Mat> &gray,
                      const std::vector<Point> &tl, int g, int width, std::vector<Field> &fields) {
    const int m = (int) gray.size();
    std::vector<int> ox(m, 0), oy(m, 0), nx(m, 0), ny(m, 0), off(m + 1, 0);
    for (int k = 0; k < m; ++k) {
        if (!gray[k].empty()) {
            ox[k] = floorDiv(tl[k].x, g);
            oy[k] = floorDiv(tl[k].y, g);
            nx[k] = floorDiv(tl[k].x + gray[k].cols - 1, g) - ox[k] + 1;
            ny[k] = floorDiv(tl[k].y + gray[k].rows - 1, g) - oy[k] + 1;
        }
        off[k + 1] = off[k] + nx[k] * ny[k];
    }
    const int N = off[m];
    if (N == 0) return;
    const float lam = (float) kJointSmooth, mu = (float) kJointPrior;
    auto node = [&](int k, int cx, int cy) {
        const int x = cx - ox[k], y = cy - oy[k];
        if (x < 0 || y < 0 || x >= nx[k] || y >= ny[k]) return -1;
        return off[k] + y * nx[k] + x;
    };
    struct Link {
        int a, b;
        float w;
    };
    std::vector<Link> links;
    std::vector<float> diag(N, mu);
    std::vector<Vec2f> rhs(N, Vec2f(0, 0));
    for (const FlowConstraint &c : cons) {
        const int a = node(c.p, c.cx, c.cy), b = node(c.q, c.cx, c.cy);
        if (a < 0 || b < 0) continue;
        links.push_back({a, b, c.w});
        diag[a] += c.w;
        diag[b] += c.w;
        rhs[b] += c.f * c.w;
        rhs[a] -= c.f * c.w;
    }
    // Độ mượt: 4 ô kề trong cùng một bản.
    for (int k = 0; k < m; ++k)
        for (int y = 0; y < ny[k]; ++y)
            for (int x = 0; x < nx[k]; ++x) {
                const int i = off[k] + y * nx[k] + x;
                if (x + 1 < nx[k]) {
                    links.push_back({i, i + 1, lam});
                    diag[i] += lam;
                    diag[i + 1] += lam;
                }
                if (y + 1 < ny[k]) {
                    links.push_back({i, i + nx[k], lam});
                    diag[i] += lam;
                    diag[i + nx[k]] += lam;
                }
            }
    auto mul = [&](const std::vector<Vec2f> &x, std::vector<Vec2f> &y) {
        for (int i = 0; i < N; ++i) y[i] = x[i] * diag[i];
        for (const Link &l : links) {
            y[l.a] -= x[l.b] * l.w;
            y[l.b] -= x[l.a] * l.w;
        }
    };
    auto dot2 = [&](const std::vector<Vec2f> &a, const std::vector<Vec2f> &b) {
        Vec2d sum(0, 0);
        for (int i = 0; i < N; ++i) {
            sum[0] += (double) a[i][0] * b[i][0];
            sum[1] += (double) a[i][1] * b[i][1];
        }
        return sum;
    };
    // Conjugate gradient, tiền điều kiện Jacobi; hai thành phần x, y giải song song (cùng ma trận).
    std::vector<Vec2f> x(N, Vec2f(0, 0)), r = rhs, z(N), d(N), Ad(N);
    for (int i = 0; i < N; ++i) z[i] = r[i] / diag[i];
    d = z;
    Vec2d rz = dot2(r, z);
    const Vec2d rz0 = rz;
    int it = 0;
    for (; it < kJointMaxIter; ++it) {
        if (rz[0] <= 1e-8 * rz0[0] + 1e-12 && rz[1] <= 1e-8 * rz0[1] + 1e-12) break;
        mul(d, Ad);
        const Vec2d dAd = dot2(d, Ad);
        const Vec2f alpha((float) (dAd[0] > 0 ? rz[0] / dAd[0] : 0), (float) (dAd[1] > 0 ? rz[1] / dAd[1] : 0));
        for (int i = 0; i < N; ++i) {
            x[i] += alpha.mul(d[i]);
            r[i] -= alpha.mul(Ad[i]);
            z[i] = r[i] / diag[i];
        }
        const Vec2d rzNew = dot2(r, z);
        const Vec2f beta((float) (rz[0] > 0 ? rzNew[0] / rz[0] : 0), (float) (rz[1] > 0 ? rzNew[1] / rz[1] : 0));
        for (int i = 0; i < N; ++i) d[i] = z[i] + beta.mul(d[i]);
        rz = rzNew;
    }
    // Ràng buộc còn lệch sau khi giải (pixel khung flow): lớn thì lưới cứng quá hoặc các cặp mâu thuẫn nhau.
    double res = 0, wsum = 0;
    for (const FlowConstraint &c : cons) {
        const int a = node(c.p, c.cx, c.cy), b = node(c.q, c.cx, c.cy);
        if (a < 0 || b < 0) continue;
        const Vec2f e = x[b] - x[a] - c.f;
        res += c.w * std::hypot(e[0], e[1]);
        wsum += c.w;
    }
    LOGI("Trường uốn chung: %d ô, %zu ràng buộc, %d vòng CG, ràng buộc còn lệch trung bình %.2f px", N,
         cons.size(), it, wsum > 0 ? res / wsum : 0.0);

    // Lưới -> trường dày: giá trị của ô đặt ở tâm ô, phóng to tuyến tính rồi cắt đúng khung bản ảnh.
    for (int k = 0; k < m; ++k) {
        if (gray[k].empty()) continue;
        Mat grid(ny[k], nx[k], CV_32FC2);
        for (int y = 0; y < ny[k]; ++y)
            for (int xx = 0; xx < nx[k]; ++xx) grid.at<Vec2f>(y, xx) = x[off[k] + y * nx[k] + xx];
        Mat big;
        resize(grid, big, Size(nx[k] * g, ny[k] * g), 0, 0, INTER_LINEAR);
        const Rect crop(tl[k].x - ox[k] * g, tl[k].y - oy[k] * g, gray[k].cols, gray[k].rows);
        fields[k].d = big(crop).clone();
        fields[k].corner = tl[k];
        fields[k].width = width;
        std::vector<Mat> ch;
        split(fields[k].d, ch);
        Mat mag;
        magnitude(ch[0], ch[1], mag);
        minMaxLoc(mag, nullptr, &fields[k].maxPx);
    }
}


// Uốn một bản (ở khung flow, chưa uốn) theo trường của nó: điểm x lấy màu từ x + d(x).
void warpByField(const Mat &gray, const Mat &mask, const Field &field, Mat &g2, Mat &m2) {
    if (field.empty()) {
        g2 = gray;
        m2 = mask;
        return;
    }
    Mat mx(gray.size(), CV_32F), my(gray.size(), CV_32F);
    for (int y = 0; y < gray.rows; ++y) {
        const Vec2f *dr = field.d.ptr<Vec2f>(y);
        float *xr = mx.ptr<float>(y), *yr = my.ptr<float>(y);
        for (int x = 0; x < gray.cols; ++x) {
            xr[x] = x + dr[x][0];
            yr[x] = y + dr[x][1];
        }
    }
    remap(gray, g2, mx, my, INTER_LINEAR, BORDER_REPLICATE);
    remap(mask, m2, mx, my, INTER_NEAREST, BORDER_CONSTANT, Scalar(0));
}

// Lượt tinh chỉnh của trường uốn chung: uốn mỗi bản theo trường hiện có, đo flow còn lại ở các vùng chồng rồi
// giải phần bù, cộng dồn vào trường. Lượt đầu coi flow F không đổi khi ảnh dịch đi (tuyến tính hoá) và chỉ có
// điểm qua được mô hình epipolar; lượt này bù phần sai đó. Flow còn lại nhỏ nên dùng flow tự do có trần.
void refineJointFields(const std::vector<Mat> &gray, const std::vector<Mat> &mask, const std::vector<Point> &tl,
                       const std::vector<std::pair<int, int>> &pairList, int g, int width, double degPx,
                       std::vector<Field> &fields) {
    const int m = (int) gray.size();
    std::vector<Mat> g2(m), m2(m);
    for (int k = 0; k < m; ++k)
        if (!gray[k].empty()) warpByField(gray[k], mask[k], fields[k], g2[k], m2[k]);
    Ptr<DISOpticalFlow> dis = DISOpticalFlow::create(DISOpticalFlow::PRESET_MEDIUM);
    const float cap = (float) (kJointRefineMaxDeg * degPx);
    const int poleSkip = (int) (kFlowPoleSkipDeg * degPx), fullH = (int) std::ceil(180 * degPx);
    std::vector<FlowConstraint> cons;
    for (const auto &pq : pairList) {
        const int p = pq.first, q = pq.second;
        if (g2[p].empty() || g2[q].empty()) continue;
        Rect ov = Rect(tl[p], g2[p].size()) & Rect(tl[q], g2[q].size());
        ov &= Rect(ov.x, poleSkip, ov.width, std::max(0, fullH - 2 * poleSkip));
        if (ov.width < 16 || ov.height < 16) continue;
        Rect lp(ov.tl() - tl[p], ov.size()), lq(ov.tl() - tl[q], ov.size());
        Mat both = m2[p](lp) & m2[q](lq);
        if (countNonZero(both) < 400) continue;
        Mat a = g2[p](lp).clone(), b = g2[q](lq).clone(), F, G, conf;
        dis->calc(a, b, F);
        dis->calc(b, a, G);
        cornerMinEigenVal(a, conf, 7, 3);
        conf *= 1.0 / kFlowConfRef;
        min(conf, 1.0, conf);
        const int cx0 = floorDiv(ov.x, g), cy0 = floorDiv(ov.y, g);
        const int ncx = floorDiv(ov.x + ov.width - 1, g) - cx0 + 1;
        const int ncy = floorDiv(ov.y + ov.height - 1, g) - cy0 + 1;
        Mat cellW = Mat::zeros(ncy, ncx, CV_32F), cellF = Mat::zeros(ncy, ncx, CV_32FC2);
        for (int y = 0; y < ov.height; ++y) {
            const uchar *bm = both.ptr<uchar>(y);
            const Vec2f *fr = F.ptr<Vec2f>(y);
            const float *cr = conf.ptr<float>(y);
            for (int x = 0; x < ov.width; ++x) {
                if (!bm[x] || cr[x] < 0.05f) continue;
                const Vec2f f = fr[x];
                if (std::abs(f[0]) > cap || std::abs(f[1]) > cap) continue;
                const int bx = cvRound(x + f[0]), by = cvRound(y + f[1]);
                if (bx < 0 || by < 0 || bx >= ov.width || by >= ov.height) continue;
                const Vec2f h = G.at<Vec2f>(by, bx);
                if (std::abs(f[0] + h[0]) > 1.f || std::abs(f[1] + h[1]) > 1.f) continue;
                const int cx = floorDiv(ov.x + x, g) - cx0, cy = floorDiv(ov.y + y, g) - cy0;
                cellW.at<float>(cy, cx) += cr[x];
                cellF.at<Vec2f>(cy, cx) += f * cr[x];
            }
        }
        const float cellArea = (float) (g * g);
        for (int cy = 0; cy < ncy; ++cy)
            for (int cx = 0; cx < ncx; ++cx) {
                const float w = cellW.at<float>(cy, cx);
                if (w > 0) cons.push_back({p, q, cx0 + cx, cy0 + cy, w / cellArea, cellF.at<Vec2f>(cy, cx) / w});
            }
    }
    std::vector<Field> delta(m);
    solveJointFields(cons, gray, tl, g, width, delta);
    for (int k = 0; k < m; ++k) {
        if (fields[k].empty() || delta[k].empty()) continue;
        fields[k].d += delta[k].d;
        std::vector<Mat> ch;
        split(fields[k].d, ch);
        Mat mag;
        magnitude(ch[0], ch[1], mag);
        minMaxLoc(mag, nullptr, &fields[k].maxPx);
    }
}


// Bản đồ độ lệch còn lại (độ) sau khi uốn, cho đường nối: ở mỗi vùng chồng, uốn hai bản theo trường cuối cùng
// rồi đo flow giữa chúng. Chỗ có góc cạnh mà flow hai chiều không khớp (che khuất, lệch quá tầm) tính mức trần.
// Khác bản đồ thị sai (đo trước khi uốn): vật gần đã được uốn khớp thì cắt qua cũng không lộ.
std::vector<PairParallax> residualMaps(const std::vector<Mat> &gray, const std::vector<Mat> &mask,
                                       const std::vector<Point> &tl, const std::vector<std::pair<int, int>> &pairList,
                                       const std::vector<Field> &fields, double degPx, double ratio) {
    const int m = (int) gray.size();
    std::vector<Mat> g2(m), m2(m);
    for (int k = 0; k < m; ++k)
        if (!gray[k].empty()) warpByField(gray[k], mask[k], fields[k], g2[k], m2[k]);
    Ptr<DISOpticalFlow> dis = DISOpticalFlow::create(DISOpticalFlow::PRESET_MEDIUM);
    const float span = (float) (kSeamParallaxCapDeg - kSeamParallaxFloorDeg);
    const int poleSkip = (int) (kFlowPoleSkipDeg * degPx), fullH = (int) std::ceil(180 * degPx);
    std::vector<PairParallax> out;
    for (const auto &pq : pairList) {
        const int p = pq.first, q = pq.second;
        if (g2[p].empty() || g2[q].empty()) continue;
        Rect ov = Rect(tl[p], g2[p].size()) & Rect(tl[q], g2[q].size());
        ov &= Rect(ov.x, poleSkip, ov.width, std::max(0, fullH - 2 * poleSkip));
        if (ov.width < 16 || ov.height < 16) continue;
        Rect lp(ov.tl() - tl[p], ov.size()), lq(ov.tl() - tl[q], ov.size());
        Mat both = m2[p](lp) & m2[q](lq);
        if (countNonZero(both) < 400) continue;
        Mat a = g2[p](lp).clone(), b = g2[q](lq).clone(), F, G, conf;
        dis->calc(a, b, F);
        dis->calc(b, a, G);
        cornerMinEigenVal(a, conf, 7, 3);
        conf *= 1.0 / kFlowConfRef;
        Mat num(ov.size(), CV_32F, Scalar(0)), den(ov.size(), CV_32F, Scalar(0));
        for (int y = 0; y < ov.height; ++y) {
            const uchar *bm = both.ptr<uchar>(y);
            const float *cr = conf.ptr<float>(y);
            float *nr = num.ptr<float>(y), *dr = den.ptr<float>(y);
            for (int x = 0; x < ov.width; ++x) {
                if (!bm[x] || cr[x] < kSeamParallaxMinConf) continue;
                const float len = flowAt(F, G, x, y);
                const float deg = len < 0 ? span : std::min(span, (float) (len / degPx * kSeamResidualScale));
                nr[x] = deg;
                dr[x] = 1;
            }
        }
        const double sigma = std::max(0.5, kSeamParallaxSmoothDeg * degPx);
        GaussianBlur(num, num, Size(), sigma);
        GaussianBlur(den, den, Size(), sigma);
        Mat P(ov.size(), CV_32F, Scalar(0));
        for (int y = 0; y < ov.height; ++y) {
            const float *nr = num.ptr<float>(y), *dr = den.ptr<float>(y);
            float *pr = P.ptr<float>(y);
            for (int x = 0; x < ov.width; ++x)
                if (dr[x] >= kSeamParallaxMinDensity) pr[x] = std::max(0.f, nr[x] / dr[x] - (float) kSeamResidualFloorDeg);
        }
        const int r = std::max(1, cvRound(kSeamParallaxGrowDeg * degPx));
        dilate(P, P, getStructuringElement(MORPH_ELLIPSE, Size(2 * r + 1, 2 * r + 1)));
        PairParallax pp{p, q};
        pp.roi = Rect(cvRound(ov.x * ratio), cvRound(ov.y * ratio), std::max(1, cvRound(ov.width * ratio)),
                      std::max(1, cvRound(ov.height * ratio)));
        resize(P, pp.deg, pp.roi.size(), 0, 0, INTER_AREA);
        out.push_back(std::move(pp));
    }
    return out;
}

}  // namespace

// Tính trường uốn cho từng bản ảnh: ở vùng chồng của hai bản p, q, tính flow F (p -> q) rồi cho p
// lùi nửa đường (-F/2), q tiến nửa đường (+F/2) để nội dung gặp nhau ở giữa. Ra ngoài vùng chồng,
// độ uốn giảm dần về 0. Cặp nào có flow khớp mô hình epipolar thì F được ép theo mô hình đó và cho phép
// dịch tới kEpiMaxDeg; cặp không khớp (máy gần như chỉ xoay, hoặc flow nhiễu) giữ cách cũ, trần kFlowMaxDeg.
std::vector<Field> computeFlowFields(const std::vector<Instance> &inst, const std::vector<Mat> &warped,
                                     const std::vector<Mat> &masks, const std::vector<Point> &corners,
                                     int width, double seamRatio, std::vector<PairParallax> *parallax,
                                     const BridgeChains *bridges) {
    const int m = (int) inst.size();
    const double s = width / (2 * CV_PI);
    const double degPx = s * CV_PI / 180.0;
    const float maxFlow = (float) (kFlowMaxDeg * degPx);
    const int poleSkip = (int) (kFlowPoleSkipDeg * degPx);
    const int fullH = (int) std::ceil(CV_PI * s);

    std::vector<Mat> gray(m), mask(m), num(m), weight(m);
    std::vector<Point> tl(m);
    for (int k = 0; k < m; ++k) {
        const int i = inst[k].img;
        tl[k] = corners[i] + Point(inst[k].shift * width, 0);
        if (warped[i].empty()) continue;
        cvtColor(warped[i], gray[k], COLOR_BGR2GRAY);
        mask[k] = masks[i];
        num[k] = Mat::zeros(warped[i].size(), CV_32FC2);
        weight[k] = Mat::zeros(warped[i].size(), CV_32F);
    }

    Ptr<DISOpticalFlow> dis = DISOpticalFlow::create(DISOpticalFlow::PRESET_MEDIUM);
    const float epiMaxFlow = (float) (kEpiMaxDeg * degPx);
    RNG rng(20261023);
    int pairs = 0, epiPairs = 0, chainPairs = 0;
    const int g = std::max(2, cvRound(kJointGridDeg * degPx));
    std::vector<FlowConstraint> cons;
    std::vector<std::pair<int, int>> pairList;
    double maxParallaxDeg = 0;
    for (int p = 0; p < m; ++p)
        for (int q = p + 1; q < m; ++q) {
            if (gray[p].empty() || gray[q].empty() || inst[p].img == inst[q].img) continue;
            Rect rp(tl[p], gray[p].size()), rq(tl[q], gray[q].size());
            Rect band(rp.x, poleSkip, rp.width, std::max(0, fullH - 2 * poleSkip));
            Rect ov = rp & rq & band;
            if (ov.width < 16 || ov.height < 16) continue;
            Rect lp(ov.tl() - tl[p], ov.size()), lq(ov.tl() - tl[q], ov.size());
            Mat both = mask[p](lp) & mask[q](lq);
            if (countNonZero(both) < 400) continue;

            Mat a = gray[p](lp).clone(), b = gray[q](lq).clone();
            Mat F, G;
            const int ip = inst[p].img, iq = inst[q].img;
            const int gap = std::min(ip, iq);
            const bool useChain = bridges && kUseBridges && std::abs(ip - iq) == 1 && gap < (int) bridges->size() &&
                                  !(*bridges)[gap].frames.empty();
            if (useChain) {
                int hops = 0;
                double cover = bridgeFlows(*bridges, gap, ip < iq, View{gray[p], mask[p], tl[p]},
                                           View{gray[q], mask[q], tl[q]}, ov, width, a, b, F, G, hops);
                if (cover >= kBridgeMinCover) ++chainPairs;
                LOGI("Cặp %d-%d: %zu khung cầu nối, chuỗi %d bước, phủ %.0f%% vùng chồng", ip, iq,
                     (*bridges)[gap].frames.size(), hops, cover * 100);
            } else {
                dis->calc(a, b, F);
                dis->calc(b, a, G);
            }
            ++pairs;
            pairList.emplace_back(p, q);
            // Độ tin cậy: giá trị riêng nhỏ của structure tensor. Chỗ có góc cạnh thật thì cao; tường
            // trơn hoặc vạch song song (trần tôn) thì thấp vì flow dọc theo vạch không xác định được.
            Mat conf;
            cornerMinEigenVal(a, conf, 7, 3);
            conf *= 1.0 / kFlowConfRef;
            min(conf, 1.0, conf);

            // Gom các điểm có flow đáng tin (kiểm tra hai chiều, đủ góc cạnh).
            std::vector<FlowSample> samples;
            for (int y = 0; y < ov.height; ++y) {
                const uchar *bm = both.ptr<uchar>(y);
                const Vec2f *fr = F.ptr<Vec2f>(y);
                const float *cr = conf.ptr<float>(y);
                for (int x = 0; x < ov.width; ++x) {
                    if (!bm[x] || cr[x] < 0.05f) continue;
                    Vec2f f = fr[x];
                    if (std::abs(f[0]) > epiMaxFlow || std::abs(f[1]) > epiMaxFlow) continue;
                    // Kiểm tra hai chiều: đi theo F rồi quay về theo G phải về gần chỗ cũ.
                    int bx = cvRound(x + f[0]), by = cvRound(y + f[1]);
                    if (bx < 0 || by < 0 || bx >= ov.width || by >= ov.height) continue;
                    Vec2f g = G.at<Vec2f>(by, bx);
                    if (std::abs(f[0] + g[0]) > 1.5f || std::abs(f[1] + g[1]) > 1.5f) continue;
                    double gx = ov.x + x, gy = ov.y + y;
                    samples.push_back({panoDir(gx, gy, s), panoDir(gx + f[0], gy + f[1], s), f, cr[x], x, y});
                }
            }

            EpiModel M = fitEpipolar(samples, s, rng);
            std::vector<float> along;
            Mat np = num[p](lp), wp = weight[p](lp), nq = num[q](lq), wq = weight[q](lq);
            Mat accNum, accDen;
            if (parallax) {
                accNum = Mat::zeros(ov.size(), CV_32F);
                accDen = Mat::zeros(ov.size(), CV_32F);
            }
            // Gom điểm flow theo ô lưới toàn cảnh cho trường uốn chung.
            const int cx0 = floorDiv(ov.x, g), cy0 = floorDiv(ov.y, g);
            const int ncx = floorDiv(ov.x + ov.width - 1, g) - cx0 + 1;
            const int ncy = floorDiv(ov.y + ov.height - 1, g) - cy0 + 1;
            Mat cellW, cellF;
            if (kFlowJoint) {
                cellW = Mat::zeros(ncy, ncx, CV_32F);
                cellF = Mat::zeros(ncy, ncx, CV_32FC2);
            }
            for (const FlowSample &smp : samples) {
                Vec2f f = smp.f;
                if (M.ok) {
                    float perp;
                    f = constrainFlow(M, smp, s, perp);
                    float mag = std::hypot(f[0], f[1]);
                    if (perp > std::max(1.5 * kEpiInlierPx, 0.2 * std::hypot(smp.f[0], smp.f[1]))) continue;
                    if (mag > epiMaxFlow) continue;
                    along.push_back(mag);
                } else if (std::abs(f[0]) > maxFlow || std::abs(f[1]) > maxFlow) {
                    continue;
                }
                float c = smp.conf;
                if (kFlowJoint) {
                    const int cx = floorDiv(ov.x + smp.x, g) - cx0, cy = floorDiv(ov.y + smp.y, g) - cy0;
                    cellW.at<float>(cy, cx) += c;
                    cellF.at<Vec2f>(cy, cx) += f * c;
                }
                np.at<Vec2f>(smp.y, smp.x) -= f * (0.5f * c);
                wp.at<float>(smp.y, smp.x) += c;
                nq.at<Vec2f>(smp.y, smp.x) += f * (0.5f * c);
                wq.at<float>(smp.y, smp.x) += c;
                if (parallax) {
                    // Rải thị sai ở chỗ điểm nằm trong ảnh p và chỗ nó tới trong ảnh q.
                    const float deg = (float) (std::hypot(f[0], f[1]) / degPx);
                    const int xs[2] = {smp.x, cvRound(smp.x + f[0])}, ys[2] = {smp.y, cvRound(smp.y + f[1])};
                    for (int t = 0; t < 2; ++t) {
                        if (xs[t] < 0 || ys[t] < 0 || xs[t] >= ov.width || ys[t] >= ov.height) continue;
                        accNum.at<float>(ys[t], xs[t]) += c * deg;
                        accDen.at<float>(ys[t], xs[t]) += c;
                    }
                }
            }
            if (kFlowJoint) {
                const float cellArea = (float) (g * g);
                for (int cy = 0; cy < ncy; ++cy)
                    for (int cx = 0; cx < ncx; ++cx) {
                        float w = cellW.at<float>(cy, cx);
                        if (w <= 0) continue;
                        cons.push_back({p, q, cx0 + cx, cy0 + cy, w / cellArea, cellF.at<Vec2f>(cy, cx) / w});
                    }
            }
            if (parallax) {
                Mat confB;
                cornerMinEigenVal(b, confB, 7, 3);
                confB *= 1.0 / kFlowConfRef;
                parallax->push_back(parallaxMap(accNum, accDen, F, G, conf, confB, both, ov, p, q, degPx, seamRatio));
            }
            if (M.ok) {
                ++epiPairs;
                double p95 = 0;
                if (!along.empty()) {
                    size_t k = along.size() * 95 / 100;
                    std::nth_element(along.begin(), along.begin() + k, along.end());
                    p95 = along[k] / degPx;
                }
                maxParallaxDeg = std::max(maxParallaxDeg, p95);
                LOGI("Cặp %d-%d: epipolar khớp %d/%d điểm có dịch, b=(%.2f,%.2f,%.2f), thị sai p95 %.1f°",
                     inst[p].img, inst[q].img, M.inliers, M.moving, M.b[0], M.b[1], M.b[2], p95);
            } else {
                LOGI("Cặp %d-%d: không khớp epipolar (%d/%d điểm có dịch), dùng flow tự do",
                     inst[p].img, inst[q].img, M.inliers, M.moving);
            }
        }

    std::vector<Field> fields(m);
    if (kFlowJoint) {
        solveJointFields(cons, gray, tl, g, width, fields);
        for (int pass = 1; pass < kJointPasses; ++pass)
            refineJointFields(gray, mask, tl, pairList, g, width, degPx, fields);
        if (parallax && kSeamResidual > 0) {
            std::vector<PairParallax> res = residualMaps(gray, mask, tl, pairList, fields, degPx, seamRatio);
            if (kSeamResidual == 1) {
                *parallax = std::move(res);
            } else {
                for (PairParallax &r : res)
                    for (const PairParallax &old : *parallax)
                        if (old.p == r.p && old.q == r.q && old.roi == r.roi) max(r.deg, old.deg, r.deg);
                *parallax = std::move(res);
            }
        }
    }
    // Cách cũ: làm mượt và cho độ uốn giảm dần ra ngoài vùng chồng (normalized convolution).
    const double sigma = kFlowSmoothDeg * degPx;
    for (int k = 0; k < m && !kFlowJoint; ++k) {
        if (num[k].empty()) continue;
        Mat valid;
        mask[k].convertTo(valid, CV_32F, 1.0 / 255);
        Mat gn, gw, gv;
        GaussianBlur(num[k], gn, Size(), sigma);
        GaussianBlur(weight[k], gw, Size(), sigma);
        GaussianBlur(valid, gv, Size(), sigma * 2);
        Mat den = gw + gv * kFlowFalloff + 1e-6;
        std::vector<Mat> ch;
        split(gn, ch);
        divide(ch[0], den, ch[0]);
        divide(ch[1], den, ch[1]);
        merge(ch, fields[k].d);
        fields[k].corner = tl[k];
        fields[k].width = width;
        Mat mag;
        magnitude(ch[0], ch[1], mag);
        minMaxLoc(mag, nullptr, &fields[k].maxPx);
    }
    LOGI("Optical flow: %d cặp vùng chồng, %d cặp theo epipolar, %d cặp dò qua khung cầu nối, thị sai lớn nhất "
         "%.1f°", pairs, epiPairs, chainPairs, maxParallaxDeg);
    return fields;
}

}  // namespace sphere
