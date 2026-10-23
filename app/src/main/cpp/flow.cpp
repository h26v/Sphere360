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

    // Làm mượt và cho độ uốn giảm dần ra ngoài vùng chồng (normalized convolution).
    const double sigma = kFlowSmoothDeg * degPx;
    std::vector<Field> fields(m);
    for (int k = 0; k < m; ++k) {
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
