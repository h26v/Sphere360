#include "flow.h"

#include "epipolar.h"
#include "params.h"

namespace sphere {

// Tính trường uốn cho từng bản ảnh: ở vùng chồng của hai bản p, q, tính flow F (p -> q) rồi cho p
// lùi nửa đường (-F/2), q tiến nửa đường (+F/2) để nội dung gặp nhau ở giữa. Ra ngoài vùng chồng,
// độ uốn giảm dần về 0. Cặp nào có flow khớp mô hình epipolar thì F được ép theo mô hình đó và cho phép
// dịch tới kEpiMaxDeg; cặp không khớp (máy gần như chỉ xoay, hoặc flow nhiễu) giữ cách cũ, trần kFlowMaxDeg.
std::vector<Field> computeFlowFields(const std::vector<Instance> &inst, const std::vector<Mat> &warped,
                                     const std::vector<Mat> &masks, const std::vector<Point> &corners,
                                     int width) {
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
    int pairs = 0, epiPairs = 0;
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
            dis->calc(a, b, F);
            dis->calc(b, a, G);
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
    LOGI("Optical flow: %d cặp vùng chồng, %d cặp theo epipolar, thị sai lớn nhất %.1f°", pairs, epiPairs,
         maxParallaxDeg);
    return fields;
}

}  // namespace sphere
