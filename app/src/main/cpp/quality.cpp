#include "quality.h"

#include "params.h"

namespace sphere {

ResidualStats measureResidual(const std::vector<Instance> &inst, const std::vector<Mat> &images,
                              const std::vector<Mat> &K, const std::vector<Mat> &R, const std::vector<Field> &fields,
                              int width, const std::vector<UMat> &seamMasks, const std::vector<Point> &seamCorners,
                              int seamWidth, const std::vector<Mat> *srcMasks) {
    const int m = (int) inst.size();
    const double s = width / (2 * CV_PI);
    const double degPx = width / 360.0;
    const int poleSkip = (int) (kFlowPoleSkipDeg * degPx);
    const int fullH = (int) std::ceil(CV_PI * s);
    const double ratio = (double) seamWidth / width;

    std::vector<Mat> gray(m), mask(m), seam(m);
    std::vector<Point> tl(m);
    for (int k = 0; k < m; ++k) {
        const int i = inst[k].img;
        if (images[i].empty()) continue;
        Mat w, wm;
        Point c = warpSphere(images[i], K[i], R[i], s, w, wm, &fields[k], inst[k].shift * (double) width,
                             srcMasks ? (*srcMasks)[i] : Mat());
        if (w.empty()) continue;
        cvtColor(w, gray[k], COLOR_BGR2GRAY);
        mask[k] = wm;
        tl[k] = c + Point(inst[k].shift * width, 0);
        dilate(seamMasks[k].getMat(ACCESS_READ), seam[k], Mat());
    }

    Ptr<DISOpticalFlow> dis = DISOpticalFlow::create(DISOpticalFlow::PRESET_MEDIUM);
    const float minGrad = 6.f;  // độ lớn gradient (mức xám / pixel) coi là có cạnh
    double sumAll = 0, sumSeam = 0;
    long nAll = 0, nSeamTex = 0, nSeamBroken = 0;
    std::vector<float> seamRes;
    struct PairScore {
        int i, j, n;
        double mean;
        Mat dbg;
    };
    std::vector<PairScore> scores;
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
            Mat a = gray[p](lp).clone(), b = gray[q](lq).clone(), F, G, gx, gy;
            dis->calc(a, b, F);
            dis->calc(b, a, G);
            Sobel(a, gx, CV_32F, 1, 0, 3, 1.0 / 8);
            Sobel(a, gy, CV_32F, 0, 1, 3, 1.0 / 8);
            double pairSum = 0;
            int pairN = 0;
            // Ảnh debug: p | q | độ lệch (xanh lá = khớp, đỏ = lệch 1°+, xanh dương = flow hỏng), viền đường nối.
            Mat res;
            if (!gDebugStem.empty()) res = Mat(ov.size(), CV_8UC3, Scalar::all(0));
            for (int y = 0; y < ov.height; ++y) {
                const uchar *bm = both.ptr<uchar>(y);
                const Vec2f *fr = F.ptr<Vec2f>(y);
                const float *xr = gx.ptr<float>(y), *yr = gy.ptr<float>(y);
                const int sy = cvRound((ov.y + y) * ratio);
                for (int x = 0; x < ov.width; ++x) {
                    if (!bm[x]) continue;
                    const float g = std::hypot(xr[x], yr[x]);
                    if (g < minGrad) continue;
                    // Điểm này có nằm trên đường nối giữa p và q không (mặt nạ đường nối của cả hai đều phủ).
                    const int sx = cvRound((ov.x + x) * ratio);
                    const int px = sx - seamCorners[p].x, py = sy - seamCorners[p].y;
                    const int qx = sx - seamCorners[q].x, qy = sy - seamCorners[q].y;
                    const bool onSeam = px >= 0 && py >= 0 && px < seam[p].cols && py < seam[p].rows && qx >= 0 &&
                                        qy >= 0 && qx < seam[q].cols && qy < seam[q].rows &&
                                        seam[p].at<uchar>(py, px) && seam[q].at<uchar>(qy, qx);
                    const Vec2f f = fr[x];
                    const int bx = cvRound(x + f[0]), by = cvRound(y + f[1]);
                    bool ok = bx >= 0 && by >= 0 && bx < ov.width && by < ov.height;
                    if (ok) {
                        const Vec2f h = G.at<Vec2f>(by, bx);
                        // Chỉ xét thành phần vuông góc với cạnh: dọc cạnh flow vốn không xác định.
                        ok = std::abs((f[0] + h[0]) * xr[x] + (f[1] + h[1]) * yr[x]) <= g;
                    }
                    if (onSeam) {
                        ++nSeamTex;
                        if (!ok) ++nSeamBroken;
                    }
                    const float r = std::abs(f[0] * xr[x] + f[1] * yr[x]) / g;
                    if (!res.empty()) {
                        const float t = std::min(1.f, (float) (r / degPx));
                        res.at<Vec3b>(y, x) = !ok ? Vec3b(255, 0, 0) : Vec3b(0, (uchar) (255 * (1 - t)), (uchar) (255 * t));
                        if (onSeam) res.at<Vec3b>(y, x) = res.at<Vec3b>(y, x) * 0.5 + Vec3b(127, 127, 127);
                    }
                    if (!ok) continue;
                    sumAll += r;
                    ++nAll;
                    if (onSeam) {
                        sumSeam += r;
                        seamRes.push_back(r);
                        pairSum += r;
                        ++pairN;
                    }
                }
            }
            Mat dbg;
            if (!res.empty()) {
                Mat ac, bc;
                cvtColor(a, ac, COLOR_GRAY2BGR);
                cvtColor(b, bc, COLOR_GRAY2BGR);
                hconcat(std::vector<Mat>{ac, bc, res}, dbg);
            }
            if (pairN > 0) scores.push_back({inst[p].img, inst[q].img, pairN, pairSum / pairN, dbg});
        }

    ResidualStats st;
    st.overlapDeg = nAll ? sumAll / nAll / degPx : 0;
    st.seamPx = (int) seamRes.size();
    st.seamDeg = seamRes.empty() ? 0 : sumSeam / seamRes.size() / degPx;
    if (!seamRes.empty()) {
        size_t k = seamRes.size() * 9 / 10;
        std::nth_element(seamRes.begin(), seamRes.begin() + k, seamRes.end());
        st.seamP90Deg = seamRes[k] / degPx;
    }
    st.seamBroken = nSeamTex ? (double) nSeamBroken / nSeamTex : 0;
    LOGI("Độ lệch sau uốn: vùng chồng %.3f°, đường nối %.3f° (p90 %.3f°, %d điểm cạnh, %.1f%% flow hỏng)",
         st.overlapDeg, st.seamDeg, st.seamP90Deg, st.seamPx, st.seamBroken * 100);
    // Các cặp lệch nhiều nhất trên đường nối (tính theo tổng, tức là chỗ lộ nhiều nhất).
    std::sort(scores.begin(), scores.end(),
              [](const PairScore &x, const PairScore &y) { return x.mean * x.n > y.mean * y.n; });
    for (size_t k = 0; k < scores.size() && k < 8; ++k) {
        LOGI("  đường nối %d-%d: %d điểm cạnh, lệch trung bình %.2f°", scores[k].i, scores[k].j, scores[k].n,
             scores[k].mean / degPx);
        if (k < 4 && !scores[k].dbg.empty())
            imwrite(gDebugStem + "_res_" + std::to_string(scores[k].i) + "_" + std::to_string(scores[k].j) + ".jpg",
                    scores[k].dbg, {IMWRITE_JPEG_QUALITY, 90});
    }
    return st;
}

}  // namespace sphere
