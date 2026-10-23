#include "alignment.h"

#include "geometry.h"
#include "params.h"

namespace sphere {

std::vector<RayPair> collectRayPairs(const std::vector<ImageFeatures> &features,
                                     const std::vector<MatchesInfo> &pairwise,
                                     const std::vector<Matx33d> &sensor, const Matx33d &Kinv,
                                     double focalWork, int &pairsUsed) {
    const int n = (int) features.size();
    std::vector<RayPair> out;
    pairsUsed = 0;
    // Khoảng 4 pixel ở độ phân giải làm việc: cảnh trong nhà có thị sai nên không thể đòi khít hơn.
    const double inlierDeg = std::max(0.15, 4.0 / focalWork * 180.0 / CV_PI);
    RNG rng(12345);
    auto ray = [&](const Point2f &p) {
        Vec3d v = Kinv * Vec3d(p.x, p.y, 1);
        return v * (1.0 / norm(v));
    };
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j) {
            const MatchesInfo &m = pairwise[i * n + j];
            if (m.matches.size() < 15 || m.src_img_idx < 0) continue;
            const ImageFeatures &fa = features[m.src_img_idx];
            const ImageFeatures &fb = features[m.dst_img_idx];
            std::vector<Vec3d> ri, rj;
            for (const DMatch &dm : m.matches) {
                Vec3d a = ray(fa.keypoints[dm.queryIdx].pt);
                Vec3d b = ray(fb.keypoints[dm.trainIdx].pt);
                if (m.src_img_idx != i) std::swap(a, b);
                ri.push_back(a);
                rj.push_back(b);
            }
            const int cnt = (int) ri.size();

            Matx33d best;
            int bestInliers = 0;
            for (int it = 0; it < 500; ++it) {
                int p = rng.uniform(0, cnt), q = rng.uniform(0, cnt);
                if (p == q) continue;
                Matx33d A;
                if (!rotationFrom2(ri[p], ri[q], rj[p], rj[q], A)) continue;
                int inl = 0;
                for (int k = 0; k < cnt; ++k)
                    if (angleBetweenDeg(ri[k], A * rj[k]) < inlierDeg) ++inl;
                if (inl > bestInliers) {
                    bestInliers = inl;
                    best = A;
                }
            }
            if (bestInliers < 15) continue;

            std::vector<Vec3d> ki, kj;
            for (int pass = 0; pass < 2; ++pass) {
                ki.clear();
                kj.clear();
                for (int k = 0; k < cnt; ++k)
                    if (angleBetweenDeg(ri[k], best * rj[k]) < inlierDeg) {
                        ki.push_back(ri[k]);
                        kj.push_back(rj[k]);
                    }
                if (ki.size() < 15) break;
                best = kabsch(ki, kj);
            }
            if (ki.size() < 15) continue;

            // Ảnh cho Ri^T Rj = A, cảm biến cho Ri^T Rj: lệch quá thì là khớp nhầm cả khối
            // (ví dụ trần tôn lặp hoa văn bị trượt vài gân).
            double dev = rotationAngleDeg(best, sensor[i].t() * sensor[j]);
            if (dev > kPairSensorGateDeg) {
                LOGW("Bỏ cặp %d-%d: lệch cảm biến %.1f độ", i, j, dev);
                continue;
            }
            ++pairsUsed;
            const size_t maxPerPair = 300;
            size_t step = std::max<size_t>(1, ki.size() / maxPerPair);
            for (size_t k = 0; k < ki.size(); k += step) out.push_back({i, j, ki[k], kj[k]});
        }
    return out;
}

// Tinh chỉnh góc của mọi tấm cùng lúc. Mỗi điểm khớp kéo hai tia về trùng nhau; mỗi tấm bị neo chặt
// theo trọng lực và neo lỏng theo hướng ngang của cảm biến.
std::vector<Matx33d> refineRotations(const std::vector<RayPair> &obs, const std::vector<Matx33d> &sensor,
                                     double focal, double huber) {
    const int n = (int) sensor.size();
    std::vector<Matx33d> R = sensor;
    const Vec3d down(0, 1, 0);  // trục y của toàn cảnh hướng xuống
    const double gScale = 1.0 / (kGravitySigmaDeg * CV_PI / 180.0);
    const double hScale = 1.0 / (kHeadingSigmaDeg * CV_PI / 180.0);

    auto cost = [&](const std::vector<Matx33d> &Rc) {
        double c = 0;
        for (const RayPair &o : obs) {
            double e = norm(Rc[o.i] * o.ui - Rc[o.j] * o.uj) * focal;
            c += e <= huber ? e * e : 2 * huber * e - huber * huber;
        }
        for (int k = 0; k < n; ++k) {
            Vec3d g = (Rc[k].t() * down - sensor[k].t() * down) * gScale;
            Vec3d h = logRot(Rc[k] * sensor[k].t()) * hScale;
            c += g.dot(g) + h.dot(h);
        }
        return c;
    };

    double lambda = 1e-3;
    double current = cost(R);
    const double initial = current;
    for (int iter = 0; iter < 60; ++iter) {
        Mat JtJ = Mat::zeros(3 * n, 3 * n, CV_64F);
        Mat Jte = Mat::zeros(3 * n, 1, CV_64F);
        auto addBlock = [&](int a, int b, const Matx33d &M) {
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c) JtJ.at<double>(3 * a + r, 3 * b + c) += M(r, c);
        };
        auto addVec = [&](int a, const Vec3d &v) {
            for (int r = 0; r < 3; ++r) Jte.at<double>(3 * a + r) += v[r];
        };
        for (const RayPair &o : obs) {
            Vec3d ri = R[o.i] * o.ui, rj = R[o.j] * o.uj;
            Vec3d e = (ri - rj) * focal;
            double en = norm(e);
            double w = en <= huber ? 1.0 : huber / en;
            // Cập nhật R <- exp(w) R  =>  d(R u)/dw = -[R u]x
            Matx33d Ji = skew(ri) * (-focal), Jj = skew(rj) * focal;
            addBlock(o.i, o.i, Ji.t() * Ji * w);
            addBlock(o.j, o.j, Jj.t() * Jj * w);
            addBlock(o.i, o.j, Ji.t() * Jj * w);
            addBlock(o.j, o.i, Jj.t() * Ji * w);
            addVec(o.i, Ji.t() * e * w);
            addVec(o.j, Jj.t() * e * w);
        }
        for (int k = 0; k < n; ++k) {
            // Trọng lực trong hệ camera: (exp(w)R)^T y ≈ R^T y + R^T [y]x w
            Vec3d g = (R[k].t() * down - sensor[k].t() * down) * gScale;
            Matx33d Jg = R[k].t() * skew(down) * gScale;
            addBlock(k, k, Jg.t() * Jg);
            addVec(k, Jg.t() * g);
            Vec3d h = logRot(R[k] * sensor[k].t()) * hScale;
            addBlock(k, k, Matx33d::eye() * (hScale * hScale));
            addVec(k, h * hScale);
        }

        bool improved = false;
        for (int tries = 0; tries < 8; ++tries) {
            Mat A = JtJ.clone();
            for (int d = 0; d < 3 * n; ++d) A.at<double>(d, d) *= (1.0 + lambda);
            Mat delta;
            solve(A, -Jte, delta, DECOMP_CHOLESKY);
            std::vector<Matx33d> cand(n);
            double maxStep = 0;
            for (int k = 0; k < n; ++k) {
                Vec3d w(delta.at<double>(3 * k), delta.at<double>(3 * k + 1), delta.at<double>(3 * k + 2));
                maxStep = std::max(maxStep, norm(w));
                cand[k] = expRot(w) * R[k];
            }
            double c = cost(cand);
            if (c < current) {
                R = cand;
                improved = (current - c) > 1e-7 * current && maxStep > 1e-7;
                current = c;
                lambda = std::max(1e-7, lambda / 3);
                break;
            }
            lambda *= 5;
        }
        if (!improved) break;
    }

    std::vector<double> errs;
    for (const RayPair &o : obs) errs.push_back(norm(R[o.i] * o.ui - R[o.j] * o.uj) * focal);
    double median = 0;
    if (!errs.empty()) {
        std::nth_element(errs.begin(), errs.begin() + errs.size() / 2, errs.end());
        median = errs[errs.size() / 2];
    }
    LOGI("Tinh chỉnh: %zu điểm, cost %.0f -> %.0f, sai số trung vị %.2f px (độ phân giải làm việc)",
         obs.size(), initial, current, median);
    for (int k = 0; k < n; ++k) {
        double dev = rotationAngleDeg(R[k], sensor[k]);
        if (dev > kMaxDeviationDeg) {
            LOGW("Ảnh %d lệch %.1f độ so với cảm biến, dùng góc cảm biến", k, dev);
            R[k] = sensor[k];
        }
    }
    return R;
}

}  // namespace sphere
