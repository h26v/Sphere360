#include "epipolar.h"

#include "params.h"

namespace sphere {

// RANSAC 2 điểm cho b (ràng buộc b·(u×v) = 0), sau đó tinh chỉnh xen kẽ omega và b có trọng số Cauchy.
EpiModel fitEpipolar(const std::vector<FlowSample> &S, double s, RNG &rng) {
    EpiModel M;
    std::vector<int> mov;
    for (int k = 0; k < (int) S.size(); ++k)
        if (std::hypot(S[k].f[0], S[k].f[1]) >= kEpiMinFlowPx) mov.push_back(k);
    M.moving = (int) mov.size();
    if (M.moving < kEpiMinSamples) return M;

    const double thr = kEpiInlierPx / s;
    std::vector<int> eval;
    const int stride = std::max(1, M.moving / 2000);
    for (int k = 0; k < M.moving; k += stride) eval.push_back(mov[k]);

    int best = 0;
    Vec3d bestB;
    const Vec3d zero(0, 0, 0);
    for (int it = 0; it < kEpiRansacIters; ++it) {
        const FlowSample &a = S[mov[rng.uniform(0, M.moving)]], &c = S[mov[rng.uniform(0, M.moving)]];
        Vec3d b = a.u.cross(a.v).cross(c.u.cross(c.v));
        double nb = norm(b);
        if (nb < 1e-12) continue;
        b *= 1.0 / nb;
        int cnt = 0;
        for (int k : eval) cnt += epiResidual(b, zero, S[k].u, S[k].v) < thr;
        if (cnt > best) {
            best = cnt;
            bestB = b;
        }
    }
    if (best < 8) return M;

    Vec3d b = bestB, om(0, 0, 0);
    for (int it = 0; it < 5; ++it) {
        // omega: b·(u×(v - omega×u)) = 0  <=>  omega·(b - (u·b)u) = b·(u×v), tuyến tính theo omega.
        Matx33d AtA = Matx33d::zeros();
        Vec3d Atb(0, 0, 0);
        double sw = 0;
        for (int k : mov) {
            const Vec3d &u = S[k].u, &v = S[k].v;
            double bu = norm(b.cross(u));
            if (bu < 1e-6) continue;
            double r = epiResidual(b, om, u, v) / thr;
            double w = S[k].conf / (1 + r * r);
            Vec3d a = (b - u * u.dot(b)) * (1.0 / bu);
            double rhs = b.dot(u.cross(v)) / bu;
            AtA += w * (a * a.t());
            Atb += w * rhs * a;
            sw += w;
        }
        if (sw <= 0) return M;
        AtA += Matx33d::eye() * (1e-6 * sw);
        om = AtA.solve(Atb, DECOMP_CHOLESKY);

        // b: vector riêng nhỏ nhất của tổng các hàng (u×(v - omega×u)) / |b×u|.
        Matx33d C = Matx33d::zeros();
        for (int k : mov) {
            const Vec3d &u = S[k].u;
            double bu = norm(b.cross(u));
            if (bu < 1e-6) continue;
            double r = epiResidual(b, om, u, S[k].v) / thr;
            double w = S[k].conf / (1 + r * r);
            Vec3d row = u.cross(S[k].v - om.cross(u)) * (1.0 / bu);
            C += w * (row * row.t());
        }
        Mat vals, vecs;
        eigen(Mat(C), vals, vecs);
        Vec3d nb(vecs.at<double>(2, 0), vecs.at<double>(2, 1), vecs.at<double>(2, 2));
        if (nb.dot(b) < 0) nb = -nb;
        b = nb * (1.0 / norm(nb));
    }

    for (int k : mov) M.inliers += epiResidual(b, om, S[k].u, S[k].v) < thr;
    M.b = b;
    M.omega = om;
    M.ok = M.inliers >= kEpiMinSamples && M.inliers >= kEpiMinInlierRatio * M.moving;
    return M;
}

// Flow (pixel) theo mô hình: phần xoay dư cộng một đoạn trượt dọc đường epipolar. perpPx = phần flow đo
// được nằm ngoài mô hình.
Vec2f constrainFlow(const EpiModel &M, const FlowSample &S, double s, float &perpPx) {
    const Vec3d &u = S.u;
    Vec2d fom = tangentToPx(u, M.omega.cross(u), s);
    Vec2d f(S.f[0], S.f[1]);
    Vec3d t = M.b - u * u.dot(M.b);
    double nt = norm(t);
    Vec2d fc = fom;
    if (nt > 1e-6) {
        Vec2d e = tangentToPx(u, t * (1.0 / nt), s);
        double ne = norm(e);
        if (ne > 1e-9) {
            e *= 1.0 / ne;
            fc += e * (f - fom).dot(e);
        }
    }
    perpPx = (float) norm(f - fc);
    return Vec2f((float) fc[0], (float) fc[1]);
}

}  // namespace sphere
