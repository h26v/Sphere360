#include "bridge.h"

#include <map>

#include "geometry.h"
#include "params.h"
#include "warp.h"

namespace sphere {

namespace {

Mat loadGrayUpright(const std::string &path, int rotationCw) {
    Mat img = imread(path, IMREAD_GRAYSCALE);
    if (img.empty()) return img;
    switch ((rotationCw % 360 + 360) % 360) {
        case 90: rotate(img, img, ROTATE_90_CLOCKWISE); break;
        case 180: rotate(img, img, ROTATE_180); break;
        case 270: rotate(img, img, ROTATE_90_COUNTERCLOCKWISE); break;
        default: break;
    }
    return img;
}

// Cắt view theo vùng E của khung flow (thử cả bản dịch một vòng để phủ chỗ nối kinh tuyến 180°).
void cropView(const View &v, Rect E, int width, Mat &g, Mat &m) {
    g = Mat::zeros(E.size(), CV_8U);
    m = Mat::zeros(E.size(), CV_8U);
    if (v.gray.empty()) return;
    for (int off : {0, -width, width}) {
        Rect placed(v.tl + Point(off, 0), v.gray.size());
        Rect r = placed & E;
        if (r.empty()) continue;
        Rect src(r.tl() - placed.tl(), r.size()), dst(r.tl() - E.tl(), r.size());
        v.gray(src).copyTo(g(dst), v.mask(src));
        v.mask(src).copyTo(m(dst), v.mask(src));
    }
}

inline bool sample2(const Mat &f, float x, float y, Vec2f &out) {
    int x0 = (int) std::floor(x), y0 = (int) std::floor(y);
    if (x0 < 0 || y0 < 0 || x0 + 1 >= f.cols || y0 + 1 >= f.rows) return false;
    float ax = x - x0, ay = y - y0;
    const Vec2f *r0 = f.ptr<Vec2f>(y0), *r1 = f.ptr<Vec2f>(y0 + 1);
    out = (r0[x0] * (1 - ax) + r0[x0 + 1] * ax) * (1 - ay) + (r1[x0] * (1 - ax) + r1[x0 + 1] * ax) * ay;
    return true;
}

inline bool maskAt(const Mat &m, float x, float y) {
    int xi = cvRound(x), yi = cvRound(y);
    return xi >= 0 && yi >= 0 && xi < m.cols && yi < m.rows && m.at<uchar>(yi, xi);
}

// Dò từ views.front() tới views.back(), mỗi bước một flow DIS giữa hai ảnh liền nhau (kiểm tra hai chiều).
// F: độ dịch tổng (pixel khung flow) cho từng điểm của ov, valid: điểm dò được tới cuối chuỗi.
void chainFlow(const std::vector<View> &views, Rect ov, int width, int fullH, double degPx, Mat &F, Mat &valid,
               const std::string &debugPath = "") {
    const int margin = (int) std::ceil(kBridgeMaxDeg * degPx);
    Rect E(ov.x - margin, ov.y - margin, ov.width + 2 * margin, ov.height + 2 * margin);
    E &= Rect(ov.x - 4 * margin - width, 0, ov.width + 8 * margin + 2 * width, fullH);
    Mat X(ov.size(), CV_32FC2);
    for (int y = 0; y < ov.height; ++y)
        for (int x = 0; x < ov.width; ++x) X.at<Vec2f>(y, x) = Vec2f(ov.x + x - E.x, ov.y + y - E.y);
    const Mat X0 = X.clone();
    valid = Mat(ov.size(), CV_8U, Scalar(255));

    Ptr<DISOpticalFlow> dis = DISOpticalFlow::create(DISOpticalFlow::PRESET_MEDIUM);
    const float tol = (float) kBridgeConsistPx;
    Mat g0, m0, g1, m1, f, bk;
    cropView(views[0], E, width, g0, m0);
    // Ảnh debug: mỗi ảnh của chuỗi (vùng E), chấm xanh là điểm còn dò được, chấm đỏ là điểm mất ở bước này.
    std::vector<Mat> dbg;
    auto drawStep = [&](const Mat &g, const Mat &lost) {
        if (debugPath.empty()) return;
        Mat c;
        cvtColor(g, c, COLOR_GRAY2BGR);
        for (int y = 0; y < ov.height; y += 6)
            for (int x = 0; x < ov.width; x += 6) {
                Vec2f pt = X.at<Vec2f>(y, x);
                Point ip(cvRound(pt[0]), cvRound(pt[1]));
                if (valid.at<uchar>(y, x)) circle(c, ip, 1, Scalar(0, 255, 0), -1);
                else if (!lost.empty() && lost.at<uchar>(y, x)) circle(c, ip, 1, Scalar(0, 0, 255), -1);
            }
        dbg.push_back(c);
    };
    drawStep(g0, Mat());
    for (size_t k = 1; k < views.size(); ++k) {
        const Mat before = valid.clone();
        cropView(views[k], E, width, g1, m1);
        dis->calc(g0, g1, f);
        dis->calc(g1, g0, bk);
        for (int y = 0; y < ov.height; ++y) {
            uchar *vr = valid.ptr<uchar>(y);
            Vec2f *xr = X.ptr<Vec2f>(y);
            for (int x = 0; x < ov.width; ++x) {
                if (!vr[x]) continue;
                Vec2f pt = xr[x], d, back;
                if (!maskAt(m0, pt[0], pt[1]) || !sample2(f, pt[0], pt[1], d)) {
                    vr[x] = 0;
                    continue;
                }
                Vec2f p1 = pt + d;
                if (!maskAt(m1, p1[0], p1[1]) || !sample2(bk, p1[0], p1[1], back) ||
                    std::abs(d[0] + back[0]) > tol || std::abs(d[1] + back[1]) > tol) {
                    vr[x] = 0;
                    continue;
                }
                xr[x] = p1;
            }
        }
        if (!debugPath.empty()) drawStep(g1, before & ~valid);
        std::swap(g0, g1);
        std::swap(m0, m1);
    }
    F = X - X0;
    if (!dbg.empty()) {
        // Xếp thành lưới 4 cột.
        const int cols = 4, rows = ((int) dbg.size() + cols - 1) / cols;
        Mat grid(rows * E.height, cols * E.width, CV_8UC3, Scalar::all(40));
        for (size_t k = 0; k < dbg.size(); ++k)
            dbg[k].copyTo(grid(Rect((int) (k % cols) * E.width, (int) (k / cols) * E.height, E.width, E.height)));
        imwrite(debugPath, grid, {IMWRITE_JPEG_QUALITY, 85});
    }
}

// Điền chỗ F không hợp lệ bằng trung bình có trọng số của lân cận (flow khởi đầu cho DIS phải dày đặc).
void fillFlow(Mat &F, const Mat &valid, double sigma) {
    Mat w;
    valid.convertTo(w, CV_32F, 1.0 / 255);
    std::vector<Mat> ch;
    split(F, ch);
    Mat ws;
    GaussianBlur(w, ws, Size(), sigma);
    for (Mat &c : ch) {
        Mat cs;
        GaussianBlur(c.mul(w), cs, Size(), sigma);
        divide(cs, ws + 1e-4, cs);
        cs.copyTo(c, w < 0.5);
    }
    merge(ch, F);
}

// Tinh chỉnh flow khởi đầu Fc giữa a và b: kéo b về theo Fc, đo phần lệch còn lại R bằng DIS, rồi
// F(x) = R(x) + Fc(x + R(x)).
void refineFlow(const Mat &a, const Mat &b, const Mat &Fc, DISOpticalFlow &dis, Mat &F) {
    Mat map(Fc.size(), CV_32FC2);
    for (int y = 0; y < Fc.rows; ++y) {
        const Vec2f *fr = Fc.ptr<Vec2f>(y);
        Vec2f *mr = map.ptr<Vec2f>(y);
        for (int x = 0; x < Fc.cols; ++x) mr[x] = Vec2f((float) x, (float) y) + fr[x];
    }
    Mat bw, R;
    remap(b, bw, map, noArray(), INTER_LINEAR, BORDER_CONSTANT, Scalar(0));
    dis.calc(a, bw, R);
    F.create(Fc.size(), CV_32FC2);
    for (int y = 0; y < Fc.rows; ++y) {
        const Vec2f *rr = R.ptr<Vec2f>(y), *fr = Fc.ptr<Vec2f>(y);
        Vec2f *out = F.ptr<Vec2f>(y);
        for (int x = 0; x < Fc.cols; ++x) {
            Vec2f c;
            if (!sample2(Fc, x + rr[x][0], y + rr[x][1], c)) c = fr[x];
            out[x] = rr[x] + c;
        }
    }
}

// Đường ít bước nhất (BFS) từ tấm đầu tới tấm cuối qua các khung cands, mỗi bước xoay không quá
// kBridgeStepDeg. Trả về chỉ số trong cands theo thứ tự đi; rỗng nếu không có đường (không tính bước nhảy
// thẳng từ tấm tới tấm).
std::vector<int> pickPath(const std::vector<Matx33d> &cands, const Matx33d &from, const Matx33d &to) {
    const int n = (int) cands.size();
    const int start = n, end = n + 1;
    auto rotOf = [&](int v) -> const Matx33d & {
        if (v == start) return from;
        if (v == end) return to;
        return cands[v];
    };
    std::vector<int> prev(n + 2, -2);
    std::vector<int> queue = {start};
    prev[start] = -1;
    for (size_t head = 0; head < queue.size(); ++head) {
        const int u = queue[head];
        if (u == end) break;
        for (int v = 0; v < n + 2; ++v) {
            if (prev[v] != -2 || v == start || (u == start && v == end)) continue;
            if (rotationAngleDeg(rotOf(u), rotOf(v)) > kBridgeStepDeg) continue;
            prev[v] = u;
            queue.push_back(v);
        }
    }
    std::vector<int> path;
    if (prev[end] == -2) return path;
    for (int v = prev[end]; v != start; v = prev[v]) path.push_back(v);
    std::reverse(path.begin(), path.end());
    return path;
}

// Điểm SIFT của một ảnh, đổi thành tia đơn vị trong hệ camera.
struct NodeFeat {
    std::vector<Vec3d> rays;
    Mat desc;
};

NodeFeat nodeFeatures(const Mat &gray, double focal, Feature2D &finder) {
    NodeFeat f;
    if (gray.empty()) return f;
    std::vector<KeyPoint> kp;
    finder.detectAndCompute(gray, noArray(), kp, f.desc);
    Matx33d K;
    intrinsics(focal, gray.size()).convertTo(Mat(K, false), CV_64F);
    const Matx33d Kinv = K.inv();
    for (const KeyPoint &k : kp) {
        Vec3d v = Kinv * Vec3d(k.pt.x, k.pt.y, 1);
        f.rays.push_back(v * (1.0 / norm(v)));
    }
    return f;
}

// Phép xoay A (tia của b đổi sang hệ a) giữa hai ảnh liền nhau của chuỗi.
bool stepRotation(const NodeFeat &a, const NodeFeat &b, double inlierDeg, RNG &rng, Matx33d &A, int &inliers) {
    inliers = 0;
    if (a.desc.rows < 2 || b.desc.rows < 2) return false;
    std::vector<std::vector<DMatch>> knn;
    BFMatcher(NORM_L2).knnMatch(a.desc, b.desc, knn, 2);
    std::vector<Vec3d> ra, rb;
    for (const auto &m : knn)
        if (m.size() == 2 && m[0].distance < 0.8f * m[1].distance) {
            ra.push_back(a.rays[m[0].queryIdx]);
            rb.push_back(b.rays[m[0].trainIdx]);
        }
    return robustRotation(ra, rb, inlierDeg, kBridgeMinInliers, rng, A, inliers);
}

}  // namespace

std::vector<RelRotation> bridgeRotations(const BridgeInput &in, const std::vector<Matx33d> &sensor,
                                         const std::vector<Mat> &stillGray, double matchFocal) {
    std::vector<RelRotation> out;
    const int n = (int) sensor.size();
    const int m = (int) in.paths.size();
    if (m == 0 || in.focal <= 0 || n < 2) return out;
    std::vector<Matx33d> S(m);
    for (int k = 0; k < m; ++k) in.sensorR[k].convertTo(Mat(S[k], false), CV_64F);
    std::vector<std::vector<int>> gap(n);
    for (int k = 0; k < m; ++k)
        if (in.after[k] >= 0 && in.after[k] < n) gap[in.after[k]].push_back(k);

    Ptr<SIFT> sift = SIFT::create(kBridgeMatchFeatures);
    const double scale = matchFocal / in.focal;
    // ~1.5 pixel ở ảnh so khớp: hai ảnh liền nhau của chuỗi gần như cùng chỗ nên đòi khít được.
    const double inlierDeg = 1.5 / matchFocal * 180.0 / CV_PI;
    std::map<int, NodeFeat> bridgeFeat;  // chỉ giữ khung của các khoảng còn dùng tới
    std::vector<NodeFeat> stillFeat(n);
    std::vector<bool> stillDone(n, false);
    RNG rng(20261023);
    int chains = 0, steps = 0, measured = 0;
    for (int a = 0; a + 1 < n; ++a) {
        std::vector<int> idx;
        for (int g = std::max(0, a - 1); g <= std::min(n - 1, a + 1); ++g)
            for (int k : gap[g]) idx.push_back(k);
        // Bỏ đặc trưng của khung không còn dùng tới (thuộc khoảng trước a - 1).
        for (auto it = bridgeFeat.begin(); it != bridgeFeat.end();)
            it = in.after[it->first] < a - 1 ? bridgeFeat.erase(it) : std::next(it);
        if (idx.empty()) continue;
        std::vector<Matx33d> rots;
        for (int k : idx) rots.push_back(S[k]);
        const std::vector<int> path = pickPath(rots, sensor[a], sensor[a + 1]);
        if (path.empty()) continue;

        auto stillFeature = [&](int i) -> const NodeFeat & {
            if (!stillDone[i]) {
                stillFeat[i] = nodeFeatures(stillGray[i], matchFocal, *sift);
                stillDone[i] = true;
            }
            return stillFeat[i];
        };
        auto bridgeFeature = [&](int k) -> const NodeFeat & {
            auto it = bridgeFeat.find(k);
            if (it != bridgeFeat.end()) return it->second;
            Mat g = loadGrayUpright(in.paths[k], in.rotationCw), small;
            if (!g.empty()) resize(g, small, Size(), scale, scale, INTER_AREA);
            return bridgeFeat[k] = nodeFeatures(small, matchFocal, *sift);
        };
        // Các nút của chuỗi: tấm a, các khung, tấm a + 1 (kèm góc con quay để thế cho bước không đo được).
        std::vector<const NodeFeat *> nodes = {&stillFeature(a)};
        std::vector<Matx33d> nodeR = {sensor[a]};
        for (int p : path) {
            nodes.push_back(&bridgeFeature(idx[p]));
            nodeR.push_back(S[idx[p]]);
        }
        nodes.push_back(&stillFeature(a + 1));
        nodeR.push_back(sensor[a + 1]);

        Matx33d total = Matx33d::eye();
        int good = 0;
        const int cnt = (int) nodes.size() - 1;
        for (int s = 0; s < cnt; ++s) {
            Matx33d A;
            int inl = 0;
            if (stepRotation(*nodes[s], *nodes[s + 1], inlierDeg, rng, A, inl)) {
                ++good;
            } else {
                A = nodeR[s].t() * nodeR[s + 1];
            }
            total = total * A;
        }
        steps += cnt;
        measured += good;
        const double weight = (double) good / cnt;
        const Matx33d sensorRel = sensor[a].t() * sensor[a + 1];
        LOGI("Góc %d-%d qua chuỗi %d bước (%d đo bằng ảnh): %.1f°, con quay %.1f°, lệch nhau %.1f°", a, a + 1, cnt,
             good, rotationAngleDeg(Matx33d::eye(), total), rotationAngleDeg(Matx33d::eye(), sensorRel),
             rotationAngleDeg(total, sensorRel));
        if (weight < 0.5) continue;
        out.push_back({a, a + 1, total, weight});
        ++chains;
    }
    LOGI("Đo góc qua khung cầu nối: %d cặp tấm liền nhau, %d/%d bước đo được bằng ảnh", chains, measured, steps);
    return out;
}

BridgeChains loadBridges(const BridgeInput &in, const std::vector<Matx33d> &sensor,
                         const std::vector<Matx33d> &refined, int flowWidth) {
    const int n = (int) sensor.size();
    BridgeChains chains(std::max(0, n - 1));
    if (in.paths.empty() || in.focal <= 0 || n < 2) return chains;
    const double scale = std::min(1.0, flowWidth / (2 * CV_PI * in.focal));
    // Gom theo khoảng giữa hai tấm, giữ thứ tự thời gian.
    std::vector<std::vector<int>> gap(n - 1);
    for (size_t k = 0; k < in.paths.size(); ++k) {
        int a = in.after[k];
        if (a >= 0 && a + 1 < n) gap[a].push_back((int) k);
    }
    int loaded = 0;
    for (int a = 0; a + 1 < n; ++a) {
        if (gap[a].empty()) continue;
        // Độ lệch giữa góc cảm biến và góc tinh chỉnh ở hai đầu: C = refined * sensor^T.
        chains[a].Ra = refined[a];
        chains[a].Rb = refined[a + 1];
        const Matx33d Ca = refined[a] * sensor[a].t(), Cb = refined[a + 1] * sensor[a + 1].t();
        const Vec3d dw = logRot(Ca.t() * Cb);
        const int cnt = (int) gap[a].size();
        for (int idx = 0; idx < cnt; ++idx) {
            const int k = gap[a][idx];
            Mat g = loadGrayUpright(in.paths[k], in.rotationCw);
            if (g.empty()) continue;
            BridgeFrame bf;
            resize(g, bf.gray, Size(), scale, scale, INTER_AREA);
            bf.K = intrinsics(in.focal * scale, bf.gray.size());
            Matx33d S;
            in.sensorR[k].convertTo(Mat(S, false), CV_64F);
            const Matx33d C = Ca * expRot(dw * ((idx + 1.0) / (cnt + 1.0)));
            bf.Rd = C * S;
            Mat(bf.Rd).convertTo(bf.R, CV_32F);
            chains[a].frames.push_back(bf);
            ++loaded;
        }
    }
    LOGI("Khung cầu nối: nạp %d/%zu khung, thu nhỏ %.2f lần", loaded, in.paths.size(), scale);
    return chains;
}

double bridgeFlows(const BridgeChains &chains, int gap, bool forward, const View &p, const View &q, Rect ov,
                   int width, const Mat &a, const Mat &b, Mat &F, Mat &G, int &hops) {
    const double s = width / (2 * CV_PI);
    const double degPx = s * CV_PI / 180.0;
    const int fullH = (int) std::ceil(CV_PI * s);
    const BridgeChain &chain = chains[gap];
    std::vector<const BridgeFrame *> cands;
    for (int g = std::max(0, gap - 1); g <= std::min((int) chains.size() - 1, gap + 1); ++g)
        for (const BridgeFrame &bf : chains[g].frames) cands.push_back(&bf);
    std::vector<Matx33d> rots;
    for (const BridgeFrame *c : cands) rots.push_back(c->Rd);
    const std::vector<int> path = forward ? pickPath(rots, chain.Ra, chain.Rb) : pickPath(rots, chain.Rb, chain.Ra);
    hops = path.empty() ? 0 : (int) path.size() + 1;
    if (!gDebugStem.empty()) {
        const Matx33d &from = forward ? chain.Ra : chain.Rb, &to = forward ? chain.Rb : chain.Ra;
        std::string steps;
        const Matx33d *prev = &from;
        for (int k : path) {
            steps += " " + std::to_string((int) std::lround(rotationAngleDeg(*prev, cands[k]->Rd) * 10));
            prev = &cands[k]->Rd;
        }
        steps += " " + std::to_string((int) std::lround(rotationAngleDeg(*prev, to) * 10));
        LOGI("Chuỗi %d-%d: hai tấm lệch %.1f°, các bước (0.1°):%s", forward ? gap : gap + 1, forward ? gap + 1 : gap,
             rotationAngleDeg(from, to), steps.c_str());
    }
    Ptr<DISOpticalFlow> dis = DISOpticalFlow::create(DISOpticalFlow::PRESET_MEDIUM);
    if (path.empty()) {
        dis->calc(a, b, F);
        dis->calc(b, a, G);
        return 0;
    }
    std::vector<View> views;
    views.push_back(p);
    for (int k : path) {
        const BridgeFrame &bf = *cands[k];
        View v;
        v.tl = warpSphere(bf.gray, bf.K, bf.R, s, v.gray, v.mask);
        if (!v.gray.empty()) views.push_back(v);
    }
    views.push_back(q);

    Mat Fc, Vf, Gc, Vg;
    std::string debugPath;
    if (!gDebugStem.empty()) debugPath = gDebugStem + "_chain_" + std::to_string(forward ? gap : gap + 1) + "_" +
                                         std::to_string(forward ? gap + 1 : gap) + ".jpg";
    chainFlow(views, ov, width, fullH, degPx, Fc, Vf, debugPath);
    std::reverse(views.begin(), views.end());
    chainFlow(views, ov, width, fullH, degPx, Gc, Vg);
    const double cover = (double) countNonZero(Vf) / std::max(1, ov.area());
    if (cover < kBridgeMinCover) {
        dis->calc(a, b, F);
        dis->calc(b, a, G);
        return cover;
    }
    const double sigma = kBridgeFillDeg * degPx;
    fillFlow(Fc, Vf, sigma);
    fillFlow(Gc, Vg, sigma);
    refineFlow(a, b, Fc, *dis, F);
    refineFlow(b, a, Gc, *dis, G);
    return cover;
}

}  // namespace sphere
