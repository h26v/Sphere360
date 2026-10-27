// Ghép ảnh cầu 360x180 (equirectangular 2:1) từ nhiều tấm chụp có kèm hướng cảm biến.
//
// Các bước (tham khảo cách Google Photo Sphere / Street View làm):
//  1. Tìm điểm đặc trưng SIFT, chỉ so khớp các cặp ảnh nằm cạnh nhau theo cảm biến.
//  2. Mỗi cặp: RANSAC 2 điểm tìm phép xoay tương đối, bỏ khớp nhầm.
//  3. Tinh chỉnh góc của mọi tấm cùng lúc (Levenberg–Marquardt, Huber). Neo chặt theo trọng lực
//     (cảm biến đo chiều "xuống" rất chuẩn), neo lỏng theo hướng ngang (con quay trôi theo thời gian).
//  4. Thị sai: ở mỗi vùng chồng, tính optical flow giữa hai ảnh; giải một trường uốn chung cho mọi ảnh để
//     nội dung các ảnh gặp nhau, độ uốn nhạt dần ra xa vùng chồng; lượt hai đo flow còn lại rồi bù.
//  5. Cân bằng sáng, tìm đường nối bằng graph cut trên ảnh nhỏ (né chỗ còn lệch sau khi uốn), giao vùng
//     đỉnh/đáy cho tấm chụp thẳng lên/xuống, nối vòng ở kinh tuyến 180°.
//  6. Dựng ảnh ở độ phân giải đầy đủ và trộn nhiều dải tần (multi-band).

#include "stitcher.h"

#include "alignment.h"
#include "bridge.h"
#include "flow.h"
#include "geometry.h"
#include "params.h"
#include "quality.h"
#include "seams.h"
#include "warp.h"

namespace sphere {

namespace {

// Ảnh debug ở khung đường nối: trên là ảnh ghép thô kèm đường nối (đỏ), dưới là thị sai lớn nhất của các
// cặp (đen = khớp, sáng = lệch nhiều).
void writeSeamDebug(const std::string &path, const std::vector<UMat> &warped, const std::vector<UMat> &masks,
                    const std::vector<Point> &corners, const std::vector<PairParallax> &parallax, int width) {
    const int H = width / 2, W = width + width / 8, ox = width / 2;
    Mat comp(H, W, CV_8UC3, Scalar::all(0)), owner(H, W, CV_32S, Scalar(-1)), par(H, W, CV_32F, Scalar(0));
    for (size_t k = 0; k < warped.size(); ++k) {
        Mat img = warped[k].getMat(ACCESS_READ), m = masks[k].getMat(ACCESS_READ);
        for (int y = 0; y < img.rows; ++y)
            for (int x = 0; x < img.cols; ++x) {
                int gx = corners[k].x + x + ox, gy = corners[k].y + y;
                if (gx < 0 || gy < 0 || gx >= W || gy >= H || !m.at<uchar>(y, x)) continue;
                comp.at<Vec3b>(gy, gx) = img.at<Vec3b>(y, x);
                owner.at<int>(gy, gx) = (int) k;
            }
    }
    for (const PairParallax &pp : parallax)
        for (int y = 0; y < pp.deg.rows; ++y)
            for (int x = 0; x < pp.deg.cols; ++x) {
                int gx = pp.roi.x + x + ox, gy = pp.roi.y + y;
                if (gx < 0 || gy < 0 || gx >= W || gy >= H) continue;
                float &v = par.at<float>(gy, gx);
                v = std::max(v, pp.deg.at<float>(y, x));
            }
    for (int y = 0; y + 1 < H; ++y)
        for (int x = 0; x + 1 < W; ++x) {
            int o = owner.at<int>(y, x);
            if (o >= 0 && ((owner.at<int>(y, x + 1) >= 0 && owner.at<int>(y, x + 1) != o) ||
                           (owner.at<int>(y + 1, x) >= 0 && owner.at<int>(y + 1, x) != o)))
                comp.at<Vec3b>(y, x) = Vec3b(0, 0, 255);
        }
    Mat p8, pc;
    par.convertTo(p8, CV_8U, 255.0 / (kSeamParallaxCapDeg - kSeamParallaxFloorDeg));
    applyColorMap(p8, pc, COLORMAP_INFERNO);
    Mat out;
    vconcat(comp, pc, out);
    imwrite(path, out, {IMWRITE_JPEG_QUALITY, 90});
}

// Mặt nạ đi kèm ảnh (frame_XX_mask.png cạnh frame_XX.jpg): điểm nào có dữ liệu. Dùng khi ảnh được dựng lại từ
// một tâm khác (vd theo độ sâu) nên có chỗ trống. Không có file thì trả về rỗng.
Mat loadMask(const std::string &path, Size size) {
    const size_t dot = path.find_last_of('.');
    Mat m = imread((dot == std::string::npos ? path : path.substr(0, dot)) + "_mask.png", IMREAD_GRAYSCALE);
    if (m.empty()) return m;
    if (m.cols > m.rows && size.width < size.height) rotate(m, m, ROTATE_90_CLOCKWISE);
    if (m.size() != size) resize(m, m, size, 0, 0, INTER_NEAREST);
    return m;
}

Mat loadUpright(const std::string &path) {
    // IMREAD_COLOR áp dụng luôn EXIF orientation.
    Mat img = imread(path, IMREAD_COLOR);
    if (img.empty()) return img;
    // Ảnh luôn chụp dọc; nếu vẫn ngang thì EXIF không có, tự xoay.
    if (img.cols > img.rows) rotate(img, img, ROTATE_90_CLOCKWISE);
    return img;
}

// Điền chỗ trống của ảnh theo kiểu push-pull: thu nhỏ dần (trung bình có trọng số các điểm có dữ liệu), rồi
// phóng ngược lên, chỗ trống ở mỗi tầng lấy từ tầng thô hơn. Ra màu mượt nối liền với xung quanh.
void pushPullFill(Mat &img, const Mat &known) {
    std::vector<Mat> cols, wts;
    Mat c, w;
    img.convertTo(c, CV_32FC3);
    known.convertTo(w, CV_32F, 1.0 / 255);
    {
        std::vector<Mat> ch;
        split(c, ch);
        for (Mat &x : ch) x = x.mul(w);
        merge(ch, c);
    }
    cols.push_back(c);
    wts.push_back(w);
    while (std::min(cols.back().cols, cols.back().rows) > 4) {
        Mat c2, w2;
        resize(cols.back(), c2, Size(), 0.5, 0.5, INTER_AREA);
        resize(wts.back(), w2, Size(), 0.5, 0.5, INTER_AREA);
        cols.push_back(c2);
        wts.push_back(w2);
    }
    // Tầng thô nhất: chỗ nào vẫn trống thì lấy trung bình cả tầng.
    Mat filled;
    {
        Mat &cc = cols.back(), &ww = wts.back();
        const double sw = sum(ww)[0];
        Scalar mean = sw > 0 ? sum(cc) * (1.0 / sw) : Scalar::all(0);
        filled = Mat(cc.size(), CV_32FC3);
        for (int y = 0; y < cc.rows; ++y)
            for (int x = 0; x < cc.cols; ++x) {
                const float wv = ww.at<float>(y, x);
                const Vec3f v = cc.at<Vec3f>(y, x);
                filled.at<Vec3f>(y, x) = wv > 1e-4f ? v / wv : Vec3f((float) mean[0], (float) mean[1], (float) mean[2]);
            }
    }
    for (int l = (int) cols.size() - 2; l >= 0; --l) {
        Mat up;
        resize(filled, up, cols[l].size(), 0, 0, INTER_LINEAR);
        Mat &cc = cols[l], &ww = wts[l];
        filled = Mat(cc.size(), CV_32FC3);
        for (int y = 0; y < cc.rows; ++y)
            for (int x = 0; x < cc.cols; ++x) {
                // Có đủ dữ liệu (trọng số >= 1) thì giữ, thiếu thì trộn thêm từ tầng thô.
                const float wv = std::min(1.f, ww.at<float>(y, x));
                const Vec3f own = ww.at<float>(y, x) > 1e-4f ? cc.at<Vec3f>(y, x) / ww.at<float>(y, x) : Vec3f();
                filled.at<Vec3f>(y, x) = own * wv + up.at<Vec3f>(y, x) * (1 - wv);
            }
    }
    filled.convertTo(img, img.type());
}

// Lấp chỗ không ảnh nào phủ quanh hai cực (khi không chụp tấm đỉnh / đáy). Vùng quanh cực được chiếu ra mặt
// phẳng tiếp xúc (ở equirect cực bị kéo thành cả một hàng), điền push-pull rồi chiếu ngược vào đúng chỗ trống.
void fillPoleHoles(Mat &pano, const Mat &covered) {
    const int W = pano.cols, H = pano.rows;
    const double s = W / (2 * CV_PI);
    for (int pole = -1; pole <= 1; pole += 2) {
        // Chỗ trống trong 60° quanh cực (máy không ngẩng/cúi tới thì chỗ hở có thể xa cực tới ~50°): góc lớn nhất tới cực.
        const int band = (int) std::ceil(s * 60 * CV_PI / 180);
        const int y0 = pole < 0 ? 0 : std::max(0, H - band), y1 = pole < 0 ? std::min(H, band) : H;
        double maxPolar = -1;
        int holes = 0;
        for (int y = y0; y < y1; ++y) {
            const uchar *cr = covered.ptr<uchar>(y);
            for (int x = 0; x < W; ++x)
                if (!cr[x]) {
                    ++holes;
                    const double polar = pole < 0 ? (y + 0.5) / s : (H - y - 0.5) / s;
                    maxPolar = std::max(maxPolar, polar);
                }
        }
        if (holes == 0) continue;
        // Mặt phẳng tiếp xúc tại cực, phủ thêm 8° quanh chỗ trống để có màu xung quanh.
        const double R = std::min(80.0 * CV_PI / 180, maxPolar + 8 * CV_PI / 180);
        const int N = 1024;
        const double f = N / 2.0 / std::tan(R);
        // Điểm (X, Z) trên mặt phẳng <-> hướng (X, pole * f, Z) trong hệ toàn cảnh (y hướng xuống).
        Mat mx(N, N, CV_32F), my(N, N, CV_32F);
        for (int j = 0; j < N; ++j)
            for (int i = 0; i < N; ++i) {
                const double X = i - N / 2.0 + 0.5, Z = j - N / 2.0 + 0.5;
                const double nrm = std::sqrt(X * X + f * f + Z * Z);
                const double dy = pole * f / nrm;
                mx.at<float>(j, i) = (float) (s * std::atan2(X, Z) + W / 2.0);
                my.at<float>(j, i) = (float) (s * (CV_PI - std::acos(std::max(-1.0, std::min(1.0, dy)))));
            }
        Mat plane, known;
        remap(pano, plane, mx, my, INTER_LINEAR, BORDER_WRAP);
        remap(covered, known, mx, my, INTER_NEAREST, BORDER_WRAP);
        erode(known, known, Mat());  // bỏ viền mép chỗ trống (màu trộn với đen)
        pushPullFill(plane, known);
        // Chiếu ngược: chỉ ghi vào chỗ trống.
        for (int y = y0; y < y1; ++y) {
            const uchar *cr = covered.ptr<uchar>(y);
            Vec3b *pr = pano.ptr<Vec3b>(y);
            const double ph = std::max(1e-6, std::min(CV_PI - 1e-6, (y + 0.5) / s));
            for (int x = 0; x < W; ++x) {
                if (cr[x]) continue;
                const double th = (x + 0.5 - W / 2.0) / s;
                const double dx = std::sin(ph) * std::sin(th), dz = std::sin(ph) * std::cos(th);
                const double dy = -std::cos(ph);
                if (dy * pole <= 1e-6) continue;
                const double px = f * dx / (dy * pole) + N / 2.0 - 0.5, pz = f * dz / (dy * pole) + N / 2.0 - 0.5;
                const int ix = (int) std::floor(px), iz = (int) std::floor(pz);
                if (ix < 0 || iz < 0 || ix + 1 >= N || iz + 1 >= N) continue;
                const float ax = (float) (px - ix), az = (float) (pz - iz);
                Vec3f v = Vec3f(plane.at<Vec3b>(iz, ix)) * (1 - ax) * (1 - az) +
                          Vec3f(plane.at<Vec3b>(iz, ix + 1)) * ax * (1 - az) +
                          Vec3f(plane.at<Vec3b>(iz + 1, ix)) * (1 - ax) * az +
                          Vec3f(plane.at<Vec3b>(iz + 1, ix + 1)) * ax * az;
                pr[x] = Vec3b(saturate_cast<uchar>(v[0]), saturate_cast<uchar>(v[1]), saturate_cast<uchar>(v[2]));
            }
        }
        LOGI("Lấp vùng %s không có ảnh: %d điểm, tới %.1f° quanh cực", pole < 0 ? "đỉnh trời" : "mặt đất", holes,
             maxPolar * 180 / CV_PI);
    }
}

// ------------------------------------------------------------------------------ hình học

}  // namespace

std::string stitch(const std::vector<std::string> &paths, const std::vector<Mat> &sensorR,
                   double focalFull, const BridgeInput &bridges, int outWidth, int jpegQuality, bool deghost,
                   const std::string &outPath, const Progress &progress) {
    const int n = static_cast<int>(paths.size());
    if (n < 2) return "Cần ít nhất 2 ảnh";
    outWidth = std::max(1024, outWidth / 2 * 2);
    const int outHeight = outWidth / 2;
    const double nativeWidth = 2 * CV_PI * focalFull;
    const double marginRad = kWrapMarginDeg * CV_PI / 180.0;
    {
        const size_t dot = outPath.find_last_of('.');
        gDebugStem = !kDebugSeams ? "" : dot == std::string::npos ? outPath : outPath.substr(0, dot);
    }

    // 1. Đặc trưng ở độ phân giải làm việc, kèm bản nhỏ cho bước flow và bước đường nối.
    progress.report(0, "Đang tìm điểm đặc trưng");
    Ptr<Feature2D> finder = SIFT::create(3000);
    std::vector<ImageFeatures> features(n);
    std::vector<Mat> flowImages(n), seamImages(n);
    std::vector<Mat> matchGray(n);  // ảnh xám tiêu cự kBridgeMatchFocal để đo góc qua khung cầu nối
    std::vector<Size> fullSizes(n);
    std::vector<Mat> flowSrcMasks(n), seamSrcMasks(n);  // rỗng nếu ảnh không kèm mặt nạ
    double workScale = 1;
    const double flowScale = std::min(1.0, 1.25 * kFlowWidth / nativeWidth);
    const double seamScale = std::min(1.0, 1.25 * kSeamWidth / nativeWidth);
    for (int i = 0; i < n; ++i) {
        Mat full = loadUpright(paths[i]);
        if (full.empty()) return "Không đọc được ảnh: " + paths[i];
        fullSizes[i] = full.size();
        if (i == 0) workScale = std::min(1.0, std::sqrt(kWorkMegapix * 1e6 / full.size().area()));
        Mat work;
        resize(full, work, Size(), workScale, workScale, INTER_AREA);
        computeImageFeatures(finder, work, features[i]);
        if (kAlignMode == 1 && !bridges.paths.empty()) {
            Mat g;
            cvtColor(work, g, COLOR_BGR2GRAY);
            const double ms = kBridgeMatchFocal / (focalFull * workScale);
            resize(g, matchGray[i], Size(), ms, ms, ms < 1 ? INTER_AREA : INTER_LINEAR);
        }
        features[i].img_idx = i;
        resize(full, flowImages[i], Size(), flowScale, flowScale, INTER_AREA);
        resize(full, seamImages[i], Size(), seamScale, seamScale, INTER_AREA);
        flowSrcMasks[i] = loadMask(paths[i], flowImages[i].size());
        seamSrcMasks[i] = loadMask(paths[i], seamImages[i].size());
        progress.report(2 + 18 * (i + 1) / n, "Đang tìm điểm đặc trưng");
    }
    finder.release();

    // 2. So khớp các cặp có trục nhìn gần nhau theo cảm biến.
    std::vector<Matx33d> sensor(n);
    for (int i = 0; i < n; ++i) sensorR[i].convertTo(Mat(sensor[i], false), CV_64F);
    const Size fs = fullSizes[0];
    const double hfov = 2 * std::atan(fs.width / (2 * focalFull));
    const double vfov = 2 * std::atan(fs.height / (2 * focalFull));
    const double pairThresh = std::cos(std::max(hfov, vfov) * 0.95);
    Mat_<uchar> pairMask(n, n, (uchar) 0);
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (opticalAxis(sensor[i]).dot(opticalAxis(sensor[j])) > pairThresh) pairMask(i, j) = pairMask(j, i) = 1;

    progress.report(22, "Đang so khớp ảnh");
    std::vector<MatchesInfo> pairwise;
    {
        BestOf2NearestMatcher matcher(false, 0.3f);
        matcher(features, pairwise, pairMask.getUMat(ACCESS_READ));
        matcher.collectGarbage();
    }

    // 3. Tinh chỉnh góc.
    progress.report(30, "Đang căn chỉnh góc");
    const double focalWork = focalFull * workScale;
    Matx33d Kwork(focalWork, 0, fs.width * workScale / 2.0, 0, focalWork, fs.height * workScale / 2.0, 0, 0, 1);
    int pairsUsed = 0;
    std::vector<RelRotation> pairRots;
    std::vector<RayPair> obs = collectRayPairs(features, pairwise, sensor, Kwork.inv(), focalWork, pairsUsed,
                                               &pairRots);
    LOGI("Cặp ảnh dùng được: %d, điểm khớp: %zu", pairsUsed, obs.size());
    std::vector<Matx33d> rot;
    if (kAlignMode == 1) {
        const std::vector<RelRotation> chainRots = bridgeRotations(bridges, sensor, matchGray, kBridgeMatchFocal);
        matchGray.clear();
        rot = averageRotations(pairRots, chainRots, sensor);
    } else {
        rot = refineRotations(obs, sensor, focalWork, kHuberFullPx * workScale);
    }
    resetDeviating(rot, sensor);
    pairwise.clear();
    features.clear();
    std::vector<Mat> R32(n);
    for (int i = 0; i < n; ++i) Mat(rot[i]).convertTo(R32[i], CV_32F);

    // 4. Optical flow ở khung rộng kFlowWidth.
    progress.report(38, "Đang xử lý thị sai");
    const double sFlow = kFlowWidth / (2 * CV_PI);
    std::vector<Mat> flowWarped(n), flowMasks(n);
    std::vector<Point> flowCorners(n);
    for (int i = 0; i < n; ++i) {
        flowCorners[i] = warpSphere(flowImages[i], intrinsics(focalFull * flowScale, flowImages[i].size()),
                                    R32[i], sFlow, flowWarped[i], flowMasks[i], nullptr, 0, flowSrcMasks[i]);
    }
    std::vector<Mat> residualImages, residualK;
    if (kDebugResidual) {
        residualImages = flowImages;
        for (const Mat &img : flowImages) residualK.push_back(intrinsics(focalFull * flowScale, img.size()));
    }
    flowImages.clear();
    // Ảnh nằm sát mép trái có thêm một bản dịch đi một vòng để phủ dải lặp bên phải.
    std::vector<Instance> inst;
    const int flowMargin = (int) std::ceil(sFlow * marginRad);
    for (int i = 0; i < n; ++i) {
        inst.push_back({i, 0});
        if (!flowWarped[i].empty() && flowCorners[i].x < -kFlowWidth / 2 + flowMargin) inst.push_back({i, 1});
    }
    const int m = (int) inst.size();
    std::vector<PairParallax> parallax;
    BridgeChains chains;
    if (deghost && kUseBridges) chains = loadBridges(bridges, sensor, rot, kFlowWidth);
    std::vector<Field> fields = deghost ? computeFlowFields(inst, flowWarped, flowMasks, flowCorners, kFlowWidth,
                                                            (double) kSeamWidth / kFlowWidth, &parallax, &chains)
                                        : std::vector<Field>(m);
    chains.clear();
    if (kDebugDump && !gDebugStem.empty()) {
        // Phép xoay cuối cùng và trường uốn của từng bản, để phân tích ngoài máy (khung flow rộng kFlowWidth).
        FILE *f = std::fopen((gDebugStem + "_cams.txt").c_str(), "w");
        if (f) {
            std::fprintf(f, "%d %.6f %d\n", n, focalFull, kFlowWidth);
            for (int i = 0; i < n; ++i) {
                std::fprintf(f, "%d %d", fullSizes[i].width, fullSizes[i].height);
                for (int k = 0; k < 9; ++k) std::fprintf(f, " %.9f", rot[i].val[k]);
                std::fprintf(f, "\n");
            }
            std::fclose(f);
        }
        f = std::fopen((gDebugStem + "_fields.bin").c_str(), "wb");
        if (f) {
            for (int k = 0; k < m; ++k) {
                const Mat &d = fields[k].d;
                int hdr[6] = {inst[k].img, inst[k].shift, fields[k].corner.x, fields[k].corner.y, d.cols, d.rows};
                std::fwrite(hdr, sizeof(int), 6, f);
                for (int y = 0; y < d.rows; ++y) std::fwrite(d.ptr<float>(y), sizeof(float), d.cols * 2, f);
            }
            std::fclose(f);
        }
    }
    flowWarped.clear();
    flowMasks.clear();

    // 5. Cân bằng sáng và đường nối ở khung nhỏ kSeamWidth, trên ảnh đã uốn.
    progress.report(45, "Đang cân bằng sáng");
    const double sSeam = kSeamWidth / (2 * CV_PI);
    std::vector<Point> seamCorners(m);
    std::vector<UMat> seamWarped(m), seamMasks(m);
    std::vector<Mat> validMasks(m);
    for (int k = 0; k < m; ++k) {
        const int i = inst[k].img;
        Mat wi, wm;
        Point c = warpSphere(seamImages[i], intrinsics(focalFull * seamScale, seamImages[i].size()), R32[i], sSeam,
                             wi, wm, &fields[k], inst[k].shift * (double) kSeamWidth, seamSrcMasks[i]);
        if (wi.empty()) {
            wi = Mat::zeros(1, 1, CV_8UC3);
            wm = Mat::zeros(1, 1, CV_8U);
        }
        seamCorners[k] = c + Point(inst[k].shift * kSeamWidth, 0);
        wi.copyTo(seamWarped[k]);
        wm.copyTo(seamMasks[k]);
        validMasks[k] = wm.clone();
    }
    seamImages.clear();

    // Hệ số sáng riêng cho từng ảnh và từng kênh màu (Google cũng chỉnh theo từng ảnh). Chia ô như
    // GAIN_BLOCKS làm tường trơn bị loang từng mảng.
    Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(ExposureCompensator::CHANNELS);
    compensator->feed(seamCorners, seamWarped, seamMasks);

    progress.report(50, "Đang tìm đường nối");
    {
        std::vector<UMat> warpedF(m);
        for (int k = 0; k < m; ++k) seamWarped[k].convertTo(warpedF[k], CV_32F);
        std::vector<PairParallax> debugParallax;
        if (kDebugSeams) debugParallax = parallax;
        Ptr<SeamFinder> seamFinder = makePtr<ParallaxSeamFinder>(std::move(parallax));
        seamFinder->find(warpedF, seamCorners, seamMasks);
        assignPoleCaps(rot, inst, validMasks, seamCorners, seamMasks, sSeam);
        if (kDebugSeams) {
            writeSeamDebug(gDebugStem + "_seams.jpg", seamWarped, seamMasks, seamCorners, debugParallax, kSeamWidth);
        }
    }
    seamWarped.clear();
    validMasks.clear();
    if (kDebugResidual) {
        measureResidual(inst, residualImages, residualK, R32, fields, kFlowWidth, seamMasks, seamCorners, kSeamWidth,
                        &flowSrcMasks);
        residualImages.clear();
    }

    // 6. Dựng ảnh ở độ phân giải đầy đủ và trộn.
    const float pxPerRad = static_cast<float>(outWidth / (2 * CV_PI));
    const int margin = (int) std::ceil(pxPerRad * marginRad);
    const Rect canvas(-outWidth / 2, 0, outWidth + margin, outHeight);
    Ptr<Blender> blender;
    if (outWidth <= kMultiBandMaxWidth) {
        blender = makePtr<MultiBandBlender>(false, 5);
    } else {
        Ptr<FeatherBlender> fb = makePtr<FeatherBlender>();
        fb->setSharpness(0.02f);
        blender = fb;
    }
    blender->prepare(canvas);

    for (int i = 0; i < n; ++i) {
        progress.report(58 + 37 * i / n, "Đang dựng ảnh cầu");
        Mat full = loadUpright(paths[i]);
        if (full.empty()) return "Không đọc được ảnh: " + paths[i];
        Mat K = intrinsics(focalFull, full.size());
        const Mat fullMask = loadMask(paths[i], full.size());
        for (int k = 0; k < m; ++k) {
            if (inst[k].img != i) continue;
            Mat warped, warpedMask;
            Point c = warpSphere(full, K, R32[i], pxPerRad, warped, warpedMask, &fields[k],
                                 inst[k].shift * (double) outWidth, fullMask);
            if (warped.empty()) continue;
            const Point tl = c + Point(inst[k].shift * outWidth, 0);
            Rect roi = Rect(tl, warped.size()) & canvas;
            if (roi.area() <= 0) continue;

            compensator->apply(k, tl, warped, warpedMask);
            Mat seamMask, dilated;
            dilate(seamMasks[k], dilated, Mat());
            resize(dilated, seamMask, warpedMask.size(), 0, 0, INTER_LINEAR_EXACT);
            Mat mask = seamMask & warpedMask;

            Rect local(roi.tl() - tl, roi.size());
            Mat imgS;
            warped(local).convertTo(imgS, CV_16S);
            blender->feed(imgS, mask(local), roi.tl());
        }
    }

    progress.report(96, "Đang lưu ảnh");
    Mat result, resultMask;
    blender->blend(result, resultMask);
    blender.release();
    Mat ext;
    result.convertTo(ext, CV_8U);
    result.release();
    // Trộn chuyển dải lặp bên phải vào mép trái để kinh tuyến 180° liền mạch. Bên nào không có ảnh thì lấy bên
    // kia, không trộn với màu đen.
    Mat covered = resultMask(Rect(0, 0, outWidth, outHeight)).clone();
    for (int c = 0; c < margin && outWidth + c < ext.cols; ++c) {
        const float w = (float) ((c + 0.5) / margin);
        for (int y = 0; y < outHeight; ++y) {
            const bool left = resultMask.at<uchar>(y, c) != 0, right = resultMask.at<uchar>(y, outWidth + c) != 0;
            Vec3b &dst = ext.at<Vec3b>(y, c);
            const Vec3b src = ext.at<Vec3b>(y, outWidth + c);
            if (left && right) {
                for (int k = 0; k < 3; ++k) dst[k] = saturate_cast<uchar>(src[k] * (1 - w) + dst[k] * w);
            } else if (right) {
                dst = src;
            }
            if (right) covered.at<uchar>(y, c) = 255;
        }
    }
    resultMask.release();
    Mat out = ext(Rect(0, 0, outWidth, outHeight)).clone();
    ext.release();

    // Không chụp tấm đỉnh / đáy thì quanh cực có chỗ không ảnh nào phủ.
    fillPoleHoles(out, covered);
    covered.release();

    std::vector<int> params = {IMWRITE_JPEG_QUALITY, jpegQuality};
    if (!imwrite(outPath, out, params)) return "Không ghi được file " + outPath;
    progress.report(100, "Xong");
    return "";
}

}  // namespace sphere
