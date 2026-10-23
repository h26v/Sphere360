// Ghép ảnh cầu 360x180 (equirectangular 2:1) từ nhiều tấm chụp có kèm hướng cảm biến.
//
// Các bước (tham khảo cách Google Photo Sphere / Street View làm):
//  1. Tìm điểm đặc trưng SIFT, chỉ so khớp các cặp ảnh nằm cạnh nhau theo cảm biến.
//  2. Mỗi cặp: RANSAC 2 điểm tìm phép xoay tương đối, bỏ khớp nhầm.
//  3. Tinh chỉnh góc của mọi tấm cùng lúc (Levenberg–Marquardt, Huber). Neo chặt theo trọng lực
//     (cảm biến đo chiều "xuống" rất chuẩn), neo lỏng theo hướng ngang (con quay trôi theo thời gian).
//  4. Thị sai: ở mỗi vùng chồng, tính optical flow giữa hai ảnh rồi uốn mỗi ảnh đi nửa đường để nội
//     dung gặp nhau; độ uốn giảm dần ra xa vùng chồng (giống Street View, bản đơn giản hoá).
//  5. Cân bằng sáng, tìm đường nối bằng graph cut trên ảnh nhỏ, giao vùng đỉnh/đáy cho tấm chụp
//     thẳng lên/xuống, nối vòng ở kinh tuyến 180°.
//  6. Dựng ảnh ở độ phân giải đầy đủ và trộn nhiều dải tần (multi-band).

#include "stitcher.h"

#include "alignment.h"
#include "flow.h"
#include "geometry.h"
#include "params.h"
#include "seams.h"
#include "warp.h"

namespace sphere {

namespace {

Mat loadUpright(const std::string &path) {
    // IMREAD_COLOR áp dụng luôn EXIF orientation.
    Mat img = imread(path, IMREAD_COLOR);
    if (img.empty()) return img;
    // Ảnh luôn chụp dọc; nếu vẫn ngang thì EXIF không có, tự xoay.
    if (img.cols > img.rows) rotate(img, img, ROTATE_90_CLOCKWISE);
    return img;
}

// ------------------------------------------------------------------------------ hình học

}  // namespace

std::string stitch(const std::vector<std::string> &paths, const std::vector<Mat> &sensorR,
                   double focalFull, int outWidth, int jpegQuality, bool deghost, const std::string &outPath,
                   const Progress &progress) {
    const int n = static_cast<int>(paths.size());
    if (n < 2) return "Cần ít nhất 2 ảnh";
    outWidth = std::max(1024, outWidth / 2 * 2);
    const int outHeight = outWidth / 2;
    const double nativeWidth = 2 * CV_PI * focalFull;
    const double marginRad = kWrapMarginDeg * CV_PI / 180.0;

    // 1. Đặc trưng ở độ phân giải làm việc, kèm bản nhỏ cho bước flow và bước đường nối.
    progress.report(0, "Đang tìm điểm đặc trưng");
    Ptr<Feature2D> finder = SIFT::create(3000);
    std::vector<ImageFeatures> features(n);
    std::vector<Mat> flowImages(n), seamImages(n);
    std::vector<Size> fullSizes(n);
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
        features[i].img_idx = i;
        resize(full, flowImages[i], Size(), flowScale, flowScale, INTER_AREA);
        resize(full, seamImages[i], Size(), seamScale, seamScale, INTER_AREA);
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
    std::vector<RayPair> obs = collectRayPairs(features, pairwise, sensor, Kwork.inv(), focalWork, pairsUsed);
    LOGI("Cặp ảnh dùng được: %d, điểm khớp: %zu", pairsUsed, obs.size());
    std::vector<Matx33d> rot = refineRotations(obs, sensor, focalWork, kHuberFullPx * workScale);
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
                                    R32[i], sFlow, flowWarped[i], flowMasks[i]);
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
    std::vector<Field> fields = deghost ? computeFlowFields(inst, flowWarped, flowMasks, flowCorners, kFlowWidth)
                                        : std::vector<Field>(m);
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
                             wi, wm, &fields[k], inst[k].shift * (double) kSeamWidth);
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
        Ptr<SeamFinder> seamFinder = makePtr<GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR_GRAD);
        seamFinder->find(warpedF, seamCorners, seamMasks);
        assignPoleCaps(rot, inst, validMasks, seamCorners, seamMasks, sSeam);
    }
    seamWarped.clear();
    validMasks.clear();

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
        for (int k = 0; k < m; ++k) {
            if (inst[k].img != i) continue;
            Mat warped, warpedMask;
            Point c = warpSphere(full, K, R32[i], pxPerRad, warped, warpedMask, &fields[k],
                                 inst[k].shift * (double) outWidth);
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
    // Trộn chuyển dải lặp bên phải vào mép trái để kinh tuyến 180° liền mạch.
    for (int c = 0; c < margin && outWidth + c < ext.cols; ++c) {
        double w = (c + 0.5) / margin;
        Mat dst = ext.col(c);
        addWeighted(ext.col(outWidth + c), 1.0 - w, ext.col(c), w, 0, dst);
    }
    Mat out = ext(Rect(0, 0, outWidth, outHeight)).clone();
    ext.release();

    std::vector<int> params = {IMWRITE_JPEG_QUALITY, jpegQuality};
    if (!imwrite(outPath, out, params)) return "Không ghi được file " + outPath;
    progress.report(100, "Xong");
    return "";
}

}  // namespace sphere
