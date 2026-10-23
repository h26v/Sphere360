#pragma once
// Khung cầu nối: ảnh xám nhỏ chụp liên tục lúc lia máy giữa hai tấm liên tiếp. Giữa hai tấm, vật gần có thể
// lệch tới 10–15° (quá tầm optical flow), nhưng giữa hai khung cầu nối liền nhau chỉ lệch vài độ. Dò flow lần
// lượt qua chuỗi khung rồi cộng lại thì ra độ lệch thật giữa hai tấm.

#include "alignment.h"
#include "common.h"
#include "stitcher.h"

namespace sphere {

// Một khung cầu nối đã nạp, thu nhỏ cho khung flow.
struct BridgeFrame {
    Mat gray;  // CV_8U, đã xoay đứng
    Mat K;     // nội tại theo kích thước gray
    Mat R;     // CV_32F, camera -> toàn cảnh, đã hiệu chỉnh theo góc tinh chỉnh của hai tấm hai đầu
    Matx33d Rd;
};

// Các khung (theo thứ tự thời gian) giữa tấm a và tấm a + 1, kèm góc đã tinh chỉnh của hai tấm.
struct BridgeChain {
    std::vector<BridgeFrame> frames;
    Matx33d Ra, Rb;
};

// chains[i]: chuỗi giữa tấm i và tấm i + 1.
using BridgeChains = std::vector<BridgeChain>;

// Nạp khung cầu nối cho khung flow rộng flowWidth. Hướng cảm biến của khung được hiệu chỉnh bằng độ lệch
// giữa góc cảm biến và góc đã tinh chỉnh của hai tấm hai đầu, nội suy theo vị trí của khung trong chuỗi.
BridgeChains loadBridges(const BridgeInput &in, const std::vector<Matx33d> &sensor,
                         const std::vector<Matx33d> &refined, int flowWidth);

// Góc tương đối giữa hai tấm chụp liền nhau đo qua chuỗi khung cầu nối: mỗi bước (≤ kBridgeStepDeg) so khớp
// SIFT giữa hai ảnh liền nhau của chuỗi; bước nào ít điểm khớp đúng thì dùng góc con quay cho bước đó.
// stillGray: các tấm, ảnh xám với tiêu cự matchFocal. weight = tỉ lệ bước đo được bằng ảnh.
std::vector<RelRotation> bridgeRotations(const BridgeInput &in, const std::vector<Matx33d> &sensor,
                                         const std::vector<Mat> &stillGray, double matchFocal);

// Một ảnh đã nắn lên khung flow (toạ độ như warpSphere, x từ -width/2).
struct View {
    Mat gray, mask;
    Point tl;
};

// Flow hai chiều giữa tấm gap (p) và tấm gap + 1 (q) trên vùng chồng ov, dò qua khung cầu nối; forward = false
// nếu p, q được đưa theo thứ tự ngược lại. Chuỗi là đường ít bước nhất từ p tới q qua các khung của khoảng này và
// hai khoảng bên cạnh, mỗi bước xoay không quá kBridgeStepDeg (người chụp có thể lia quá rồi quay lại, nên không
// đi theo thứ tự thời gian). a, b là vùng ov của p và q. Kết quả chuỗi được tinh chỉnh bằng DIS giữa thẳng a và b
// để khử sai số cộng dồn. hops: số bước, 0 nếu không có đường. Trả về tỉ lệ (0..1) vùng chồng dò được qua chuỗi;
// nếu dưới kBridgeMinCover thì F, G là flow DIS thường.
double bridgeFlows(const BridgeChains &chains, int gap, bool forward, const View &p, const View &q, Rect ov,
                   int width, const Mat &a, const Mat &b, Mat &F, Mat &G, int &hops);

}  // namespace sphere
