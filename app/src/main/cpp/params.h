#pragma once
// Các tham số của bộ ghép. Mặc định như dưới; lệnh debug đổi được theo tên lúc chạy (applyParams).

#include <string>

namespace sphere {

// Đặt tham số theo tên, dạng "kFlowMaxDeg=8,kSeamParallaxWeight=4". Trả về chuỗi rỗng nếu được, ngược lại là
// thông báo lỗi (các tham số trước chỗ lỗi vẫn đã được đặt).
std::string applyParams(const std::string &spec);
// Trả mọi tham số về mặc định.
void resetParams();
// Mọi tham số hiện tại, mỗi dòng "tên=giá trị".
std::string describeParams();

inline double kWorkMegapix = 1.0;
// Chiều rộng khung toàn cảnh dùng cho optical flow và cho graph cut (Google dùng 512 cho graph cut).
inline int kFlowWidth = 2048;
inline int kSeamWidth = 768;
inline int kMultiBandMaxWidth = 8192;

// Phép xoay tương đối của một cặp ảnh lệch quá ngưỡng này so với cảm biến thì coi là khớp nhầm.
inline double kPairSensorGateDeg = 12.0;
// Sai số giả định: trọng lực rất chuẩn, hướng ngang trôi nhiều (đo được tới ~14 độ).
inline double kGravitySigmaDeg = 1.5;
inline double kHeadingSigmaDeg = 15.0;
// Ngưỡng Huber cho sai số điểm khớp (pixel ở độ phân giải đầy đủ; Google dùng 35).
inline double kHuberFullPx = 30.0;
// Tinh chỉnh xong mà độ hiệu chỉnh của ảnh lệch quá ngưỡng này so với mốc chung của mọi ảnh thì bỏ, đặt lại
// theo cảm biến cộng độ hiệu chỉnh của các ảnh bên cạnh.
inline double kMaxDeviationDeg = 25.0;
inline bool kFallbackKeepDrift = false;

// Căn góc (alignment.cpp). Mỗi cặp ảnh cho một phép đo góc tương đối, mỗi chuỗi khung cầu nối giữa hai tấm liền
// nhau cũng vậy; con quay neo góc giữa hai tấm chụp liền nhau (chính xác trong ngắn hạn, trôi chậm).
inline int kAlignMode = 1;  // 0 = tinh chỉnh theo điểm khớp (cũ), 1 = trung bình phép xoay có chống nhiễu
inline double kPairSigmaDeg = 1.5;  // sai số góc của một cặp ảnh đủ điểm (thị sai làm lệch cả cặp)
inline double kPairFullInliers = 60.0;  // cặp có từng này điểm khớp đúng thì đủ trọng số
inline double kPairRobust = 2.0;  // Cauchy: lệch quá bấy nhiêu lần sigma thì giảm dần trọng số
inline double kChainSigmaDeg = 0.7;  // sai số góc đo qua chuỗi khung cầu nối
inline double kChainRobust = 3.0;
// Sai số góc con quay giữa hai tấm chụp liền nhau. 2° kéo vài tấm lệch khỏi ảnh (khung cửa nhân đôi ở lượt
// 20261023_171607); 8° thì cặp ảnh lại kéo lệch; 4° tốt nhất trên cả lượt đó và 20261023_181150.
inline double kSensorStepSigmaDeg = 4.0;
inline double kSensorStepRobust = 3.0;
inline double kBridgeMatchFocal = 500.0;  // tiêu cự (pixel) của ảnh dùng để so khớp các bước của chuỗi
inline int kBridgeMatchFeatures = 1000;
inline int kBridgeMinInliers = 25;  // bước nào ít điểm khớp đúng hơn thì dùng góc con quay cho bước đó

// Vùng quanh đỉnh/đáy giao hẳn cho tấm chụp thẳng lên/xuống.
inline double kPoleCapDeg = 18.0;
// Dải lặp bên phải để nối vòng kinh tuyến 180°.
inline double kWrapMarginDeg = 20.0;

// Uốn ảnh theo optical flow.
inline double kFlowMaxDeg = 6.0;          // độ dịch tối đa chấp nhận
inline double kFlowSmoothDeg = 4.0;       // độ mượt của trường uốn
inline double kFlowFalloff = 0.08;        // càng lớn thì càng ít uốn phần ngoài vùng chồng
inline double kFlowPoleSkipDeg = 10.0;    // sát cực ảnh equirect méo quá, không tính flow
// Mức "góc cạnh" (giá trị riêng nhỏ của structure tensor, ảnh 8 bit, khối 7x3) coi là tin cậy hoàn toàn.
inline double kFlowConfRef = 1.5e-3;
// Xoay người khi chụp thì tâm ống kính dịch đi giữa các tấm. Khi đó một điểm chỉ có thể trượt dọc đường
// tròn lớn đi qua nó và hướng dịch chuyển b của cặp ảnh (đường epipolar), xa gần chỉ khác độ dài. Ước lượng
// b cho từng cặp rồi ép flow theo đúng hướng đó nên cho uốn mạnh hơn mà đường thẳng không bị bẻ lung tung.
inline double kEpiMaxDeg = 15.0;          // độ dịch tối đa khi flow khớp mô hình epipolar
inline double kEpiMinFlowPx = 1.0;        // flow nhỏ hơn thì hướng không đáng tin, không dùng để ước lượng b
inline double kEpiInlierPx = 1.2;         // sai lệch cho phép khỏi đường epipolar (pixel khung flow)
inline int kEpiMinSamples = 150;
inline double kEpiMinInlierRatio = 0.55;
inline int kEpiRansacIters = 300;

// Đường nối biết thị sai: chỗ hai ảnh lệch nhau nhiều (vật gần) hoặc flow không tin được (che khuất, lệch
// quá tầm flow) bị cộng thêm chi phí, để đường nối vòng qua vùng xa và vật gần lấy trọn từ một ảnh.
inline double kSeamParallaxCapDeg = 6.0;    // mức thị sai tối đa; flow hỏng cũng gán mức này
inline double kSeamParallaxFloorDeg = 0.5;  // dưới mức này coi như không lệch (sai số căn góc)
inline double kSeamParallaxWeight = 3.0;    // mỗi độ thị sai = bấy nhiêu lần chi phí màu trung vị
// Chỉ đo thị sai ở chỗ đủ góc cạnh (độ tin cậy flow, xem kFlowConfRef). Tường trơn thì flow hai chiều lệch
// nhau lung tung nhưng cắt qua đó cũng không lộ, để chi phí màu lo.
inline double kSeamParallaxMinConf = 0.3;
// Flow hai chiều không khớp nhau chỉ coi là vật gần (che khuất, lệch quá tầm flow) khi dày đặc: tỉ lệ điểm
// hỏng trong ô kSeamBadWindowDeg từ Lo (0) tới Hi (mức trần). Mặc định tắt (Lo >= 1): trên sàn gạch, trần tôn
// (hoạ tiết lặp) flow hỏng dày đặc cả ở chỗ xa nên làm bản đồ thị sai nhiễu (lượt 20261023_171607).
inline double kSeamBadWindowDeg = 1.5;
inline double kSeamBadRatioLo = 1.0;
inline double kSeamBadRatioHi = 1.5;
// Nới vùng thị sai ra bấy nhiêu độ để đường nối không đi sát mép vật gần.
inline double kSeamParallaxGrowDeg = 1.0;
// Thị sai đo được là trung bình có trọng số của các điểm flow đã chấp nhận trong bán kính này (Gaussian), chỉ
// tính khi mật độ điểm (0..1) đủ kSeamParallaxMinDensity.
inline double kSeamParallaxSmoothDeg = 0.75;
inline double kSeamParallaxMinDensity = 0.05;
// Khung cầu nối (bridge.h): với hai tấm chụp liền nhau, dò flow qua chuỗi khung chụp lúc lia máy.
inline bool kUseBridges = true;
inline double kBridgeMaxDeg = 15.0;     // độ lệch tổng tối đa dò theo chuỗi
inline double kBridgeConsistPx = 1.5;   // mỗi bước: flow đi rồi về phải lệch ít hơn (pixel khung flow)
inline double kBridgeStepDeg = 6.0;     // mỗi bước của chuỗi xoay không quá bấy nhiêu (trong tầm của DIS)
inline double kBridgeMinCover = 0.2;    // chuỗi dò được ít hơn tỉ lệ vùng chồng này thì bỏ, dùng flow thường
inline double kBridgeFillDeg = 2.0;     // bán kính điền chỗ chuỗi đứt trước khi làm flow khởi đầu

// Ghi ảnh debug (bản đồ thị sai và đường nối ở khung đường nối) cạnh file kết quả.
inline bool kDebugSeams = false;

}  // namespace sphere
