#pragma once
// Các tham số của bộ ghép.

namespace sphere {

inline constexpr double kWorkMegapix = 1.0;
// Chiều rộng khung toàn cảnh dùng cho optical flow và cho graph cut (Google dùng 512 cho graph cut).
inline constexpr int kFlowWidth = 2048;
inline constexpr int kSeamWidth = 768;
inline constexpr int kMultiBandMaxWidth = 8192;

// Phép xoay tương đối của một cặp ảnh lệch quá ngưỡng này so với cảm biến thì coi là khớp nhầm.
inline constexpr double kPairSensorGateDeg = 12.0;
// Sai số giả định: trọng lực rất chuẩn, hướng ngang trôi nhiều (đo được tới ~14 độ).
inline constexpr double kGravitySigmaDeg = 1.5;
inline constexpr double kHeadingSigmaDeg = 15.0;
// Ngưỡng Huber cho sai số điểm khớp (pixel ở độ phân giải đầy đủ; Google dùng 35).
inline constexpr double kHuberFullPx = 30.0;
// Tinh chỉnh xong mà vẫn lệch quá ngưỡng này thì bỏ, dùng góc cảm biến.
inline constexpr double kMaxDeviationDeg = 25.0;

// Vùng quanh đỉnh/đáy giao hẳn cho tấm chụp thẳng lên/xuống.
inline constexpr double kPoleCapDeg = 18.0;
// Dải lặp bên phải để nối vòng kinh tuyến 180°.
inline constexpr double kWrapMarginDeg = 20.0;

// Uốn ảnh theo optical flow.
inline constexpr double kFlowMaxDeg = 6.0;          // độ dịch tối đa chấp nhận
inline constexpr double kFlowSmoothDeg = 4.0;       // độ mượt của trường uốn
inline constexpr double kFlowFalloff = 0.08;        // càng lớn thì càng ít uốn phần ngoài vùng chồng
inline constexpr double kFlowPoleSkipDeg = 10.0;    // sát cực ảnh equirect méo quá, không tính flow
// Mức "góc cạnh" (giá trị riêng nhỏ của structure tensor, ảnh 8 bit, khối 7x3) coi là tin cậy hoàn toàn.
inline constexpr double kFlowConfRef = 1.5e-3;
// Xoay người khi chụp thì tâm ống kính dịch đi giữa các tấm. Khi đó một điểm chỉ có thể trượt dọc đường
// tròn lớn đi qua nó và hướng dịch chuyển b của cặp ảnh (đường epipolar), xa gần chỉ khác độ dài. Ước lượng
// b cho từng cặp rồi ép flow theo đúng hướng đó nên cho uốn mạnh hơn mà đường thẳng không bị bẻ lung tung.
inline constexpr double kEpiMaxDeg = 15.0;          // độ dịch tối đa khi flow khớp mô hình epipolar
inline constexpr double kEpiMinFlowPx = 1.0;        // flow nhỏ hơn thì hướng không đáng tin, không dùng để ước lượng b
inline constexpr double kEpiInlierPx = 1.2;         // sai lệch cho phép khỏi đường epipolar (pixel khung flow)
inline constexpr int kEpiMinSamples = 150;
inline constexpr double kEpiMinInlierRatio = 0.55;
inline constexpr int kEpiRansacIters = 300;

}  // namespace sphere
