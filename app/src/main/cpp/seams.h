#pragma once
// Đường nối: graph cut biết thị sai, và giao vùng quanh đỉnh/đáy.

#include "flow.h"
#include "warp.h"

// Cần OpenCV core (qua common.h) đứng trước.
#include <opencv2/imgproc/detail/gcgraph.hpp>

namespace sphere {

// Graph cut như GraphCutSeamFinder(COST_COLOR_GRAD) của OpenCV, cộng thêm chi phí theo thị sai đo được ở
// vùng chồng: đường nối né vật gần và chỗ flow không tin được, đi qua vùng xa nơi hai ảnh khớp nhau.
class ParallaxSeamFinder : public PairwiseSeamFinder {
public:
    explicit ParallaxSeamFinder(std::vector<PairParallax> parallax) : parallax_(std::move(parallax)) {}

    void find(const std::vector<UMat> &src, const std::vector<Point> &corners,
              std::vector<UMat> &masks) override;

private:
    void findInPair(size_t first, size_t second, Rect roi) override;

    std::vector<PairParallax> parallax_;
    std::vector<Mat> dx_, dy_;
    double seamLength_ = 0, seamParallax_ = 0, seamMaxParallax_ = 0;
    int seamOverCap_ = 0;
};

// Giao vùng quanh đỉnh/đáy cho tấm chụp thẳng lên/xuống.
void assignPoleCaps(const std::vector<Matx33d> &R, const std::vector<Instance> &inst,
                    const std::vector<Mat> &valid, const std::vector<Point> &corners,
                    std::vector<UMat> &seamMasks, double s);

}  // namespace sphere
