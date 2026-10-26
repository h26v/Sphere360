#include "params.h"

#include <cstdlib>
#include <sstream>

namespace sphere {

namespace {

struct Entry {
    enum Type { DOUBLE, INT, BOOL };
    const char *name;
    Type type;
    void *ptr;
    double def;  // mặc định, chụp lúc khởi tạo thư viện (trước khi có ai đổi)

    void set(double v) const {
        if (type == DOUBLE) *static_cast<double *>(ptr) = v;
        else if (type == INT) *static_cast<int *>(ptr) = (int) std::lround(v);
        else *static_cast<bool *>(ptr) = v != 0;
    }

    double get() const {
        if (type == DOUBLE) return *static_cast<double *>(ptr);
        if (type == INT) return *static_cast<int *>(ptr);
        return *static_cast<bool *>(ptr) ? 1 : 0;
    }
};

// Sinh từ params.h (tools/scripts/patch_debug_api.py); thêm tham số mới thì thêm một dòng ở đây.
const Entry kEntries[] = {
    {"kWorkMegapix", Entry::DOUBLE, &kWorkMegapix, (double) kWorkMegapix},
    {"kFlowWidth", Entry::INT, &kFlowWidth, (double) kFlowWidth},
    {"kSeamWidth", Entry::INT, &kSeamWidth, (double) kSeamWidth},
    {"kMultiBandMaxWidth", Entry::INT, &kMultiBandMaxWidth, (double) kMultiBandMaxWidth},
    {"kPairSensorGateDeg", Entry::DOUBLE, &kPairSensorGateDeg, (double) kPairSensorGateDeg},
    {"kGravitySigmaDeg", Entry::DOUBLE, &kGravitySigmaDeg, (double) kGravitySigmaDeg},
    {"kHeadingSigmaDeg", Entry::DOUBLE, &kHeadingSigmaDeg, (double) kHeadingSigmaDeg},
    {"kHuberFullPx", Entry::DOUBLE, &kHuberFullPx, (double) kHuberFullPx},
    {"kAlignMode", Entry::INT, &kAlignMode, (double) kAlignMode},
    {"kPairSigmaDeg", Entry::DOUBLE, &kPairSigmaDeg, (double) kPairSigmaDeg},
    {"kPairFullInliers", Entry::DOUBLE, &kPairFullInliers, (double) kPairFullInliers},
    {"kPairRobust", Entry::DOUBLE, &kPairRobust, (double) kPairRobust},
    {"kChainSigmaDeg", Entry::DOUBLE, &kChainSigmaDeg, (double) kChainSigmaDeg},
    {"kChainRobust", Entry::DOUBLE, &kChainRobust, (double) kChainRobust},
    {"kSensorStepSigmaDeg", Entry::DOUBLE, &kSensorStepSigmaDeg, (double) kSensorStepSigmaDeg},
    {"kSensorStepRobust", Entry::DOUBLE, &kSensorStepRobust, (double) kSensorStepRobust},
    {"kBridgeMatchFocal", Entry::DOUBLE, &kBridgeMatchFocal, (double) kBridgeMatchFocal},
    {"kBridgeMatchFeatures", Entry::INT, &kBridgeMatchFeatures, (double) kBridgeMatchFeatures},
    {"kBridgeMinInliers", Entry::INT, &kBridgeMinInliers, (double) kBridgeMinInliers},
    {"kMaxDeviationDeg", Entry::DOUBLE, &kMaxDeviationDeg, (double) kMaxDeviationDeg},
    {"kFallbackKeepDrift", Entry::BOOL, &kFallbackKeepDrift, (double) kFallbackKeepDrift},
    {"kPoleCapDeg", Entry::DOUBLE, &kPoleCapDeg, (double) kPoleCapDeg},
    {"kWrapMarginDeg", Entry::DOUBLE, &kWrapMarginDeg, (double) kWrapMarginDeg},
    {"kFlowMaxDeg", Entry::DOUBLE, &kFlowMaxDeg, (double) kFlowMaxDeg},
    {"kFlowSmoothDeg", Entry::DOUBLE, &kFlowSmoothDeg, (double) kFlowSmoothDeg},
    {"kFlowFalloff", Entry::DOUBLE, &kFlowFalloff, (double) kFlowFalloff},
    {"kFlowJoint", Entry::BOOL, &kFlowJoint, (double) kFlowJoint},
    {"kJointGridDeg", Entry::DOUBLE, &kJointGridDeg, (double) kJointGridDeg},
    {"kJointSmooth", Entry::DOUBLE, &kJointSmooth, (double) kJointSmooth},
    {"kJointPrior", Entry::DOUBLE, &kJointPrior, (double) kJointPrior},
    {"kJointMaxIter", Entry::INT, &kJointMaxIter, (double) kJointMaxIter},
    {"kJointPasses", Entry::INT, &kJointPasses, (double) kJointPasses},
    {"kSeamResidual", Entry::INT, &kSeamResidual, (double) kSeamResidual},
    {"kSeamResidualScale", Entry::DOUBLE, &kSeamResidualScale, (double) kSeamResidualScale},
    {"kSeamResidualFloorDeg", Entry::DOUBLE, &kSeamResidualFloorDeg, (double) kSeamResidualFloorDeg},
    {"kJointRefineMaxDeg", Entry::DOUBLE, &kJointRefineMaxDeg, (double) kJointRefineMaxDeg},
    {"kFlowPoleSkipDeg", Entry::DOUBLE, &kFlowPoleSkipDeg, (double) kFlowPoleSkipDeg},
    {"kFlowConfRef", Entry::DOUBLE, &kFlowConfRef, (double) kFlowConfRef},
    {"kEpiMaxDeg", Entry::DOUBLE, &kEpiMaxDeg, (double) kEpiMaxDeg},
    {"kEpiMinFlowPx", Entry::DOUBLE, &kEpiMinFlowPx, (double) kEpiMinFlowPx},
    {"kEpiInlierPx", Entry::DOUBLE, &kEpiInlierPx, (double) kEpiInlierPx},
    {"kEpiMinSamples", Entry::INT, &kEpiMinSamples, (double) kEpiMinSamples},
    {"kEpiMinInlierRatio", Entry::DOUBLE, &kEpiMinInlierRatio, (double) kEpiMinInlierRatio},
    {"kEpiRansacIters", Entry::INT, &kEpiRansacIters, (double) kEpiRansacIters},
    {"kSeamParallaxCapDeg", Entry::DOUBLE, &kSeamParallaxCapDeg, (double) kSeamParallaxCapDeg},
    {"kSeamParallaxFloorDeg", Entry::DOUBLE, &kSeamParallaxFloorDeg, (double) kSeamParallaxFloorDeg},
    {"kSeamParallaxWeight", Entry::DOUBLE, &kSeamParallaxWeight, (double) kSeamParallaxWeight},
    {"kSeamParallaxMinConf", Entry::DOUBLE, &kSeamParallaxMinConf, (double) kSeamParallaxMinConf},
    {"kSeamBadWindowDeg", Entry::DOUBLE, &kSeamBadWindowDeg, (double) kSeamBadWindowDeg},
    {"kSeamBadRatioLo", Entry::DOUBLE, &kSeamBadRatioLo, (double) kSeamBadRatioLo},
    {"kSeamBadRatioHi", Entry::DOUBLE, &kSeamBadRatioHi, (double) kSeamBadRatioHi},
    {"kSeamParallaxGrowDeg", Entry::DOUBLE, &kSeamParallaxGrowDeg, (double) kSeamParallaxGrowDeg},
    {"kSeamParallaxSmoothDeg", Entry::DOUBLE, &kSeamParallaxSmoothDeg, (double) kSeamParallaxSmoothDeg},
    {"kSeamParallaxMinDensity", Entry::DOUBLE, &kSeamParallaxMinDensity, (double) kSeamParallaxMinDensity},
    {"kUseBridges", Entry::BOOL, &kUseBridges, (double) kUseBridges},
    {"kBridgeMaxDeg", Entry::DOUBLE, &kBridgeMaxDeg, (double) kBridgeMaxDeg},
    {"kBridgeConsistPx", Entry::DOUBLE, &kBridgeConsistPx, (double) kBridgeConsistPx},
    {"kBridgeStepDeg", Entry::DOUBLE, &kBridgeStepDeg, (double) kBridgeStepDeg},
    {"kBridgeMinCover", Entry::DOUBLE, &kBridgeMinCover, (double) kBridgeMinCover},
    {"kBridgeFillDeg", Entry::DOUBLE, &kBridgeFillDeg, (double) kBridgeFillDeg},
    {"kDebugSeams", Entry::BOOL, &kDebugSeams, (double) kDebugSeams},
    {"kDebugResidual", Entry::BOOL, &kDebugResidual, (double) kDebugResidual},
    {"kDebugDump", Entry::BOOL, &kDebugDump, (double) kDebugDump},
};

}  // namespace

std::string applyParams(const std::string &spec) {
    std::stringstream ss(spec);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (item.find_first_not_of(" ") == std::string::npos) continue;
        size_t eq = item.find('=');
        if (eq == std::string::npos) return "Thiếu '=': " + item;
        std::string name = item.substr(0, eq), value = item.substr(eq + 1);
        name.erase(0, name.find_first_not_of(' '));
        name.erase(name.find_last_not_of(' ') + 1);
        char *end = nullptr;
        double v = std::strtod(value.c_str(), &end);
        if (end == value.c_str()) {
            if (value == "true") v = 1;
            else if (value == "false") v = 0;
            else return "Giá trị không hợp lệ: " + item;
        }
        bool found = false;
        for (const Entry &e : kEntries)
            if (name == e.name) {
                e.set(v);
                found = true;
            }
        if (!found) return "Không có tham số " + name;
    }
    return "";
}

void resetParams() {
    for (const Entry &e : kEntries) e.set(e.def);
}

std::string describeParams() {
    std::ostringstream os;
    for (const Entry &e : kEntries) os << e.name << "=" << e.get() << (e.get() != e.def ? "  (mặc định " : "")
                                       << (e.get() != e.def ? std::to_string(e.def) + ")" : "") << "\n";
    return os.str();
}

}  // namespace sphere
