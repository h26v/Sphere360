// Cầu nối JNI: NativeStitcher.stitch() bên Java gọi vào sphere::stitch().

#include <jni.h>

#include "params.h"
#include "stitcher.h"

namespace {

std::string toString(JNIEnv *env, jstring js) {
    if (js == nullptr) return "";
    const char *c = env->GetStringUTFChars(js, nullptr);
    std::string s = c;
    env->ReleaseStringUTFChars(js, c);
    return s;
}

std::vector<std::string> toStrings(JNIEnv *env, jobjectArray arr) {
    const int n = arr == nullptr ? 0 : env->GetArrayLength(arr);
    std::vector<std::string> out(n);
    for (int i = 0; i < n; ++i) {
        auto js = (jstring) env->GetObjectArrayElement(arr, i);
        out[i] = toString(env, js);
        env->DeleteLocalRef(js);
    }
    return out;
}

// 9 số mỗi ảnh (hàng trước) -> ma trận 3x3 CV_32F.
std::vector<cv::Mat> toRotations(JNIEnv *env, jfloatArray arr, int n) {
    std::vector<cv::Mat> rot(n);
    if (n == 0) return rot;
    jfloat *r = env->GetFloatArrayElements(arr, nullptr);
    for (int i = 0; i < n; ++i) rot[i] = cv::Mat(3, 3, CV_32F, r + 9 * i).clone();
    env->ReleaseFloatArrayElements(arr, r, JNI_ABORT);
    return rot;
}

}  // namespace

extern "C" JNIEXPORT jstring JNICALL
Java_com_sphere360_app_stitch_NativeStitcher_setParams(JNIEnv *env, jclass, jstring spec) {
    sphere::resetParams();
    std::string err = sphere::applyParams(toString(env, spec));
    return err.empty() ? nullptr : env->NewStringUTF(err.c_str());
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_sphere360_app_stitch_NativeStitcher_describeParams(JNIEnv *env, jclass) {
    return env->NewStringUTF(sphere::describeParams().c_str());
}

extern "C" JNIEXPORT void JNICALL
Java_com_sphere360_app_stitch_NativeStitcher_setLogFile(JNIEnv *env, jclass, jstring path) {
    sphere::setLogFile(toString(env, path));
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_sphere360_app_stitch_NativeStitcher_stitchNative(
        JNIEnv *env, jclass, jobjectArray jpaths, jfloatArray jrot, jdouble focalFull, jobjectArray jbridgePaths,
        jfloatArray jbridgeRot, jintArray jbridgeAfter, jdouble bridgeFocal, jint bridgeRotation, jint outWidth,
        jint jpegQuality, jboolean deghost, jstring joutPath, jobject callback) {
    const std::vector<std::string> paths = toStrings(env, jpaths);
    const std::vector<cv::Mat> rot = toRotations(env, jrot, (int) paths.size());

    sphere::BridgeInput bridges;
    bridges.paths = toStrings(env, jbridgePaths);
    bridges.sensorR = toRotations(env, jbridgeRot, (int) bridges.paths.size());
    bridges.after.resize(bridges.paths.size());
    if (!bridges.paths.empty()) env->GetIntArrayRegion(jbridgeAfter, 0, (jsize) bridges.paths.size(), bridges.after.data());
    bridges.focal = bridgeFocal;
    bridges.rotationCw = bridgeRotation;

    const std::string outPath = toString(env, joutPath);

    sphere::Progress progress;
    if (callback != nullptr) {
        jclass cls = env->GetObjectClass(callback);
        jmethodID method = env->GetMethodID(cls, "onProgress", "(ILjava/lang/String;)V");
        progress.fn = [env, callback, method](int percent, const char *stage) {
            jstring s = env->NewStringUTF(stage);
            env->CallVoidMethod(callback, method, percent, s);
            env->DeleteLocalRef(s);
        };
    }

    std::string err;
    try {
        err = sphere::stitch(paths, rot, focalFull, bridges, outWidth, jpegQuality, deghost == JNI_TRUE, outPath,
                             progress);
    } catch (const cv::Exception &e) {
        err = std::string("Lỗi OpenCV: ") + e.what();
    } catch (const std::bad_alloc &) {
        err = "Hết bộ nhớ, hãy chọn kích thước xuất nhỏ hơn";
    } catch (const std::exception &e) {
        err = std::string("Lỗi: ") + e.what();
    }
    if (err.empty()) return nullptr;
    return env->NewStringUTF(err.c_str());
}
