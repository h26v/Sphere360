// Cầu nối JNI: NativeStitcher.stitch() bên Java gọi vào sphere::stitch().

#include <jni.h>

#include "stitcher.h"

extern "C" JNIEXPORT jstring JNICALL
Java_com_sphere360_app_stitch_NativeStitcher_stitch(JNIEnv *env, jclass, jobjectArray jpaths, jfloatArray jrot,
                                             jdouble focalFull, jint outWidth, jint jpegQuality,
                                             jboolean deghost, jstring joutPath, jobject callback) {
    const int n = env->GetArrayLength(jpaths);
    std::vector<std::string> paths(n);
    for (int i = 0; i < n; ++i) {
        auto js = (jstring) env->GetObjectArrayElement(jpaths, i);
        const char *c = env->GetStringUTFChars(js, nullptr);
        paths[i] = c;
        env->ReleaseStringUTFChars(js, c);
        env->DeleteLocalRef(js);
    }
    std::vector<cv::Mat> rot(n);
    jfloat *r = env->GetFloatArrayElements(jrot, nullptr);
    for (int i = 0; i < n; ++i) rot[i] = cv::Mat(3, 3, CV_32F, r + 9 * i).clone();
    env->ReleaseFloatArrayElements(jrot, r, JNI_ABORT);

    const char *op = env->GetStringUTFChars(joutPath, nullptr);
    std::string outPath = op;
    env->ReleaseStringUTFChars(joutPath, op);

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
        err = sphere::stitch(paths, rot, focalFull, outWidth, jpegQuality, deghost == JNI_TRUE, outPath, progress);
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
