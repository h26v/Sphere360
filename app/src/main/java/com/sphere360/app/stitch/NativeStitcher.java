package com.sphere360.app.stitch;

/** Cầu nối tới bộ ghép ảnh cầu viết bằng C++ (sphere_stitcher.cpp). */
public final class NativeStitcher {

    public interface ProgressCallback {
        void onProgress(int percent, String stage);
    }

    static {
        System.loadLibrary("sphere_stitcher");
    }

    private NativeStitcher() {}

    /**
     * @param paths          đường dẫn các ảnh JPEG gốc
     * @param rotations      9 số mỗi ảnh: ma trận xoay camera -> toàn cảnh (hàng trước)
     * @param focalPx        tiêu cự tính theo pixel của ảnh gốc
     * @param outWidth       chiều rộng ảnh xuất (chiều cao = rộng / 2)
     * @param deghost        uốn ảnh theo optical flow ở vùng chồng để giảm bóng mờ do thị sai
     * @return null nếu thành công, ngược lại là thông báo lỗi
     */
    public static native String stitch(String[] paths, float[] rotations, double focalPx,
                                       int outWidth, int jpegQuality, boolean deghost, String outPath,
                                       ProgressCallback callback);
}
