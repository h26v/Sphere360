package com.sphere360.app.stitch;

/** Cầu nối tới bộ ghép ảnh cầu viết bằng C++ (app/src/main/cpp). */
public final class NativeStitcher {

    public interface ProgressCallback {
        void onProgress(int percent, String stage);
    }

    static {
        System.loadLibrary("sphere_stitcher");
    }

    private NativeStitcher() {}

    /**
     * @param in             các tấm và khung cầu nối của lượt chụp
     * @param outWidth       chiều rộng ảnh xuất (chiều cao = rộng / 2)
     * @param deghost        uốn ảnh theo optical flow ở vùng chồng để giảm bóng mờ do thị sai
     * @return null nếu thành công, ngược lại là thông báo lỗi
     */
    public static String stitch(StitchInput in, int outWidth, int jpegQuality, boolean deghost, String outPath,
                                ProgressCallback callback) {
        return stitchNative(in.paths, in.rotations, in.focalPx, in.bridgePaths, in.bridgeRotations, in.bridgeAfter,
                in.bridgeFocalPx, in.bridgeRotation, outWidth, jpegQuality, deghost, outPath, callback);
    }

    private static native String stitchNative(String[] paths, float[] rotations, double focalPx,
                                              String[] bridgePaths, float[] bridgeRotations, int[] bridgeAfter,
                                              double bridgeFocalPx, int bridgeRotation, int outWidth,
                                              int jpegQuality, boolean deghost, String outPath,
                                              ProgressCallback callback);

    /**
     * Trả mọi tham số bộ ghép về mặc định rồi đặt theo spec, dạng "kFlowMaxDeg=8,kSeamParallaxWeight=4"
     * (tên trong params.h). Chỉ dùng cho lệnh debug.
     *
     * @return null nếu được, ngược lại là thông báo lỗi
     */
    public static native String setParams(String spec);

    /** Mọi tham số hiện tại, mỗi dòng "tên=giá trị". */
    public static native String describeParams();

    /** Ghi thêm log của bộ ghép ra file (ghi đè); null hoặc rỗng thì thôi. */
    public static native void setLogFile(String path);
}
