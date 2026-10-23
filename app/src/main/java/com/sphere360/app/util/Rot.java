package com.sphere360.app.util;

/** Phép tính ma trận 3x3 (lưu theo hàng) dùng cho hướng camera. */
public final class Rot {

    private Rot() {}

    public static float[] mul(float[] a, float[] b) {
        float[] r = new float[9];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
        return r;
    }

    public static float[] apply(float[] m, float x, float y, float z) {
        return new float[]{
                m[0] * x + m[1] * y + m[2] * z,
                m[3] * x + m[4] * y + m[5] * z,
                m[6] * x + m[7] * y + m[8] * z};
    }

    /** m^T * v */
    public static float[] applyT(float[] m, float[] v) {
        return new float[]{
                m[0] * v[0] + m[3] * v[1] + m[6] * v[2],
                m[1] * v[0] + m[4] * v[1] + m[7] * v[2],
                m[2] * v[0] + m[5] * v[1] + m[8] * v[2]};
    }

    public static float[] rotZ(double rad) {
        float c = (float) Math.cos(rad), s = (float) Math.sin(rad);
        return new float[]{c, -s, 0, s, c, 0, 0, 0, 1};
    }

    public static double angleDeg(float[] a, float[] b) {
        // atan2(|a×b|, a·b) chính xác cả khi hai vector gần trùng nhau (acos thì không).
        double cx = (double) a[1] * b[2] - (double) a[2] * b[1];
        double cy = (double) a[2] * b[0] - (double) a[0] * b[2];
        double cz = (double) a[0] * b[1] - (double) a[1] * b[0];
        double dot = (double) a[0] * b[0] + (double) a[1] * b[1] + (double) a[2] * b[2];
        return Math.toDegrees(Math.atan2(Math.sqrt(cx * cx + cy * cy + cz * cz), dot));
    }

    /**
     * Đổi ma trận thiết bị -> thế giới (đã căn yaw) thành ma trận camera -> toàn cảnh theo quy ước
     * của OpenCV SphericalWarper.
     * Camera (OpenCV): x phải, y xuống, z nhìn tới = (x, -y, -z) của thiết bị khi cầm dọc.
     * Toàn cảnh: x = Đông, y = xuống, z = hướng nhìn ban đầu.
     */
    public static float[] cameraToPano(float[] deviceToWorld) {
        float[] a = deviceToWorld;
        // X = A * diag(1, -1, -1)
        float[] x = {
                a[0], -a[1], -a[2],
                a[3], -a[4], -a[5],
                a[6], -a[7], -a[8]};
        // M * X với M = [[1,0,0],[0,0,-1],[0,1,0]]
        return new float[]{
                x[0], x[1], x[2],
                -x[6], -x[7], -x[8],
                x[3], x[4], x[5]};
    }
}
