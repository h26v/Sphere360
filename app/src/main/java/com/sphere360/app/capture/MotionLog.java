package com.sphere360.app.capture;

/**
 * Lưu vài giây gần nhất của con quay hồi chuyển và hướng máy (theo đồng hồ elapsedRealtimeNanos),
 * để biết máy đã xoay bao nhiêu trong đúng lúc cảm biến ảnh phơi sáng và máy hướng về đâu lúc đó.
 * Ghi từ luồng cảm biến, đọc từ luồng giao diện và luồng khung cầu nối.
 */
final class MotionLog {

    private static final int GYRO_CAP = 2048;
    private static final int POSE_CAP = 256;

    private final long[] gyroTs = new long[GYRO_CAP];
    private final double[] gyroRad = new double[GYRO_CAP]; // tốc độ xoay, rad/s
    private int gyroCount, gyroHead;

    private final long[] poseTs = new long[POSE_CAP];
    private final float[][] poses = new float[POSE_CAP][];
    private int poseCount, poseHead;

    synchronized void addGyro(long ts, double radPerSec) {
        gyroTs[gyroHead] = ts;
        gyroRad[gyroHead] = radPerSec;
        gyroHead = (gyroHead + 1) % GYRO_CAP;
        gyroCount = Math.min(gyroCount + 1, GYRO_CAP);
    }

    synchronized void addPose(long ts, float[] deviceToWorld) {
        poseTs[poseHead] = ts;
        poses[poseHead] = deviceToWorld;
        poseHead = (poseHead + 1) % POSE_CAP;
        poseCount = Math.min(poseCount + 1, POSE_CAP);
    }

    /**
     * Góc máy đã quét (rad) trong khoảng [start, end]: mỗi mẫu giữ tốc độ của nó tới mẫu kế tiếp.
     * Trả về -1 khi không có mẫu nào phủ khoảng này.
     */
    synchronized double sweptRad(long start, long end) {
        if (gyroCount == 0 || end <= start) return -1;
        double sum = 0;
        boolean covered = false;
        int first = (gyroHead - gyroCount + GYRO_CAP) % GYRO_CAP;
        for (int k = 0; k < gyroCount; k++) {
            int i = (first + k) % GYRO_CAP;
            long t0 = gyroTs[i];
            long t1 = k + 1 < gyroCount ? gyroTs[(i + 1) % GYRO_CAP] : end;
            long a = Math.max(t0, start), b = Math.min(t1, end);
            if (b > a) {
                sum += gyroRad[i] * (b - a) * 1e-9;
                covered = true;
            }
        }
        return covered ? sum : -1;
    }

    /** Hướng máy ghi gần thời điểm ts nhất, hoặc null nếu không có mẫu nào trong vòng 50 ms. */
    synchronized float[] poseAt(long ts) {
        float[] best = null;
        long bestDt = 50_000_000L;
        for (int k = 0; k < poseCount; k++) {
            long dt = Math.abs(poseTs[k] - ts);
            if (dt < bestDt) {
                bestDt = dt;
                best = poses[k];
            }
        }
        return best;
    }
}
