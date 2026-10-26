package com.sphere360.app.viewer;

/**
 * Làm mượt hướng máy cho chế độ xem bằng con quay: bộ lọc One Euro (Casiez và cộng sự, CHI 2012) trên
 * quaternion. Tần số cắt tăng theo tốc độ xoay: máy gần như đứng yên (tay run) thì lọc mạnh, xoay chủ ý thì
 * nới ra để không trễ. Lọc bằng slerp nên không lệch trục và không vướng điểm kỳ dị như lọc từng góc Euler.
 */
final class OrientationFilter {

    /** Tần số cắt khi đứng yên (Hz): thấp hơn thì mượt hơn nhưng trôi theo chậm hơn. */
    private static final double MIN_CUTOFF_HZ = 0.6;
    /** Mỗi rad/s tốc độ xoay cộng thêm bấy nhiêu Hz vào tần số cắt. */
    private static final double BETA = 10.0;
    /**
     * Tốc độ xoay đo trên hướng đã lọc sơ ở tần số này: run tay (8–12 Hz) bị lọc đi nên không làm bộ lọc tưởng
     * đang lia mà nới ra. Mô phỏng (run 0,3° ở 9 Hz, lắc 0,4° ở 2 Hz, lia 60°/s): biên độ rung lúc đứng yên
     * 1,68° -> 0,52°, trễ lúc lia 1,9°; đo tốc độ trên tín hiệu thô thì 0,76° và 2,2°.
     */
    private static final double SPEED_PREFILTER_HZ = 3.0;
    /** Tần số cắt khi làm mượt chính tốc độ xoay (để một cú giật không mở toang bộ lọc). */
    private static final double SPEED_CUTOFF_HZ = 1.2;

    private final double[] pre = new double[4];   // x, y, z, w đã lọc sơ (để đo tốc độ)
    private final double[] out = new double[4];   // x, y, z, w đã lọc
    private double speed;                         // rad/s, đã lọc
    private long lastNs;
    private boolean primed;

    /** Bắt đầu lại từ mẫu kế tiếp (không trượt từ hướng cũ tới). */
    void reset() {
        primed = false;
    }

    /**
     * Nhận một mẫu rotation vector (values của TYPE_GAME_ROTATION_VECTOR, x y z [w]) lúc timestampNs; scale > 0
     * nhân vào tần số cắt (nhỏ hơn 1 khi đang phóng to để lọc mạnh hơn). Trả về rotation vector đã lọc
     * (x, y, z, w) để đưa vào SensorManager.getRotationMatrixFromVector.
     */
    float[] filter(float[] values, long timestampNs, double scale) {
        double x = values[0], y = values[1], z = values[2];
        double w = values.length > 3 ? values[3] : Math.sqrt(Math.max(0, 1 - x * x - y * y - z * z));
        if (!primed) {
            set(pre, x, y, z, w);
            set(out, x, y, z, w);
            speed = 0;
            lastNs = timestampNs;
            primed = true;
            return result();
        }
        double dt = (timestampNs - lastNs) * 1e-9;
        lastNs = timestampNs;
        if (dt <= 0 || dt > 0.5) {
            // Mẫu trùng giờ hoặc vừa ngắt quãng lâu (tạm dừng): nhận luôn mẫu mới.
            set(pre, x, y, z, w);
            set(out, x, y, z, w);
            speed = 0;
            return result();
        }
        double px = pre[0], py = pre[1], pz = pre[2], pw = pre[3];
        slerpTowards(pre, x, y, z, w, alpha(SPEED_PREFILTER_HZ, dt));
        double instSpeed = angle(pre, px, py, pz, pw) / dt;
        speed += alpha(SPEED_CUTOFF_HZ, dt) * (instSpeed - speed);
        double cutoff = Math.max(0.05, scale) * (MIN_CUTOFF_HZ + BETA * speed);
        slerpTowards(out, x, y, z, w, alpha(cutoff, dt));
        return result();
    }

    private static double alpha(double cutoffHz, double dt) {
        double tau = 1.0 / (2 * Math.PI * cutoffHz);
        return 1.0 / (1.0 + tau / dt);
    }

    /** Góc quay (rad) giữa quaternion q và (x, y, z, w). */
    private static double angle(double[] q, double x, double y, double z, double w) {
        double d = Math.abs(q[0] * x + q[1] * y + q[2] * z + q[3] * w);
        return 2 * Math.acos(Math.min(1, d));
    }

    /** q <- slerp(q, (x, y, z, w), t). */
    private static void slerpTowards(double[] q, double x, double y, double z, double w, double t) {
        double d = q[0] * x + q[1] * y + q[2] * z + q[3] * w;
        if (d < 0) { // cùng một hướng có hai quaternion q và -q: chọn cái gần
            x = -x; y = -y; z = -z; w = -w;
            d = -d;
        }
        double a, b;
        if (d > 0.9995) {
            a = 1 - t;
            b = t;
        } else {
            double th = Math.acos(d), s = Math.sin(th);
            a = Math.sin((1 - t) * th) / s;
            b = Math.sin(t * th) / s;
        }
        double nx = a * q[0] + b * x, ny = a * q[1] + b * y, nz = a * q[2] + b * z, nw = a * q[3] + b * w;
        double n = Math.sqrt(nx * nx + ny * ny + nz * nz + nw * nw);
        set(q, nx / n, ny / n, nz / n, nw / n);
    }

    private static void set(double[] q, double x, double y, double z, double w) {
        q[0] = x;
        q[1] = y;
        q[2] = z;
        q[3] = w;
    }

    private float[] result() {
        // q và -q cho cùng một ma trận xoay nên dấu không quan trọng.
        return new float[]{(float) out[0], (float) out[1], (float) out[2], (float) out[3]};
    }
}
