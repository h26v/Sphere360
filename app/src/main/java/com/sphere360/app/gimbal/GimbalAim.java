package com.sphere360.app.gimbal;

import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;

import java.util.Locale;

/**
 * Đưa máy (đang kẹp trên gimbal) tới một hướng cho trước bằng vòng kín: đọc hướng máy từ con quay của chính điện
 * thoại, gửi lệnh tốc độ cho gimbal tỉ lệ với độ lệch, tới nơi thì dừng. OM7 bỏ qua lệnh quay theo góc nhưng nhận
 * lệnh tốc độ; hướng đích lại tính trong hệ thế giới như kế hoạch chụp, nên không cần biết góc riêng của gimbal.
 * yaw tăng khi xoay phải, pitch tăng khi ngẩng lên (cùng quy ước với CapturePlan). Chạy trên luồng chính.
 */
public final class GimbalAim {

    public interface Log {
        void line(String s);
    }

    public interface Done {
        void onDone(boolean ok, String summary);
    }

    /** Tốc độ lệnh (°/s) trên mỗi độ lệch. */
    private static final double KP = 2.5;
    private static final double MAX_DPS = 60;
    /** Không gần đích thêm được chừng này (độ) trong STALL_MS thì coi là kẹt ở giới hạn khớp. */
    private static final double STALL_GAIN_DEG = 0.3;
    private static final long STALL_MS = 1000;
    /** Tốc độ nhỏ nhất còn lệch: dưới mức này gimbal có thể không nhúc nhích. */
    private static final double MIN_DPS = 3;
    /** Coi là tới nơi khi lệch dưới mức này (độ) vài nhịp liền. */
    private static final double TOL_DEG = 0.4;
    private static final long PERIOD_MS = 100;
    /** Mỗi lệnh tốc độ chỉ kéo dài chừng này: mất kết nối giữa chừng thì gimbal tự dừng. */
    private static final double CMD_SECONDS = 0.3;
    private static final long TIMEOUT_MS = 10000;

    private final OsmoGimbal gimbal;
    private final Log log;
    private final Handler h = new Handler(Looper.getMainLooper());

    private volatile double yaw, pitch;
    private volatile boolean have;
    private double signYaw = 1, signPitch = 1;

    private boolean active;
    private double maxDps = MAX_DPS;
    // Cung pan với tới được (đo bằng GimbalDriver): yaw đi từ arcStart theo chiều arcDir hết arcLen độ. Có cung
    // thì độ lệch yaw tính dọc theo cung, không đi đường tắt qua chỗ gimbal không quay tới.
    private boolean hasArc;
    private double arcStart, arcLen;
    private int arcDir = 1;
    private double bestErr;
    private long bestMs;
    private double targetYaw, targetPitch;
    private long startMs;
    private int settled, steps;
    private Done done;

    public GimbalAim(OsmoGimbal gimbal, Log log) {
        this.gimbal = gimbal;
        this.log = log;
    }

    /** Hướng camera sau hiện tại (độ). Gọi mỗi khi có mẫu con quay. */
    public void onOrientation(double yawDeg, double pitchDeg) {
        yaw = yawDeg;
        pitch = pitchDeg;
        have = true;
    }

    /** Hướng camera sau từ ma trận xoay thiết bị → thế giới (SensorManager.getRotationMatrixFromVector). */
    public static double[] cameraYawPitch(float[] r) {
        double x = -r[2], y = -r[5], z = -r[8];
        return new double[]{Math.toDegrees(Math.atan2(x, y)), Math.toDegrees(Math.asin(Math.max(-1, Math.min(1, z))))};
    }

    public double yaw() {
        return yaw;
    }

    public double pitch() {
        return pitch;
    }

    public boolean busy() {
        return active;
    }

    /** Giới hạn tốc độ quay (°/s): chậm hơn thì khung nối quay video ít nhòe hơn. */
    public void setMaxSpeed(double dps) {
        maxDps = dps;
    }

    /** Cung pan với tới được: từ arcStartYaw, theo chiều dir (+1: yaw tăng), dài lenDeg. */
    public void setArc(double arcStartYaw, int dir, double lenDeg) {
        hasArc = true;
        arcStart = arcStartYaw;
        arcDir = dir >= 0 ? 1 : -1;
        arcLen = lenDeg;
    }

    public double arcLen() {
        return arcLen;
    }

    public double arcStart() {
        return arcStart;
    }

    public int arcDir() {
        return arcDir;
    }

    /** Vị trí của hướng yaw trên cung pan (độ tính từ đầu cung); ngoài cung thì âm hoặc lớn hơn arcLen. */
    public double arcPos(double yawDeg) {
        double s = ((yawDeg - arcStart) * arcDir) % 360;
        if (s < 0) s += 360;
        // Phần ngoài cung chia đôi: nửa gần đầu cung tính là âm.
        if (s > arcLen + (360 - arcLen) / 2) s -= 360;
        return s;
    }

    public void stop() {
        h.removeCallbacks(step);
        active = false;
        gimbal.speed(0, 0, 0, CMD_SECONDS, 0);
    }

    /** Quay tới hướng tuyệt đối (yaw, pitch) theo độ. */
    public void goTo(double yawDeg, double pitchDeg, Done d) {
        if (!have) {
            d.onDone(false, "chưa có hướng máy");
            return;
        }
        h.removeCallbacks(step);
        targetYaw = yawDeg;
        targetPitch = pitchDeg;
        startMs = SystemClock.elapsedRealtime();
        settled = 0;
        steps = 0;
        bestErr = Double.MAX_VALUE;
        bestMs = startMs;
        done = d;
        active = true;
        log.line(String.format(Locale.US, "đi tới yaw %.1f pitch %.1f (đang ở %.1f %.1f)", yawDeg, pitchDeg, yaw, pitch));
        h.post(step);
    }

    /** Quay thêm một khoảng so với hướng hiện tại. */
    public void goBy(double dYaw, double dPitch, Done d) {
        goTo(wrap(yaw + dYaw), pitch + dPitch, d);
    }

    private final Runnable step = new Runnable() {
        @Override
        public void run() {
            if (!active) return;
            long t = SystemClock.elapsedRealtime() - startMs;
            double eYaw = hasArc
                    ? arcDir * (Math.max(0, Math.min(arcLen, arcPos(targetYaw))) - arcPos(yaw))
                    : wrap(targetYaw - yaw);
            double ePitch = targetPitch - pitch;
            boolean in = Math.abs(eYaw) < TOL_DEG && Math.abs(ePitch) < TOL_DEG;
            if (++steps % 5 == 0 || in)
                log.line(String.format(Locale.US, "  %5d ms lệch yaw %+.2f pitch %+.2f", t, eYaw, ePitch));
            if (in) {
                gimbal.speed(0, 0, 0, CMD_SECONDS, 0);
                if (++settled >= 3) {
                    finish(true, String.format(Locale.US, "tới nơi sau %d ms, lệch yaw %+.2f pitch %+.2f", t, eYaw, ePitch));
                    return;
                }
            } else {
                settled = 0;
                gimbal.speed(signYaw * command(eYaw), signPitch * command(ePitch), 0, CMD_SECONDS, 0);
            }
            double err = Math.abs(eYaw) + Math.abs(ePitch);
            long now = SystemClock.elapsedRealtime();
            if (err < bestErr - STALL_GAIN_DEG) {
                bestErr = err;
                bestMs = now;
            } else if (!in && now - bestMs > STALL_MS) {
                gimbal.speed(0, 0, 0, CMD_SECONDS, 0);
                finish(false, String.format(Locale.US, "kẹt ở giới hạn, còn lệch yaw %+.2f pitch %+.2f", eYaw, ePitch));
                return;
            }
            if (t > TIMEOUT_MS) {
                gimbal.speed(0, 0, 0, CMD_SECONDS, 0);
                finish(false, String.format(Locale.US, "hết giờ, còn lệch yaw %+.2f pitch %+.2f", eYaw, ePitch));
                return;
            }
            h.postDelayed(this, PERIOD_MS);
        }
    };

    private double command(double err) {
        if (Math.abs(err) < TOL_DEG) return 0;
        double v = KP * err;
        double mag = Math.max(MIN_DPS, Math.min(maxDps, Math.abs(v)));
        return Math.copySign(mag, v);
    }

    private void finish(boolean ok, String s) {
        active = false;
        log.line((ok ? "OK " : "LỖI ") + s);
        Done d = done;
        done = null;
        if (d != null) d.onDone(ok, s);
    }

    /**
     * Đẩy thử từng trục 20°/s trong 0,5 s rồi đo máy quay được bao nhiêu: biết chiều của lệnh so với hướng máy,
     * độ trễ và tốc độ thật. Cần máy đứng yên trên gimbal.
     */
    public void calibrate(Done d) {
        if (!have) {
            d.onDone(false, "chưa có hướng máy");
            return;
        }
        active = true;
        final double y0 = yaw, p0 = pitch;
        log.line(String.format(Locale.US, "hiệu chỉnh: đang ở yaw %.1f pitch %.1f", y0, p0));
        gimbal.speed(20, 0, 0, 0.5, 0);
        h.postDelayed(() -> {
            final double dy = wrap(yaw - y0), y1 = yaw, p1 = pitch;
            gimbal.speed(-20, 0, 0, 0.5, 0);  // quay về
            h.postDelayed(() -> {
                gimbal.speed(0, 20, 0, 0.5, 0);
                h.postDelayed(() -> {
                    double dp = pitch - p1;
                    gimbal.speed(0, -20, 0, 0.5, 0);
                    h.postDelayed(() -> {
                        active = false;
                        boolean ok = Math.abs(dy) > 2 && Math.abs(dp) > 2;
                        if (Math.abs(dy) > 2) signYaw = Math.signum(dy);
                        if (Math.abs(dp) > 2) signPitch = Math.signum(dp);
                        String s = String.format(Locale.US,
                                "lệnh yaw +10° → máy quay %+.1f°, lệnh pitch +10° → máy quay %+.1f° (dấu %+.0f %+.0f)",
                                dy, dp, signYaw, signPitch);
                        log.line((ok ? "OK " : "LỖI ") + s);
                        d.onDone(ok, s);
                    }, 1500);
                }, 1500);
            }, 1500);
        }, 1500);
    }

    public static double wrap(double deg) {
        deg %= 360;
        if (deg > 180) deg -= 360;
        if (deg < -180) deg += 360;
        return deg;
    }
}
