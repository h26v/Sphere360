package com.sphere360.app.capture;

import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.util.Log;

import com.sphere360.app.gimbal.GimbalAim;
import com.sphere360.app.gimbal.OsmoGimbal;

import java.util.ArrayDeque;
import java.util.List;
import java.util.Locale;

/**
 * Chụp tự động bằng gimbal Osmo Mobile: dò hai đầu tầm pan, rồi lần lượt đưa máy tới từng góc của kế hoạch chụp
 * (vòng kín theo con quay, {@link GimbalAim}) và báo màn chụp bấm máy. Góc ngoài tầm với (hàng trên/dưới ở một
 * số hướng, khe pan gimbal không quay tới) thì gimbal dừng ở chỗ xa nhất tới được và vẫn chụp: ảnh lưu theo hướng
 * thật đo bằng con quay nên bộ ghép vẫn dùng được, chỉ là vùng phủ hụt đi một chút. Chạy trên luồng chính.
 */
final class GimbalDriver {

    interface Host {
        void gimbalStatus(String s);

        /** Camera sẵn sàng và không đang chụp dở; requireSteady: máy còn phải đứng yên đủ để chụp. */
        boolean readyToShoot(boolean requireSteady);

        void shoot(int target);

        /** Chờ người dùng xoay gimbal quanh ốc gắn (nhắc bằng msg), rồi gọi continueAfterRotate(). */
        void askRotate(String msg);

        /** Đã chụp hết, kể cả các tấm bù khe pan: mở màn ghép. */
        void allDone();
    }

    private static final String TAG = "GimbalDriver";
    /** Tốc độ quét dò tầm và tốc độ quay tối đa giữa các tấm (°/s): chậm cho khung nối quay video ít nhòe. */
    private static final double SWEEP_DPS = 30;
    private static final double MOVE_DPS = 30;
    private static final long TICK_MS = 100;
    private static final long SWEEP_MIN_MS = 1000;
    private static final long SWEEP_MAX_MS = 20000;
    private static final long STALL_WINDOW_MS = 700;
    private static final double STALL_DEG = 0.6;
    /** Chờ máy đứng yên tối đa chừng này rồi chụp luôn (tấm nhòe sẽ bị màn chụp bỏ và chụp lại). */
    private static final long SETTLE_MAX_MS = 3000;
    /**
     * Chỗ khép vòng (hai đầu tầm pan) phải chồng nhau ít nhất chừng này phần khung ngang; thiếu thì nhắc xoay gimbal
     * để chụp bù mỗi hàng một tấm giữa khe pan.
     */
    private static final double WRAP_OVERLAP = 0.35;

    private enum State { WAIT, SWEEP_MINUS, SWEEP_PLUS, MOVE, SETTLE, SHOOT, AWAIT_ROTATE, PAUSED, DONE }

    private final OsmoGimbal gimbal;
    private final GimbalAim aim;
    private final List<CapturePlan.Target> targets;
    private final Host host;
    private final double hFovDeg;
    private final Handler h = new Handler(Looper.getMainLooper());

    private State state = State.WAIT;
    private double yaw, pitch;
    private boolean have, arcKnown;
    // Quét dò tầm: yaw cộng dồn (không quấn về ±180) và lịch sử để biết khi nào kẹt.
    private double unwrapped, sweepBase;
    private final ArrayDeque<double[]> hist = new ArrayDeque<>();
    private long stateStart;
    private double arcStartYaw;
    private int current = -1;
    private int lastRow;
    // Tấm bù khe pan: {yaw, pitch, chỉ số target dùng lại}. Chụp sau khi người dùng xoay gimbal.
    private final java.util.ArrayList<double[]> gapShots = new java.util.ArrayList<>();
    private boolean gapPhase;
    private double[] currentGap;

    GimbalDriver(OsmoGimbal gimbal, GimbalAim aim, List<CapturePlan.Target> targets, double hFovDeg, Host host) {
        this.gimbal = gimbal;
        this.aim = aim;
        this.targets = targets;
        this.hFovDeg = hFovDeg;
        this.host = host;
        aim.setMaxSpeed(MOVE_DPS);
    }

    boolean started() {
        return state != State.WAIT;
    }

    /** Hướng camera hiện tại trong hệ của kế hoạch chụp (độ). */
    void onOrientation(double yawDeg, double pitchDeg) {
        if (have) unwrapped += GimbalAim.wrap(yawDeg - yaw);
        yaw = yawDeg;
        pitch = pitchDeg;
        have = true;
        aim.onOrientation(yawDeg, pitchDeg);
        long now = SystemClock.elapsedRealtime();
        hist.addLast(new double[]{now, unwrapped});
        while (hist.size() > 2 && now - hist.peekFirst()[0] > 2000) hist.removeFirst();
    }

    /** Bắt đầu khi gimbal đã nối, camera đã sẵn sàng và đã có hướng máy. */
    void start() {
        if (state != State.WAIT || !have) return;
        Log.i(TAG, "bắt đầu: dò tầm pan");
        enter(State.SWEEP_MINUS);
        host.gimbalStatus("Gimbal: dò tầm quay ngang…");
        h.post(sweepTick);
    }

    void pause() {
        h.removeCallbacksAndMessages(null);
        aim.stop();
        if (state != State.WAIT && state != State.DONE) state = State.PAUSED;
    }

    void resume() {
        if (state != State.PAUSED) return;
        if (!arcKnown) {
            enter(State.SWEEP_MINUS);
            h.post(sweepTick);
        } else {
            next();
        }
    }

    /** Người dùng đã xoay gimbal: dò lại tầm pan (tay cầm đã đổi hướng) rồi chụp các tấm bù khe. */
    void continueAfterRotate() {
        if (state != State.AWAIT_ROTATE) return;
        gapPhase = true;
        arcKnown = false;
        host.gimbalStatus("Gimbal: dò lại tầm quay ngang…");
        enter(State.SWEEP_MINUS);
        h.post(sweepTick);
    }

    /** Màn chụp báo tấm vừa chụp: nhận (sang góc tiếp) hay bỏ vì nhòe (chụp lại góc này). */
    void onShot(boolean accepted) {
        if (state != State.SHOOT) return;
        h.removeCallbacks(shootWatchdog);
        if (accepted) {
            if (currentGap != null) gapShots.remove(currentGap);
            currentGap = null;
            next();
        } else {
            enter(State.SETTLE);
            h.post(settleTick);
        }
    }

    private void enter(State s) {
        state = s;
        stateStart = SystemClock.elapsedRealtime();
    }

    private final Runnable sweepTick = new Runnable() {
        @Override
        public void run() {
            if (state != State.SWEEP_MINUS && state != State.SWEEP_PLUS) return;
            long t = SystemClock.elapsedRealtime() - stateStart;
            boolean stalled = t > SWEEP_MIN_MS && swept(STALL_WINDOW_MS) < STALL_DEG;
            if (stalled || t > SWEEP_MAX_MS) {
                gimbal.speed(0, 0, 0, 0.3, 0);
                if (state == State.SWEEP_MINUS) {
                    arcStartYaw = yaw;
                    sweepBase = unwrapped;
                    enter(State.SWEEP_PLUS);
                } else {
                    double arc = unwrapped - sweepBase;
                    aim.setArc(arcStartYaw, arc >= 0 ? 1 : -1, Math.abs(arc));
                    arcKnown = true;
                    Log.i(TAG, String.format(Locale.US, "tầm pan %.1f° từ yaw %.1f, chiều %+d", Math.abs(arc),
                            arcStartYaw, arc >= 0 ? 1 : -1));
                    next();
                    return;
                }
            } else {
                double v = state == State.SWEEP_MINUS ? -SWEEP_DPS : SWEEP_DPS;
                gimbal.speed(v, 0, 0, 0.3, 0);
            }
            h.postDelayed(this, TICK_MS);
        }
    };

    /** Yaw đã quay trong windowMs vừa qua (độ). */
    private double swept(long windowMs) {
        double[] last = hist.peekLast();
        if (last == null) return 999;
        for (double[] e : hist) {
            if (last[0] - e[0] <= windowMs) {
                return last[0] - e[0] < windowMs * 0.7 ? 999 : Math.abs(last[1] - e[1]);
            }
        }
        return 999;
    }

    /** Chọn góc tiếp theo: tấm mốc (target 0) trước, rồi lần lượt từng hàng, trong hàng lấy góc gần nhất dọc cung pan. */
    private void next() {
        if (gapPhase) {
            nextGap();
            return;
        }
        int pick = -1;
        if (!targets.get(0).done) {
            pick = 0;
        } else {
            double here = aim.arcPos(yaw);
            double best = Double.MAX_VALUE;
            int[] rowOrder = {lastRow, 0, 1, 2, 3, 4};
            for (int row : rowOrder) {
                for (int i = 0; i < targets.size(); i++) {
                    CapturePlan.Target t = targets.get(i);
                    if (t.done || t.row != row) continue;
                    double d = row >= 3 ? 0 : Math.abs(clampArc(t.yawDeg) - here);
                    if (d < best) {
                        best = d;
                        pick = i;
                    }
                }
                if (pick >= 0) break;
            }
        }
        if (pick < 0) {
            aim.stop();
            if (planGapShots()) {
                enter(State.AWAIT_ROTATE);
                String msg = String.format(Locale.US, "Xoay gimbal quanh ốc gắn khoảng nửa vòng (giữ nguyên chân máy), " +
                        "rồi bấm \"Đã xoay xong\" để chụp bù %d tấm", gapShots.size());
                host.gimbalStatus("Gimbal: " + msg);
                host.askRotate(msg);
            } else {
                finishAll();
            }
            return;
        }
        current = pick;
        CapturePlan.Target t = targets.get(pick);
        lastRow = t.row;
        enter(State.MOVE);
        int done = 0;
        for (CapturePlan.Target x : targets) if (x.done) done++;
        host.gimbalStatus(String.format(Locale.US, "Gimbal: tới tấm %d/%d (%s)", done + 1, targets.size(),
                CapturePlan.ROW_NAMES[t.row]));
        // Tấm đỉnh/đáy: giữ hướng ngang hiện tại, chỉ ngẩng/cúi hết mức.
        double ty = t.row >= 3 ? yaw : t.yawDeg;
        aim.goTo(ty, t.pitchDeg, (ok, msg) -> {
            if (state != State.MOVE) return;
            Log.i(TAG, "tấm " + current + ": " + msg);
            enter(State.SETTLE);
            h.postDelayed(settleTick, 200);
        });
    }

    private void finishAll() {
        enter(State.DONE);
        aim.stop();
        host.gimbalStatus("Gimbal: đã chụp xong");
        host.allDone();
    }

    /**
     * Chỗ khép vòng: hai đầu tầm pan cộng nửa khung mỗi bên. Chồng nhau ít quá (hoặc hở) thì lên danh sách tấm bù:
     * mỗi hàng (trừ đỉnh, đáy) một tấm ở giữa khe, cùng góc ngẩng với hàng đó.
     */
    private boolean planGapShots() {
        gapShots.clear();
        double gap = 360 - aim.arcLen();
        double overlap = hFovDeg - gap;
        Log.i(TAG, String.format(Locale.US, "khe pan %.1f°, chỗ khép vòng chồng %.1f° (cần %.1f°)", gap, overlap,
                WRAP_OVERLAP * hFovDeg));
        if (overlap >= WRAP_OVERLAP * hFovDeg) return false;
        double gapYaw = GimbalAim.wrap(aim.arcStart() + aim.arcDir() * (aim.arcLen() + gap / 2));
        for (int row = 0; row <= 2; row++) {
            int idx = -1;
            double best = Double.MAX_VALUE;
            for (int i = 0; i < targets.size(); i++) {
                CapturePlan.Target t = targets.get(i);
                if (t.row != row) continue;
                double d = Math.abs(GimbalAim.wrap(t.yawDeg - gapYaw));
                if (d < best) {
                    best = d;
                    idx = i;
                }
            }
            if (idx >= 0) gapShots.add(new double[]{gapYaw, targets.get(idx).pitchDeg, idx});
        }
        return !gapShots.isEmpty();
    }

    private void nextGap() {
        if (gapShots.isEmpty()) {
            finishAll();
            return;
        }
        // Gần nhất dọc cung pan mới.
        double here = aim.arcPos(yaw);
        double[] pick = null;
        double best = Double.MAX_VALUE;
        for (double[] g : gapShots) {
            double d = Math.abs(clampArc(g[0]) - here) + Math.abs(g[1] - pitch) * 0.01;
            if (d < best) {
                best = d;
                pick = g;
            }
        }
        currentGap = pick;
        current = (int) pick[2];
        enter(State.MOVE);
        host.gimbalStatus(String.format(Locale.US, "Gimbal: chụp bù khe (%s), còn %d tấm",
                CapturePlan.ROW_NAMES[targets.get(current).row], gapShots.size()));
        aim.goTo(pick[0], pick[1], (ok, msg) -> {
            if (state != State.MOVE) return;
            Log.i(TAG, "tấm bù khe " + current + ": " + msg);
            enter(State.SETTLE);
            h.postDelayed(settleTick, 200);
        });
    }

    private double clampArc(double yawDeg) {
        return Math.max(0, Math.min(aim.arcLen(), aim.arcPos(yawDeg)));
    }

    /** Lệnh chụp không ra ảnh (camera bận, lỗi): chụp lại góc này. */
    private final Runnable shootWatchdog = () -> {
        if (state != State.SHOOT) return;
        Log.w(TAG, "chụp tấm " + current + " không thấy kết quả, chụp lại");
        enter(State.SETTLE);
        h.post(this.settleTick);
    };

    private final Runnable settleTick = new Runnable() {
        @Override
        public void run() {
            if (state != State.SETTLE) return;
            boolean late = SystemClock.elapsedRealtime() - stateStart > SETTLE_MAX_MS;
            if (host.readyToShoot(!late)) {
                enter(State.SHOOT);
                host.shoot(current);
                h.removeCallbacks(shootWatchdog);
                h.postDelayed(shootWatchdog, 8000);
                return;
            }
            h.postDelayed(this, 50);
        }
    };
}
