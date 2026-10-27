package com.sphere360.app.probe;

import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;

import com.sphere360.app.gimbal.OsmoGimbal;

import java.io.PrintWriter;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Dò vùng hướng máy với tới được ở tư thế đặt gimbal hiện tại: quét pan hết tầm, rồi ở STATIONS trạm trải đều
 * dọc tầm pan, ngẩng tới khi kẹt rồi cúi tới khi kẹt. Ghi hướng máy (ma trận xoay từ con quay) và gói trạng thái
 * gimbal ra CSV để tính độ phủ mặt cầu của tư thế (tools/gimbal_cover.py, tools/gimbal_sweet.py). Không đụng tới roll: OM7 nhận
 * lệnh tốc độ roll nhưng rung lắc tại chỗ và tự đưa máy về nằm ngang.
 */
final class GimbalMapper {

    interface Out {
        void log(String s);
    }

    private static final double SPEED = 35;           // °/s
    private static final long TICK_MS = 100;
    private static final long MIN_MS = 1000;          // chưa xét kẹt trước mốc này
    private static final long STALL_WINDOW_MS = 700;
    private static final double STALL_DEG = 0.6;      // quay ít hơn chừng này trong cửa sổ thì coi là kẹt
    private static final long MAX_MS = 14000;
    private static final int STATIONS = 10;

    private static final class Seg {
        final String name;
        final int axis;      // 0 pan, 1 tilt
        final int dir;
        final double deg;    // > 0: quay đúng chừng này (đo bằng con quay); 0: tới khi kẹt; < 0: nửa tầm tilt vừa đo

        Seg(String name, int axis, int dir, double deg) {
            this.name = name;
            this.axis = axis;
            this.dir = dir;
            this.deg = deg;
        }
    }

    private static final class Sample {
        final long t;
        final float[] r;

        Sample(long t, float[] r) {
            this.t = t;
            this.r = r;
        }
    }

    private final OsmoGimbal gimbal;
    private final Out out;
    private final PrintWriter csv;
    private final Handler h = new Handler(Looper.getMainLooper());
    private final List<Seg> segs = new ArrayList<>();
    private final ArrayDeque<Sample> history = new ArrayDeque<>();
    private int idx = -1;
    private long segStart;
    private float[] segStartR;
    private double segMaxDeg;        // góc lớn nhất đã quay so với đầu đoạn
    private double segPathDeg;       // tổng các bước quay nhỏ từ đầu đoạn (đo được cả tầm quá 180°)
    private double panArcDeg, tiltSpanDeg;
    private boolean running;
    private String phase = "idle";
    private long lastRow;
    private short[] status = new short[0];
    private Runnable onDone;

    GimbalMapper(OsmoGimbal gimbal, Out out, PrintWriter csv) {
        this.gimbal = gimbal;
        this.out = out;
        this.csv = csv;
        csv.println("t_ms,phase,r0,r1,r2,r3,r4,r5,r6,r7,r8,s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11");
    }

    boolean running() {
        return running;
    }

    void start(Runnable done) {
        onDone = done;
        segs.clear();
        segs.add(new Seg("pan-", 0, -1, 0));
        segs.add(new Seg("pan+", 0, 1, 0));   // quét hết tầm pan: đo panArcDeg
        running = true;
        idx = -1;
        next();
    }

    void stop() {
        running = false;
        h.removeCallbacks(tick);
        gimbal.speed(0, 0, 0, 0.3, 0);
        phase = "idle";
    }

    /** Mẫu con quay (ma trận xoay thiết bị → thế giới). */
    void onRotation(float[] r) {
        long now = SystemClock.elapsedRealtime();
        Sample prev = history.peekLast();
        if (prev != null && running) segPathDeg += angle(prev.r, r);
        history.addLast(new Sample(now, r.clone()));
        while (history.size() > 2 && now - history.peekFirst().t > 3000) history.removeFirst();
        if (running && now - lastRow >= 40) {
            lastRow = now;
            StringBuilder s = new StringBuilder();
            s.append(now).append(',').append(phase);
            for (int i = 0; i < 9; i++) s.append(',').append(String.format(Locale.US, "%.5f", r[i]));
            for (int i = 0; i < 12; i++) s.append(',').append(i < status.length ? status[i] : 0);
            csv.println(s);
        }
    }

    /** 24 byte đầu của gói trạng thái 04/05 (12 số int16). */
    void onStatus(byte[] payload) {
        int n = Math.min(12, payload.length / 2);
        short[] s = new short[n];
        for (int i = 0; i < n; i++) s[i] = (short) ((payload[2 * i] & 0xFF) | (payload[2 * i + 1] << 8));
        status = s;
    }

    private void next() {
        if (!running) return;
        idx++;
        if (idx == 2) planStations();
        if (idx >= segs.size()) {
            stop();
            out.log("Dò tầm xong");
            if (onDone != null) onDone.run();
            return;
        }
        Seg s = segs.get(idx);
        phase = s.name;
        segStart = SystemClock.elapsedRealtime();
        Sample last = history.peekLast();
        segStartR = last != null ? last.r : null;
        segMaxDeg = 0;
        segPathDeg = 0;
        h.post(tick);
    }

    /** Sau khi biết tầm pan: các trạm trải đều, đi từ đầu pan+ lùi dần về; ở mỗi trạm quét tilt hai phía. */
    private void planStations() {
        double step = panArcDeg / (STATIONS - 1);
        for (int k = 0; k < STATIONS; k++) {
            if (k > 0) segs.add(new Seg("move" + k, 0, -1, step));
            segs.add(new Seg("st" + k + "_tilt+", 1, 1, 0));
            segs.add(new Seg("st" + k + "_tilt-", 1, -1, 0));
            segs.add(new Seg("st" + k + "_tiltmid", 1, 1, -1));
        }
        segs.add(new Seg("panmid", 0, 1, panArcDeg / 2));
        out.log(String.format(Locale.US, "Tầm pan %.1f°, %d trạm cách nhau %.1f°", panArcDeg, STATIONS, step));
    }

    private final Runnable tick = new Runnable() {
        @Override
        public void run() {
            if (!running) return;
            Seg s = segs.get(idx);
            long t = SystemClock.elapsedRealtime() - segStart;
            double turned = turnedSinceStart();
            segMaxDeg = Math.max(segMaxDeg, turned);
            double want = s.deg < 0 ? tiltSpanDeg / 2 : s.deg;
            boolean stalled = t > MIN_MS && sweptDeg(STALL_WINDOW_MS) < STALL_DEG;
            boolean end = want > 0 ? (turned >= want || stalled) : stalled;
            if (end || t > MAX_MS) {
                gimbal.speed(0, 0, 0, 0.3, 0);
                if (s.name.equals("pan+")) panArcDeg = segPathDeg;
                if (s.name.endsWith("tilt-")) tiltSpanDeg = segMaxDeg;
                out.log(String.format(Locale.US, "  %-12s %5d ms, quay %.1f° (cộng dồn %.1f°)%s", s.name, t, turned,
                        segPathDeg, stalled ? " (kẹt)" : ""));
                h.postDelayed(GimbalMapper.this::next, 400);
                return;
            }
            double v = s.dir * SPEED;
            gimbal.speed(s.axis == 0 ? v : 0, s.axis == 1 ? v : 0, 0, 0.3, 0);
            h.postDelayed(this, TICK_MS);
        }
    };

    private double turnedSinceStart() {
        Sample now = history.peekLast();
        if (now == null || segStartR == null) return 0;
        return angle(segStartR, now.r);
    }

    /** Góc máy đã quay trong windowMs vừa qua (độ). */
    private double sweptDeg(long windowMs) {
        Sample now = history.peekLast();
        if (now == null) return 999;
        Sample old = null;
        for (Sample e : history) {
            if (now.t - e.t <= windowMs) {
                old = e;
                break;
            }
        }
        if (old == null || now.t - old.t < windowMs * 0.7) return 999;
        return angle(old.r, now.r);
    }

    /** Góc giữa hai phép xoay (độ): từ vết của AᵀB. */
    private static double angle(float[] a, float[] b) {
        double tr = 0;
        for (int i = 0; i < 9; i++) tr += a[i] * b[i];
        return Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2))));
    }
}
