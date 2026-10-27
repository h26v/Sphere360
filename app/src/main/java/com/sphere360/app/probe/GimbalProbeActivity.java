package com.sphere360.app.probe;

import android.Manifest;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.sphere360.app.gimbal.Duml;
import com.sphere360.app.gimbal.GimbalAim;
import com.sphere360.app.gimbal.OsmoGimbal;
import com.sphere360.app.ui.Ui;

import java.util.Locale;

/**
 * Màn hình thử: gimbal Osmo Mobile có nhận lệnh quay qua Bluetooth không. Ghi log lên màn hình và logcat (thẻ
 * "GimbalProbe"). Điều khiển được từ adb (manifest đòi quyền DUMP nên chỉ adb và chính app mở được):
 *
 * <pre>
 * adb shell am start -n com.sphere360.app/.probe.GimbalProbeActivity --es do connect [--es addr AA:BB:..]
 * adb shell am start -n com.sphere360.app/.probe.GimbalProbeActivity --es do calib|left|right|up|down|stop
 * adb shell am start -n com.sphere360.app/.probe.GimbalProbeActivity --es do goto --ef dyaw 40 --ef dpitch 0
 * adb shell am start -n com.sphere360.app/.probe.GimbalProbeActivity --es do raw --ei set 4 --ei cmd 20 --es payload "..."
 * </pre>
 *
OM7 bỏ qua lệnh quay theo góc (do angle, center) nhưng nhận lệnh tốc độ (do spin), nên các nút quay đi
 * theo vòng kín {@link GimbalAim}. Thêm --ei type 64 để xin gimbal trả lời sau khi làm lệnh (mặc định 0: không xin, như om-research).
 */
public class GimbalProbeActivity extends Activity implements OsmoGimbal.Listener, SensorEventListener {

    private static final String TAG = "GimbalProbe";
    private static final int REQ_PERM = 1;

    private Ui u;
    private TextView state, live, log;
    private ScrollView scroll;
    private OsmoGimbal gimbal;
    private volatile float[] attitude;
    private Intent pending;
    private GimbalAim aim;
    private SensorManager sensors;
    private final float[] rot = new float[9];
    private java.io.PrintWriter file;
    private final android.os.Handler main = new android.os.Handler(android.os.Looper.getMainLooper());
    /** Chuỗi thử tự động: hiệu chỉnh rồi lần lượt đi các bước (yaw, pitch) để dò giới hạn hai trục. */
    private static final double[][] AUTO_MOVES = {{0, 60}, {0, -90}, {0, 30},
            {45, 0}, {45, 0}, {45, 0}, {45, 0}, {45, 0}, {45, 0}, {45, 0}, {45, 0},
            {-45, 0}, {-45, 0}, {-45, 0}, {-45, 0}, {-45, 0}, {-45, 0}, {-45, 0}, {-45, 0}};
    private boolean autoRunning;
    private GimbalMapper mapper;
    private java.io.PrintWriter mapCsv;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        u = new Ui(this);
        Ui.paintBars(this);

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setBackgroundColor(Ui.BG);
        root.setPadding(u.px(16), u.px(12), u.px(16), u.px(12));
        Ui.padForSystemBars(root, true, true);

        root.addView(u.bold("Thử gimbal Osmo Mobile", 22, Ui.TEXT));
        root.addView(u.text("Bật gimbal, gắn máy lên. Thoát hẳn DJI Mimo trước (gimbal chỉ nối với một app một lúc), " +
                "rồi bấm Kết nối.", 13, Ui.TEXT_2), u.lp(-1, -2, 4));

        state = u.bold("Chưa kết nối", 15, Ui.ACCENT);
        root.addView(state, u.lp(-1, -2, 10));
        live = u.text("", 12, Ui.TEXT_2);
        root.addView(live, u.lp(-1, -2, 4));

        LinearLayout r1 = u.row();
        r1.addView(button("Kết nối", true, () -> connect(null)), u.weight(0));
        r1.addView(button("Hiệu chỉnh", false, () -> act("calib", 0)), u.weight(8));
        r1.addView(button("Dừng", false, () -> act("stop", 0)), u.weight(8));
        root.addView(r1, u.lp(-1, -2, 10));
        r1 = u.row();
        r1.addView(button("Thử tự động (sau 5 giây)", true, () -> act("auto", 0)), u.weight(0));
        root.addView(r1, u.lp(-1, -2, 10));
        LinearLayout r2 = u.row();
        r2.addView(button("◀ 30°", false, () -> act("left", 0)), u.weight(0));
        r2.addView(button("30° ▶", false, () -> act("right", 0)), u.weight(8));
        r2.addView(button("▲ 15°", false, () -> act("up", 0)), u.weight(8));
        r2.addView(button("▼ 15°", false, () -> act("down", 0)), u.weight(8));
        root.addView(r2, u.lp(-1, -2, 8));

        scroll = new ScrollView(this);
        log = u.text("", 11, Ui.TEXT);
        log.setTypeface(android.graphics.Typeface.MONOSPACE);
        scroll.addView(log);
        root.addView(scroll, new LinearLayout.LayoutParams(-1, 0, 1));
        setContentView(root);

        openLogFile();
        gimbal = new OsmoGimbal(this, this);
        aim = new GimbalAim(gimbal, this::onLog);
        sensors = (SensorManager) getSystemService(SENSOR_SERVICE);
        sensors.registerListener(this, sensors.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR), 10000);
        live.postDelayed(liveTick, 1000);
        handle(getIntent());
    }

    private Button button(String label, boolean primary, Runnable r) {
        Button b = primary ? u.primary(label) : u.secondary(label);
        b.setPadding(u.px(6), u.px(12), u.px(6), u.px(12));
        b.setOnClickListener(v -> r.run());
        return b;
    }

    private final Runnable liveTick = new Runnable() {
        @Override
        public void run() {
            float[] a = attitude;
            String s = String.format(Locale.US, "Máy: yaw %.1f° pitch %.1f°%n", aim.yaw(), aim.pitch()) + gimbal.stats();
            if (a != null) s = String.format(Locale.US, "yaw %.1f° pitch %.1f° roll %.1f° (đoán)\n", a[0], a[1], a[2]) + s;
            live.setText(s);
            live.postDelayed(this, 1000);
        }
    };

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        handle(intent);
    }

    /** Lệnh từ adb. */
    private void handle(Intent intent) {
        String what = intent.getStringExtra("do");
        if (what == null) return;
        int type = intent.getIntExtra("type", 0);
        if (what.equals("goto")) {
            aim.goBy(intent.getFloatExtra("dyaw", 0), intent.getFloatExtra("dpitch", 0), (ok, m) -> { });
        } else if (what.equals("verbose")) {
            gimbal.verbose = intent.getBooleanExtra("on", true);
        } else if (what.equals("connect")) {
            connect(intent.getStringExtra("addr"));
        } else if (what.equals("raw")) {
            byte[] payload = Duml.unhex(intent.getStringExtra("payload") == null ? "" : intent.getStringExtra("payload"));
            gimbal.send(intent.getIntExtra("recv", 4), type, intent.getIntExtra("set", 4), intent.getIntExtra("cmd", 0x14), payload);
        } else {
            act(what, type);
        }
    }

    private void act(String what, int type) {
        switch (what) {
            case "left":
                aim.goBy(-30, 0, (ok, m) -> { });
                break;
            case "right":
                aim.goBy(30, 0, (ok, m) -> { });
                break;
            case "up":
                aim.goBy(0, 15, (ok, m) -> { });
                break;
            case "down":
                aim.goBy(0, -15, (ok, m) -> { });
                break;
            case "calib":
                aim.calibrate((ok, m) -> { });
                break;
            case "stop":
                autoRunning = false;
                if (mapper != null) mapper.stop();
                aim.stop();
                break;
            case "auto":
                runAuto();
                break;
            case "map":
                runMap();
                break;
            // Lệnh góc kiểu OM2–OM4 (OM7 bỏ qua), để thử.
            case "angle":
                gimbal.rotate(30, 0, 0, Duml.MODE_RELATIVE, 1.0, type);
                break;
            case "center":
                gimbal.rotate(0, 0, 0, Duml.MODE_ABSOLUTE, 1.5, type);
                break;
            case "spin":
                gimbal.speed(30, 0, 0, 2.0, type);
                break;
            default:
                onLog("Lệnh không biết: " + what);
        }
    }

    private void connect(String addr) {
        String[] perms = Build.VERSION.SDK_INT >= 31
                ? new String[]{Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT}
                : new String[]{Manifest.permission.ACCESS_FINE_LOCATION};
        for (String p : perms) {
            if (checkSelfPermission(p) != PackageManager.PERMISSION_GRANTED) {
                pending = new Intent().putExtra("do", "connect").putExtra("addr", addr);
                onState("Cần quyền Bluetooth", false);
                requestPermissions(perms, REQ_PERM);
                return;
            }
        }
        gimbal.connect(addr);
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] results) {
        super.onRequestPermissionsResult(requestCode, permissions, results);
        for (int r : results) {
            if (r != PackageManager.PERMISSION_GRANTED) {
                onState("Không có quyền Bluetooth", false);
                return;
            }
        }
        if (pending != null) {
            Intent i = pending;
            pending = null;
            handle(i);
        }
    }

    @Override
    protected void onDestroy() {
        live.removeCallbacks(liveTick);
        sensors.unregisterListener(this);
        aim.stop();
        gimbal.close();
        if (file != null) file.close();
        super.onDestroy();
    }

    /** Log ghi cả ra file (Android/data/com.sphere360.app/files/gimbal/) để thử được khi đã rút cáp. */
    private void openLogFile() {
        try {
            java.io.File dir = new java.io.File(getExternalFilesDir(null), "gimbal");
            dir.mkdirs();
            String name = new java.text.SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new java.util.Date());
            file = new java.io.PrintWriter(new java.io.FileWriter(new java.io.File(dir, "probe_" + name + ".txt")), true);
        } catch (java.io.IOException e) {
            Log.w(TAG, "không mở được file log", e);
        }
    }

    /** Dò tầm các khớp ở tư thế hiện tại, ghi files/gimbal/map_*.csv. */
    private void runMap() {
        if (mapper != null && mapper.running()) return;
        try {
            java.io.File dir = new java.io.File(getExternalFilesDir(null), "gimbal");
            String name = "map_" + new java.text.SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new java.util.Date()) + ".csv";
            mapCsv = new java.io.PrintWriter(new java.io.FileWriter(new java.io.File(dir, name)));
            onLog("Dò tầm: ghi " + name);
        } catch (java.io.IOException e) {
            onLog("Không mở được file dò tầm: " + e);
            return;
        }
        mapper = new GimbalMapper(gimbal, this::onLog, mapCsv);
        mapper.start(() -> {
            mapCsv.close();
            onState("Dò tầm xong", true);
        });
    }

    private void runAuto() {
        if (autoRunning) return;
        autoRunning = true;
        onLog("Thử tự động: bắt đầu sau 5 giây");
        main.postDelayed(() -> {
            if (!gimbal.isReady()) {
                onLog("Chưa kết nối gimbal, đang kết nối...");
                connect(null);
                main.postDelayed(this::autoCalib, 8000);
            } else {
                autoCalib();
            }
        }, 5000);
    }

    private void autoCalib() {
        if (!autoRunning) return;
        aim.calibrate((ok, m) -> main.postDelayed(() -> autoMove(0), 1000));
    }

    private void autoMove(int i) {
        if (!autoRunning) return;
        if (i >= AUTO_MOVES.length) {
            autoRunning = false;
            onLog("Thử tự động: xong");
            onState("Thử tự động xong, cắm cáp lại", true);
            return;
        }
        onLog(String.format(Locale.US, "bước %d: thêm yaw %+.0f pitch %+.0f", i + 1, AUTO_MOVES[i][0], AUTO_MOVES[i][1]));
        aim.goBy(AUTO_MOVES[i][0], AUTO_MOVES[i][1], (ok, m) -> {
            onLog(String.format(Locale.US, "  sau bước %d: máy ở yaw %.1f pitch %.1f", i + 1, aim.yaw(), aim.pitch()));
            main.postDelayed(() -> autoMove(i + 1), 1000);
        });
    }

    @Override
    public void onLog(String line) {
        Log.i(TAG, line);
        java.io.PrintWriter f = file;
        if (f != null) {
            synchronized (this) {
                f.println(new java.text.SimpleDateFormat("HH:mm:ss.SSS", Locale.US).format(new java.util.Date()) + " " + line);
            }
        }
        runOnUiThread(() -> {
            log.append(line + "\n");
            scroll.post(() -> scroll.fullScroll(ScrollView.FOCUS_DOWN));
        });
    }

    @Override
    public void onState(String s, boolean ready) {
        onLog("state: " + s);
        runOnUiThread(() -> {
            state.setText(s);
            state.setTextColor(ready ? Ui.GOOD : Ui.ACCENT);
        });
    }

    @Override
    public void onSensorChanged(SensorEvent e) {
        SensorManager.getRotationMatrixFromVector(rot, e.values);
        double[] yp = GimbalAim.cameraYawPitch(rot);
        if (mapper != null) mapper.onRotation(rot);
        aim.onOrientation(yp[0], yp[1]);
    }

    @Override
    public void onAccuracyChanged(Sensor s, int accuracy) {
    }

    @Override
    public void onPacket(Duml.Packet p) {
        float[] a = OsmoGimbal.attitude(p);
        if (a != null) attitude = a;
        if (mapper != null && p.cmdSet == 0x04 && p.cmd == 0x05) {
            byte[] pl = p.payload;
            runOnUiThread(() -> mapper.onStatus(pl));
        }
    }
}
