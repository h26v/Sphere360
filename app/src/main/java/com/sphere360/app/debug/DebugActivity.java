package com.sphere360.app.debug;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Typeface;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.WindowManager;
import android.widget.ScrollView;
import android.widget.TextView;

import org.json.JSONObject;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;

import com.sphere360.app.data.Session;
import com.sphere360.app.stitch.NativeStitcher;
import com.sphere360.app.stitch.StitchInput;
import com.sphere360.app.ui.Ui;

/**
 * Lệnh debug qua adb, không cần bấm giao diện. Chỉ adb gọi được (manifest đòi quyền DUMP, app thường không có).
 *
 * <pre>
 * adb shell am start -n com.sphere360.app/.debug.DebugActivity --es cmd stitch --es session 20261023_171607 \
 *     [--ei width 8192] [--ez deghost true] [--es out dbg.jpg] [--es params "kSeamParallaxWeight=4,kDebugSeams=1"]
 * adb shell am start -n com.sphere360.app/.debug.DebugActivity --es cmd params
 * </pre>
 *
 * Khung cầu nối của lượt (nếu có) được dùng khi kUseBridges=1 (mặc định); thêm "kUseBridges=0" vào params để so.
 * Lệnh stitch ghi vào thư mục lượt chụp: ảnh kết quả (out, mặc định debug.jpg), log bộ ghép (debug.log), ảnh
 * đường nối nếu kDebugSeams (debug_seams.jpg) và cuối cùng là debug.done (JSON: ok, error, ms, width, params).
 * Không đụng tới pano.jpg, session.json hay Thư viện ảnh. session = "latest" là lượt mới nhất. Lệnh params ghi
 * danh sách tham số ra files/params.txt. Mỗi lần chạy cũng in một dòng thẻ Sphere360Debug ra logcat.
 */
public class DebugActivity extends Activity {

    private static final String TAG = "Sphere360Debug";
    private static volatile boolean running;

    private TextView text;
    private final Handler main = new Handler(Looper.getMainLooper());

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        text = new TextView(this);
        text.setTextColor(Ui.TEXT);
        text.setTypeface(Typeface.MONOSPACE);
        text.setTextSize(12);
        int pad = (int) (16 * getResources().getDisplayMetrics().density);
        text.setPadding(pad, pad, pad, pad);
        ScrollView scroll = new ScrollView(this);
        scroll.setBackgroundColor(Ui.BG);
        scroll.addView(text);
        Ui.padForSystemBars(scroll, true, true);
        setContentView(scroll);
        handle(getIntent());
    }

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        handle(intent);
    }

    private void show(String line) {
        Log.i(TAG, line);
        main.post(() -> text.append(line + "\n"));
    }

    private void handle(Intent intent) {
        String cmd = intent.getStringExtra("cmd");
        if (cmd == null) cmd = "stitch";
        switch (cmd) {
            case "stitch":
                stitch(intent);
                break;
            case "params":
                try {
                    String all = NativeStitcher.describeParams();
                    File f = new File(getExternalFilesDir(null), "params.txt");
                    writeText(f, all);
                    show(all);
                    show("DONE params " + f.getAbsolutePath());
                } catch (IOException e) {
                    show("ERROR " + e);
                }
                break;
            default:
                show("ERROR lệnh không biết: " + cmd);
        }
    }

    private void stitch(Intent intent) {
        if (running) {
            show("ERROR đang ghép một lượt khác");
            return;
        }
        String id = intent.getStringExtra("session");
        File dir;
        if (id == null || id.equals("latest")) {
            File[] all = Session.root(this).listFiles(File::isDirectory);
            if (all == null || all.length == 0) {
                show("ERROR chưa có lượt chụp nào");
                return;
            }
            Arrays.sort(all);
            dir = all[all.length - 1];
        } else {
            dir = new File(Session.root(this), id);
        }
        String outName = intent.getStringExtra("out");
        if (outName == null) outName = "debug.jpg";
        String stem = outName.endsWith(".jpg") ? outName.substring(0, outName.length() - 4) : outName;
        File out = new File(dir, stem + ".jpg");
        File log = new File(dir, stem + ".log");
        File done = new File(dir, stem + ".done");
        //noinspection ResultOfMethodCallIgnored
        done.delete();
        int widthReq = intent.getIntExtra("width", 8192);
        boolean deghost = intent.getBooleanExtra("deghost", true);
        int quality = intent.getIntExtra("quality", 92);
        String params = intent.getStringExtra("params");
        if (params == null) params = "";
        final String spec = params;

        running = true;
        show("stitch " + dir.getName() + " -> " + out.getName() + " width=" + widthReq + " deghost=" + deghost
                + " params=" + spec);
        Thread t = new Thread(null, () -> {
            long t0 = System.currentTimeMillis();
            JSONObject res = new JSONObject();
            String err = null;
            int width = 0;
            try {
                Session s = Session.load(dir);
                StitchInput in = StitchInput.from(s);
                int nativeWidth = s.nativeOutputWidth();
                width = widthReq <= 0 ? nativeWidth : Math.min(widthReq, nativeWidth);
                err = NativeStitcher.setParams(spec);
                if (err == null) {
                    NativeStitcher.setLogFile(log.getAbsolutePath());
                    final int[] last = {-1};
                    err = NativeStitcher.stitch(in, width, quality, deghost, out.getAbsolutePath(),
                            (percent, stage) -> {
                                if (percent / 10 != last[0] / 10) show(percent + "% " + stage);
                                last[0] = percent;
                            });
                }
            } catch (Throwable e) {
                Log.e(TAG, "stitch", e);
                err = e.toString();
            } finally {
                NativeStitcher.setLogFile(null);
                NativeStitcher.setParams("");
                long ms = System.currentTimeMillis() - t0;
                try {
                    res.put("ok", err == null);
                    res.put("error", err == null ? "" : err);
                    res.put("ms", ms);
                    res.put("width", width);
                    res.put("params", spec);
                    res.put("out", out.getAbsolutePath());
                    writeText(done, res.toString(1));
                } catch (Exception e) {
                    Log.e(TAG, "done", e);
                }
                show((err == null ? "DONE " : "ERROR ") + ms + " ms " + (err == null ? out.getAbsolutePath() : err));
                running = false;
            }
        }, "debug-stitch", 64L << 20);
        t.setPriority(Thread.MAX_PRIORITY);
        t.start();
    }

    private static void writeText(File f, String s) throws IOException {
        try (FileOutputStream os = new FileOutputStream(f)) {
            os.write(s.getBytes(StandardCharsets.UTF_8));
        }
    }
}
