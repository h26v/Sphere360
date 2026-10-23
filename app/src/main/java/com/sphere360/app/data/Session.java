package com.sphere360.app.data;

import android.content.Context;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/** Một lượt chụp: thư mục chứa các tấm ảnh gốc và session.json mô tả góc của từng tấm. */
public final class Session {

    public static final class Frame {
        public final String file;
        public final int target;
        public final float[] rotation; // camera -> toàn cảnh, 9 số theo hàng

        public Frame(String file, int target, float[] rotation) {
            this.file = file;
            this.target = target;
            this.rotation = rotation;
        }
    }

    /** Khung cầu nối: ảnh xám nhỏ chụp lúc lia máy, sau tấm thứ after (xem capture/BridgeRecorder). */
    public static final class Bridge {
        public final String file;
        public final int after;
        public final float[] rotation; // camera -> toàn cảnh, 9 số theo hàng
        public final float blurPx;

        public Bridge(String file, int after, float[] rotation, float blurPx) {
            this.file = file;
            this.after = after;
            this.rotation = rotation;
            this.blurPx = blurPx;
        }
    }

    public final File dir;
    public double focalPx;
    public int imageWidth;
    public int imageHeight;
    public int targetCount;
    public final List<Frame> frames = new ArrayList<>();
    public final List<Bridge> bridges = new ArrayList<>();
    public double bridgeFocalPx;
    public int bridgeRotation = 90; // xoay theo chiều kim đồng hồ bấy nhiêu độ thì khung cầu nối đứng
    public String savedUri;
    public int savedWidth;

    private Session(File dir) {
        this.dir = dir;
    }

    public static File root(Context ctx) {
        File r = new File(ctx.getExternalFilesDir(null), "sessions");
        //noinspection ResultOfMethodCallIgnored
        r.mkdirs();
        return r;
    }

    public static Session create(Context ctx, double focalPx, int imageWidth, int imageHeight, int targetCount) {
        String name = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        File dir = new File(root(ctx), name);
        //noinspection ResultOfMethodCallIgnored
        dir.mkdirs();
        Session s = new Session(dir);
        s.focalPx = focalPx;
        s.imageWidth = imageWidth;
        s.imageHeight = imageHeight;
        s.targetCount = targetCount;
        return s;
    }

    public static List<Session> listAll(Context ctx) {
        List<Session> out = new ArrayList<>();
        File[] dirs = root(ctx).listFiles(File::isDirectory);
        if (dirs == null) return out;
        Arrays.sort(dirs, (a, b) -> b.getName().compareTo(a.getName()));
        for (File d : dirs) {
            try {
                out.add(load(d));
            } catch (IOException | JSONException ignored) {
                // Thư mục hỏng hoặc chưa có ảnh nào.
            }
        }
        return out;
    }

    public static Session load(File dir) throws IOException, JSONException {
        String json = new String(Files.readAllBytes(new File(dir, "session.json").toPath()), StandardCharsets.UTF_8);
        JSONObject o = new JSONObject(json);
        Session s = new Session(dir);
        s.focalPx = o.getDouble("focalPx");
        s.imageWidth = o.getInt("imageWidth");
        s.imageHeight = o.getInt("imageHeight");
        s.targetCount = o.optInt("targetCount", 0);
        s.savedUri = o.optString("savedUri", null);
        s.savedWidth = o.optInt("savedWidth", 0);
        JSONArray arr = o.getJSONArray("frames");
        for (int i = 0; i < arr.length(); i++) {
            JSONObject f = arr.getJSONObject(i);
            JSONArray r = f.getJSONArray("R");
            float[] rot = new float[9];
            for (int k = 0; k < 9; k++) rot[k] = (float) r.getDouble(k);
            s.frames.add(new Frame(f.getString("file"), f.getInt("target"), rot));
        }
        s.bridgeFocalPx = o.optDouble("bridgeFocalPx", 0);
        s.bridgeRotation = o.optInt("bridgeRotation", 90);
        JSONArray br = o.optJSONArray("bridges");
        for (int i = 0; br != null && i < br.length(); i++) {
            JSONObject b = br.getJSONObject(i);
            JSONArray r = b.getJSONArray("R");
            float[] rot = new float[9];
            for (int k = 0; k < 9; k++) rot[k] = (float) r.getDouble(k);
            s.bridges.add(new Bridge(b.getString("file"), b.getInt("after"), rot, (float) b.optDouble("blur", 0)));
        }
        return s;
    }

    public void save() throws IOException {
        try {
            JSONObject o = new JSONObject();
            o.put("focalPx", focalPx);
            o.put("imageWidth", imageWidth);
            o.put("imageHeight", imageHeight);
            o.put("targetCount", targetCount);
            if (savedUri != null) {
                o.put("savedUri", savedUri);
                o.put("savedWidth", savedWidth);
            }
            JSONArray arr = new JSONArray();
            for (Frame f : frames) {
                JSONObject fo = new JSONObject();
                fo.put("file", f.file);
                fo.put("target", f.target);
                JSONArray r = new JSONArray();
                for (float v : f.rotation) r.put((double) v);
                fo.put("R", r);
                arr.put(fo);
            }
            o.put("frames", arr);
            if (!bridges.isEmpty()) {
                o.put("bridgeFocalPx", bridgeFocalPx);
                o.put("bridgeRotation", bridgeRotation);
                JSONArray ba = new JSONArray();
                for (Bridge b : bridges) {
                    JSONObject bo = new JSONObject();
                    bo.put("file", b.file);
                    bo.put("after", b.after);
                    bo.put("blur", (double) b.blurPx);
                    JSONArray r = new JSONArray();
                    for (float v : b.rotation) r.put((double) v);
                    bo.put("R", r);
                    ba.put(bo);
                }
                o.put("bridges", ba);
            }
            File tmp = new File(dir, "session.json.tmp");
            try (FileOutputStream out = new FileOutputStream(tmp)) {
                out.write(o.toString(1).getBytes(StandardCharsets.UTF_8));
            }
            if (!tmp.renameTo(new File(dir, "session.json"))) throw new IOException("Không lưu được session.json");
        } catch (JSONException e) {
            throw new IOException(e);
        }
    }

    public String title() {
        String n = dir.getName();
        if (n.length() == 15) {
            return n.substring(6, 8) + "/" + n.substring(4, 6) + "/" + n.substring(0, 4) + " " +
                    n.substring(9, 11) + ":" + n.substring(11, 13) + ":" + n.substring(13, 15);
        }
        return n;
    }

    public File panoramaFile() {
        return new File(dir, "pano.jpg");
    }

    /** Chiều rộng ảnh cầu khi giữ nguyên độ nét của camera (1 pixel ảnh gốc = 1 pixel ảnh cầu). */
    public int nativeOutputWidth() {
        int w = (int) Math.round(2 * Math.PI * focalPx);
        return w / 16 * 16;
    }

    public void deleteAll() {
        File[] files = dir.listFiles();
        if (files != null) for (File f : files) //noinspection ResultOfMethodCallIgnored
            f.delete();
        //noinspection ResultOfMethodCallIgnored
        dir.delete();
    }
}
