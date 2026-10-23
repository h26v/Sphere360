package com.sphere360.app.capture;

import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.media.Image;
import android.os.SystemClock;
import android.util.Log;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicInteger;

import com.sphere360.app.data.Session;
import com.sphere360.app.util.Rot;

/**
 * Khung cầu nối: trong lúc lia máy giữa hai tấm, giữ ảnh xám nhỏ từ luồng YUV mỗi khi máy xoay thêm STEP_DEG,
 * kèm hướng máy lúc phơi sáng. Giữa hai tấm, vật gần có thể lệch quá tầm optical flow; giữa hai khung liền nhau
 * thì chỉ lệch vài độ, nên bộ ghép dò flow qua chuỗi khung này để biết độ lệch thật.
 */
final class BridgeRecorder {

    interface Listener {
        /** Gọi trên luồng ghi file khi một khung đã lưu xong. */
        void onSaved(Session.Bridge bridge);
    }

    private static final String TAG = "Bridge";
    private static final double STEP_DEG = 2.0;
    // Nhòe tối đa (pixel của khung cầu nối). Khi đang lia, OIS không bù được nên tính cả độ quét. Bộ ghép thu
    // nhỏ khung khoảng 3 lần nên 8 px ở đây còn chưa tới 3 px ở khung flow.
    private static final double MAX_BLUR_PX = 8.0;
    private static final int MAX_FRAMES = 600;
    private static final int MAX_PENDING_WRITES = 3;
    private static final int JPEG_QUALITY = 80;

    private final File dir;
    private final double focalPx;
    private final MotionLog motion;
    private final Listener listener;
    private final ExecutorService writer = Executors.newSingleThreadExecutor();
    private final AtomicInteger pendingWrites = new AtomicInteger();

    private volatile int after = -1;   // chỉ số tấm chụp gần nhất
    private volatile float[] lastR;    // hướng (camera -> toàn cảnh) của khung hoặc tấm giữ gần nhất
    private volatile int kept, blurred;
    private volatile int blurStreak;               // số khung liền nhau bị bỏ vì nhòe
    private volatile long lastKeptMs, lastBlurMs;  // SystemClock.uptimeMillis

    BridgeRecorder(File dir, double focalPx, MotionLog motion, Listener listener) {
        this.dir = dir;
        this.focalPx = focalPx;
        this.motion = motion;
        this.listener = listener;
    }

    /** Đã nhận thêm một tấm: các khung sau thuộc khoảng giữa tấm này và tấm kế tiếp. */
    void onStill(int index, float[] rotation) {
        after = index;
        lastR = rotation;
    }

    int kept() {
        return kept;
    }

    int blurred() {
        return blurred;
    }

    int blurStreak() {
        return blurStreak;
    }

    long lastKeptMs() {
        return lastKeptMs;
    }

    long lastBlurMs() {
        return lastBlurMs;
    }

    /**
     * Xem một khung YUV (gọi trên luồng đọc ảnh). exposureNs, skewNs lấy từ kết quả khung xem trước gần nhất;
     * paused khi đang chụp tấm chính.
     */
    void onImage(Image img, long exposureNs, long skewNs, boolean paused) {
        int a = after;
        float[] last = lastR;
        if (a < 0 || last == null || paused || kept >= MAX_FRAMES || pendingWrites.get() >= MAX_PENDING_WRITES) {
            return;
        }
        long start = img.getTimestamp();
        long end = start + exposureNs + skewNs;
        float[] pose = motion.poseAt((start + end) / 2);
        if (pose == null) return;
        float[] r = Rot.cameraToPano(pose);
        if (Rot.rotationDeg(last, r) < STEP_DEG) return;
        double swept = motion.sweptRad(start, end);
        double blurPx = swept < 0 ? 0 : swept * focalPx;
        if (blurPx > MAX_BLUR_PX) {
            blurred++;
            blurStreak++;
            lastBlurMs = SystemClock.uptimeMillis();
            return;
        }

        // Chỉ lấy kênh sáng; NV21 với màu xám trung tính để YuvImage nén thành JPEG.
        int w = img.getWidth(), h = img.getHeight();
        Image.Plane yPlane = img.getPlanes()[0];
        ByteBuffer buf = yPlane.getBuffer();
        int rowStride = yPlane.getRowStride();
        byte[] nv21 = new byte[w * h * 3 / 2];
        for (int row = 0; row < h; row++) {
            buf.position(row * rowStride);
            buf.get(nv21, row * w, w);
        }
        Arrays.fill(nv21, w * h, nv21.length, (byte) 128);

        lastR = r;
        blurStreak = 0;
        lastKeptMs = SystemClock.uptimeMillis();
        int index = kept++;
        String name = String.format(Locale.US, "bridge_%03d.jpg", index);
        float blur = (float) blurPx;
        pendingWrites.incrementAndGet();
        writer.execute(() -> {
            try (FileOutputStream out = new FileOutputStream(new File(dir, name))) {
                new YuvImage(nv21, ImageFormat.NV21, w, h, null).compressToJpeg(new Rect(0, 0, w, h), JPEG_QUALITY, out);
                listener.onSaved(new Session.Bridge(name, a, r, blur));
            } catch (IOException e) {
                Log.e(TAG, "save " + name, e);
            } finally {
                pendingWrites.decrementAndGet();
            }
        });
    }

    void shutdown() {
        writer.shutdown();
    }
}
