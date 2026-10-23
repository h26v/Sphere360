package com.sphere360.app.probe;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.graphics.ImageFormat;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.opengl.GLES11Ext;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.util.Log;
import android.util.Size;
import android.view.Surface;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import com.google.ar.core.ArCoreApk;
import com.google.ar.core.Camera;
import com.google.ar.core.CameraConfig;
import com.google.ar.core.CameraConfigFilter;
import com.google.ar.core.Config;
import com.google.ar.core.Frame;
import com.google.ar.core.Pose;
import com.google.ar.core.Session;
import com.google.ar.core.SharedCamera;
import com.google.ar.core.TrackingState;
import com.google.ar.core.exceptions.NotYetAvailableException;

import java.io.File;
import java.io.FileOutputStream;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;
import java.util.Locale;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import com.sphere360.app.ui.Ui;

/**
 * Màn hình thử nghiệm: máy có ARCore và Depth API không, và có chụp được ảnh JPEG độ phân giải đầy đủ
 * trong lúc ARCore đang theo dõi vị trí (dùng chung camera qua SharedCamera) không. Kết quả ghi lên màn
 * hình và logcat (thẻ "ArProbe").
 */
public class ArProbeActivity extends Activity implements GLSurfaceView.Renderer {

    private static final String TAG = "ArProbe";

    private Ui u;
    private TextView log;
    private TextView live;
    private GLSurfaceView gl;
    private Button installButton, captureButton;

    private Session session;
    private SharedCamera shared;
    private String cameraId;
    private CameraDevice device;
    private CameraCaptureSession captureSession;
    private CaptureRequest.Builder previewBuilder;
    private ImageReader jpegReader;
    private List<Size> jpegSizes = new ArrayList<>();
    private int jpegAttempt;
    private HandlerThread thread;
    private Handler handler;

    private volatile boolean arRunning;
    private int textureId = -1;
    private boolean displaySet;
    private int viewW, viewH;

    // Số liệu đo trên luồng GL.
    private volatile TrackingState tracking = TrackingState.STOPPED;
    private volatile float[] lastT = new float[3];
    private volatile int frames, depthFrames, depthW, depthH, lostEvents;
    private volatile long firstTrackingMs, startMs;
    private volatile long captureSubmitMs;
    private volatile TrackingState trackingAtCapture;
    private volatile float[] tAtCapture;
    private int captures;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        u = new Ui(this);
        Ui.paintBars(this);

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setBackgroundColor(Ui.BG);
        root.setPadding(u.px(16), u.px(12), u.px(16), u.px(12));
        Ui.padForSystemBars(root, true, true);

        root.addView(u.bold("Thử nghiệm ARCore", 22, Ui.TEXT));
        root.addView(u.text("Cầm máy dọc, đi chậm và lia máy quanh phòng vài giây để ARCore bắt được vị trí, rồi bấm Chụp 12 MP.",
                13, Ui.TEXT_2), u.lp(-1, -2, 4));

        gl = new GLSurfaceView(this);
        gl.setPreserveEGLContextOnPause(true);
        gl.setEGLContextClientVersion(2);
        gl.setRenderer(this);
        gl.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
        root.addView(gl, u.lp(-1, u.px(2), 8));

        live = u.bold("", 14, Ui.ACCENT);
        root.addView(live, u.lp(-1, -2, 8));

        LinearLayout buttons = u.row();
        installButton = u.secondary("Cài ARCore");
        installButton.setOnClickListener(v -> requestInstall());
        Ui.setEnabled(installButton, false);
        buttons.addView(installButton, u.weight(0));
        captureButton = u.primary("Chụp 12 MP");
        captureButton.setOnClickListener(v -> captureJpeg());
        Ui.setEnabled(captureButton, false);
        buttons.addView(captureButton, u.weight(8));
        root.addView(buttons, u.lp(-1, -2, 10));

        ScrollView scroll = new ScrollView(this);
        log = u.text("", 12, Ui.TEXT);
        log.setTypeface(android.graphics.Typeface.MONOSPACE);
        scroll.addView(log);
        root.addView(scroll, new LinearLayout.LayoutParams(-1, 0, 1));
        setContentView(root);

        live.postDelayed(liveTick, 500);
    }

    private final Runnable liveTick = new Runnable() {
        @Override
        public void run() {
            if (arRunning) {
                float[] t = lastT;
                live.setText(String.format(Locale.US,
                        "%s · %d khung · vị trí (%.2f, %.2f, %.2f) m · độ sâu %d khung %dx%d · mất dấu %d lần",
                        tracking, frames, t[0], t[1], t[2], depthFrames, depthW, depthH, lostEvents));
            }
            live.postDelayed(this, 500);
        }
    };

    private void say(String s) {
        Log.i(TAG, s);
        runOnUiThread(() -> log.append(s + "\n"));
    }

    @Override
    protected void onResume() {
        super.onResume();
        thread = new HandlerThread("arprobe");
        thread.start();
        handler = new Handler(thread.getLooper());
        gl.onResume();
        checkAvailability();
    }

    @Override
    protected void onPause() {
        arRunning = false;
        gl.onPause();
        if (session != null) session.pause();
        if (captureSession != null) {
            captureSession.close();
            captureSession = null;
        }
        if (device != null) {
            device.close();
            device = null;
        }
        if (jpegReader != null) {
            jpegReader.close();
            jpegReader = null;
        }
        thread.quitSafely();
        super.onPause();
    }

    @Override
    protected void onDestroy() {
        if (session != null) session.close();
        super.onDestroy();
    }

    // ---------------------------------------------------------------- 1. ARCore có trên máy không

    private void checkAvailability() {
        ArCoreApk.Availability a = ArCoreApk.getInstance().checkAvailability(this);
        if (a.isTransient()) {
            log.postDelayed(this::checkAvailability, 300);
            return;
        }
        say("ARCore checkAvailability: " + a);
        switch (a) {
            case SUPPORTED_INSTALLED:
                Ui.setEnabled(installButton, false);
                if (session == null) createSession();
                else startCamera();
                break;
            case SUPPORTED_NOT_INSTALLED:
            case SUPPORTED_APK_TOO_OLD:
                say("=> Máy được hỗ trợ nhưng chưa có ARCore bản thật. Bấm \"Cài ARCore\" (mở Play Store).");
                Ui.setEnabled(installButton, true);
                break;
            default:
                say("=> Google không hỗ trợ ARCore trên máy này. Không làm tiếp được.");
                break;
        }
    }

    private void requestInstall() {
        try {
            ArCoreApk.InstallStatus st = ArCoreApk.getInstance().requestInstall(this, true);
            say("requestInstall: " + st);
        } catch (Exception e) {
            say("requestInstall lỗi: " + e);
        }
    }

    // ---------------------------------------------------------------- 2. phiên ARCore dùng chung camera

    private void createSession() {
        try {
            session = new Session(this, EnumSet.of(Session.Feature.SHARED_CAMERA));
        } catch (Exception e) {
            say("Không tạo được phiên ARCore: " + e);
            return;
        }
        boolean depthAuto = session.isDepthModeSupported(Config.DepthMode.AUTOMATIC);
        boolean depthRaw = session.isDepthModeSupported(Config.DepthMode.RAW_DEPTH_ONLY);
        say("Depth API: AUTOMATIC=" + depthAuto + ", RAW_DEPTH_ONLY=" + depthRaw);

        CameraConfigFilter filter = new CameraConfigFilter(session);
        List<CameraConfig> configs = session.getSupportedCameraConfigs(filter);
        say("Cấu hình camera ARCore hỗ trợ: " + configs.size());
        for (CameraConfig c : configs) {
            say(String.format(Locale.US, "  cam %s · ảnh CPU %s · texture %s · %s fps · cảm biến độ sâu %s",
                    c.getCameraId(), c.getImageSize(), c.getTextureSize(), c.getFpsRange(), c.getDepthSensorUsage()));
        }

        Config config = session.getConfig();
        config.setDepthMode(depthAuto ? Config.DepthMode.AUTOMATIC : Config.DepthMode.DISABLED);
        config.setFocusMode(Config.FocusMode.AUTO);
        config.setUpdateMode(Config.UpdateMode.LATEST_CAMERA_IMAGE);
        session.configure(config);

        shared = session.getSharedCamera();
        cameraId = session.getCameraConfig().getCameraId();
        say("ARCore dùng camera " + cameraId + ", ảnh CPU " + session.getCameraConfig().getImageSize()
                + ", texture " + session.getCameraConfig().getTextureSize());
        jpegSizes = jpegSizesOf(cameraId);
        say("Các cỡ JPEG sẽ thử: " + jpegSizes);
        jpegAttempt = 0;
        startCamera();
    }

    private List<Size> jpegSizesOf(String id) {
        List<Size> out = new ArrayList<>();
        try {
            CameraManager cm = getSystemService(CameraManager.class);
            StreamConfigurationMap map = cm.getCameraCharacteristics(id)
                    .get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            Size[] sizes = map == null ? null : map.getOutputSizes(ImageFormat.JPEG);
            if (sizes != null) {
                for (Size s : sizes) {
                    if (Math.abs(s.getWidth() * 3 - s.getHeight() * 4) <= s.getWidth() / 50) out.add(s);
                }
            }
        } catch (CameraAccessException e) {
            say("Không đọc được cỡ JPEG: " + e);
        }
        out.sort((a, b) -> Long.compare((long) b.getWidth() * b.getHeight(), (long) a.getWidth() * a.getHeight()));
        // Thử cỡ lớn nhất, rồi vài cỡ nhỏ dần nếu không ghép chung được với luồng của ARCore.
        return out.size() > 4 ? out.subList(0, 4) : out;
    }

    @SuppressLint("MissingPermission")
    private void startCamera() {
        if (shared == null || jpegSizes.isEmpty()) return;
        if (checkSelfPermission(android.Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            say("Thiếu quyền camera.");
            return;
        }
        Size js = jpegSizes.get(Math.min(jpegAttempt, jpegSizes.size() - 1));
        jpegReader = ImageReader.newInstance(js.getWidth(), js.getHeight(), ImageFormat.JPEG, 2);
        jpegReader.setOnImageAvailableListener(this::onJpeg, handler);
        shared.setAppSurfaces(cameraId, Arrays.asList(jpegReader.getSurface()));
        say("Mở camera, luồng JPEG " + js);
        try {
            CameraManager cm = getSystemService(CameraManager.class);
            cm.openCamera(cameraId, shared.createARDeviceStateCallback(deviceCallback, handler), handler);
        } catch (Exception e) {
            say("openCamera lỗi: " + e);
        }
    }

    private final CameraDevice.StateCallback deviceCallback = new CameraDevice.StateCallback() {
        @Override
        public void onOpened(CameraDevice d) {
            device = d;
            try {
                List<Surface> surfaces = new ArrayList<>(shared.getArCoreSurfaces());
                previewBuilder = d.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);
                for (Surface s : surfaces) previewBuilder.addTarget(s);
                surfaces.add(jpegReader.getSurface());
                //noinspection deprecation
                d.createCaptureSession(surfaces, shared.createARSessionStateCallback(sessionCallback, handler), handler);
            } catch (Exception e) {
                say("createCaptureSession lỗi: " + e);
            }
        }

        @Override
        public void onDisconnected(CameraDevice d) {
            d.close();
            device = null;
        }

        @Override
        public void onError(CameraDevice d, int error) {
            say("Camera lỗi " + error);
            d.close();
            device = null;
        }
    };

    private final CameraCaptureSession.StateCallback sessionCallback = new CameraCaptureSession.StateCallback() {
        @Override
        public void onConfigured(CameraCaptureSession s) {
            captureSession = s;
            say("=> Ghép được luồng ARCore + JPEG " + jpegReader.getWidth() + "x" + jpegReader.getHeight());
            try {
                s.setRepeatingRequest(previewBuilder.build(), null, handler);
                session.resume();
                startMs = SystemClock.uptimeMillis();
                arRunning = true;
                runOnUiThread(() -> Ui.setEnabled(captureButton, true));
                say("ARCore đang chạy. Lia máy chậm quanh phòng…");
            } catch (Exception e) {
                say("Không chạy được ARCore: " + e);
            }
        }

        @Override
        public void onConfigureFailed(CameraCaptureSession s) {
            say("=> KHÔNG ghép được luồng ARCore + JPEG " + jpegReader.getWidth() + "x" + jpegReader.getHeight());
            if (device != null) {
                device.close();
                device = null;
            }
            jpegReader.close();
            jpegAttempt++;
            if (jpegAttempt < jpegSizes.size()) handler.post(ArProbeActivity.this::startCamera);
            else say("Hết cỡ JPEG để thử.");
        }
    };

    // ---------------------------------------------------------------- 3. chụp JPEG lúc ARCore đang chạy

    private void captureJpeg() {
        if (device == null || captureSession == null) return;
        Ui.setEnabled(captureButton, false);
        trackingAtCapture = tracking;
        tAtCapture = lastT.clone();
        handler.post(() -> {
            try {
                CaptureRequest.Builder b = device.createCaptureRequest(CameraDevice.TEMPLATE_STILL_CAPTURE);
                for (Surface s : shared.getArCoreSurfaces()) b.addTarget(s);
                b.addTarget(jpegReader.getSurface());
                b.set(CaptureRequest.JPEG_QUALITY, (byte) 95);
                captureSubmitMs = SystemClock.uptimeMillis();
                captureSession.capture(b.build(), null, handler);
            } catch (Exception e) {
                say("Chụp lỗi: " + e);
                runOnUiThread(() -> Ui.setEnabled(captureButton, true));
            }
        });
    }

    private void onJpeg(ImageReader r) {
        long ms = SystemClock.uptimeMillis() - captureSubmitMs;
        try (Image img = r.acquireNextImage()) {
            if (img == null) return;
            ByteBuffer buf = img.getPlanes()[0].getBuffer();
            byte[] bytes = new byte[buf.remaining()];
            buf.get(bytes);
            File f = new File(getExternalFilesDir(null), String.format(Locale.US, "arprobe_%d.jpg", ++captures));
            try (FileOutputStream out = new FileOutputStream(f)) {
                out.write(bytes);
            }
            say(String.format(Locale.US, "=> Chụp được JPEG %dx%d, %d KB, sau %d ms. ARCore lúc bấm: %s",
                    img.getWidth(), img.getHeight(), bytes.length / 1024, ms, trackingAtCapture));
        } catch (Exception e) {
            say("Lưu JPEG lỗi: " + e);
        }
        // Xem ARCore có còn theo dõi và vị trí có nhảy sau khi chụp không.
        handler.postDelayed(() -> {
            float[] a = tAtCapture, b = lastT;
            double jump = Math.sqrt(sq(b[0] - a[0]) + sq(b[1] - a[1]) + sq(b[2] - a[2]));
            say(String.format(Locale.US, "   1,5 s sau khi chụp: %s, vị trí đổi %.2f m, mất dấu tổng %d lần",
                    tracking, jump, lostEvents));
            runOnUiThread(() -> Ui.setEnabled(captureButton, true));
        }, 1500);
    }

    private static double sq(double v) {
        return v * v;
    }

    // ---------------------------------------------------------------- luồng GL: cập nhật ARCore

    @Override
    public void onSurfaceCreated(GL10 g, EGLConfig c) {
        int[] tex = new int[1];
        GLES20.glGenTextures(1, tex, 0);
        textureId = tex[0];
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, textureId);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);
    }

    @Override
    public void onSurfaceChanged(GL10 g, int w, int h) {
        viewW = w;
        viewH = h;
        displaySet = false;
    }

    @Override
    public void onDrawFrame(GL10 g) {
        GLES20.glClearColor(0, 0, 0, 1);
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT);
        Session s = session;
        if (!arRunning || s == null || textureId < 0) return;
        try {
            if (!displaySet) {
                s.setCameraTextureName(textureId);
                s.setDisplayGeometry(Surface.ROTATION_0, Math.max(1, viewW), Math.max(1, viewH));
                displaySet = true;
            }
            Frame frame = s.update();
            Camera cam = frame.getCamera();
            TrackingState st = cam.getTrackingState();
            if (tracking == TrackingState.TRACKING && st != TrackingState.TRACKING) lostEvents++;
            if (st == TrackingState.TRACKING && firstTrackingMs == 0) {
                firstTrackingMs = SystemClock.uptimeMillis();
                say(String.format(Locale.US, "=> Bắt được vị trí sau %.1f s", (firstTrackingMs - startMs) / 1000.0));
            }
            tracking = st;
            frames++;
            if (st == TrackingState.TRACKING) {
                Pose p = cam.getPose();
                lastT = new float[]{p.tx(), p.ty(), p.tz()};
                try (Image depth = frame.acquireDepthImage16Bits()) {
                    depthFrames++;
                    if (depthW == 0) say("=> Có ảnh độ sâu " + depth.getWidth() + "x" + depth.getHeight());
                    depthW = depth.getWidth();
                    depthH = depth.getHeight();
                } catch (NotYetAvailableException | IllegalStateException ignored) {
                    // Độ sâu chưa sẵn hoặc chưa bật.
                }
            }
        } catch (Exception e) {
            Log.w(TAG, "update", e);
        }
    }
}
