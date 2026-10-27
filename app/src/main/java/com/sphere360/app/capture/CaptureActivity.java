package com.sphere360.app.capture;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.graphics.SurfaceTexture;
import android.graphics.drawable.GradientDrawable;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureFailure;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.media.Image;
import android.media.ImageReader;
import android.media.MediaActionSound;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.util.Log;
import android.view.Gravity;
import android.view.Surface;
import android.view.TextureView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import com.sphere360.app.data.Session;
import com.sphere360.app.data.Settings;
import com.sphere360.app.stitch.StitchActivity;
import com.sphere360.app.ui.Ui;
import com.sphere360.app.util.Rot;

/** Màn hình chụp: hướng dẫn đưa máy tới từng góc và tự chụp khi máy đứng yên đúng chỗ. */
public class CaptureActivity extends Activity implements SensorEventListener {

    private static final String TAG = "Capture";
    private static final double ALIGN_DEG = 4.0;
    private static final double ROLL_DEG = 7.0;
    /** Nhòe dự kiến tối đa (pixel ảnh gốc) để bắt đầu chụp, tính từ tốc độ xoay và tốc trập. */
    private static final double BLUR_TRIGGER_PX = 1.5;
    /** Nhòe đo được trong lúc phơi sáng vượt ngưỡng này thì bỏ tấm và chụp lại. */
    private static final double BLUR_REJECT_PX = 3.0;
    /**
     * Phần nhòe còn lại khi OIS đang chạy. Máy này không xuất dữ liệu OIS theo chuẩn Android nên dùng
     * hệ số cố định: 0,3 tương đương OIS bù được ~1,7 stop, thấp hơn mức 3–4 stop hãng thường công bố.
     */
    private static final double OIS_RESIDUAL = 0.3;
    private static final double MIN_STEADY_DEG_PER_SEC = 1.0;
    private static final double MAX_STEADY_DEG_PER_SEC = 6.0;
    /** Máy phải nằm yên trong ngưỡng liên tục chừng này mới chụp. */
    private static final long SETTLE_MS = 500;
    private static final long REJECT_NOTICE_MS = 2000;
    private static final double START_PITCH_DEG = 20.0;

    private CameraManager cameraManager;
    private CameraSpec spec;
    private CameraDevice camera;
    private CameraCaptureSession captureSession;
    private CaptureRequest.Builder previewBuilder;
    private ImageReader jpegReader;
    private ImageReader bridgeReader;
    private HandlerThread bridgeThread;
    private Handler bridgeHandler;
    private BridgeRecorder bridges;   // null nếu máy không có luồng YUV phù hợp
    private boolean bridgeStream = true;
    private volatile long previewSkewNs;
    // Hai khung cầu nối (hoặc tấm) cách nhau không quá góc này thì bộ ghép dò qua được (kBridgeStepDeg).
    private static final double BRIDGE_LINK_DEG = 6.0;
    // Từng này khung liền nhau bị bỏ vì nhòe thì nhắc lia chậm lại.
    private static final int BRIDGE_BLUR_WARN = 3;
    private float[] lastTrailDir;
    private TextView chipBridge;
    private Surface previewSurface;
    private HandlerThread cameraThread;
    private Handler cameraHandler;

    private TextureView preview;
    private GuideOverlayView overlay;
    private Ui u;
    private TextView stepLabel;
    private TextView instruction;
    private TextView chipAim, chipLevel, chipStill;
    private TextView details;
    private TextView counter;
    private RowProgressView rowProgress;
    private Button stitchButton;

    private SensorManager sensorManager;
    private Sensor rotationSensor;
    private float[] yawAlign;
    private Sensor gyroSensor;
    private double lastBestAngle;
    private double angularSpeed = 30; // độ/giây, đo bằng con quay hồi chuyển, đã làm mượt ~40 ms
    private long lastGyroNs;
    private long holdStart;
    private final MotionLog motion = new MotionLog();
    private volatile long previewExposureNs = 33_333_333L;
    private volatile boolean previewOis; // máy báo OIS đang bật ở khung xem trước
    private long rejectNoticeUntil;
    private double lastRejectPx;

    private List<CapturePlan.Target> targets;
    private Session session;
    private MediaActionSound shutter;

    private boolean resumed;
    private boolean handheld;
    private volatile boolean busy;
    private boolean exposureLocked;
    /** Khoảng lấy nét (diopter) của tấm đầu tiên; null khi chưa khóa nét. */
    private volatile Float lockedFocus;
    private int pendingTarget = -1;
    private float[] pendingPose;      // hướng máy (hệ đã căn hướng) lúc bấm chụp
    private long pendingSubmitNs;
    private String pendingFile;       // ảnh đã ghi xong, chờ kết quả đo rung
    private Exposure pendingExposure; // kết quả đo rung, chờ ảnh ghi xong

    /** Nhòe đo được trong lúc phơi sáng và hướng máy ở giữa lần phơi sáng. */
    private static final class Exposure {
        final double blurPx;
        final float[] pose;

        Exposure(double blurPx, float[] pose) {
            this.blurPx = blurPx;
            this.pose = pose;
        }
    }
    private int frameCounter;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        if (checkSelfPermission(android.Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            finish();
            return;
        }

        cameraManager = getSystemService(CameraManager.class);
        try {
            spec = CameraSpec.pickBackCamera(cameraManager);
        } catch (CameraAccessException | RuntimeException e) {
            Toast.makeText(this, "Không mở được camera: " + e.getMessage(), Toast.LENGTH_LONG).show();
            finish();
            return;
        }
        Log.i(TAG, String.format(Locale.US, "camera %s jpeg %s preview %s focal %.1f fov %.1fx%.1f",
                spec.id, spec.jpegSize, spec.previewSize, spec.focalPx, spec.hFovDeg, spec.vFovDeg));

        android.content.SharedPreferences prefs = getSharedPreferences(Settings.PREFS, MODE_PRIVATE);
        handheld = prefs.getBoolean(Settings.PREF_HANDHELD, false);
        targets = CapturePlan.create(spec.hFovDeg, spec.vFovDeg, handheld,
                prefs.getBoolean(Settings.PREF_CAPTURE_ZENITH, true), prefs.getBoolean(Settings.PREF_CAPTURE_NADIR, true));
        session = Session.create(this, spec.focalPx, spec.uprightWidth(), spec.uprightHeight(), targets.size());
        if (spec.bridgeSize != null) {
            session.bridgeFocalPx = spec.bridgeFocalPx;
            session.bridgeRotation = spec.sensorOrientation;
            bridges = new BridgeRecorder(session.dir, spec.bridgeFocalPx, motion, b -> runOnUiThread(() -> {
                session.bridges.add(b);
                addTrail(b.rotation, false);
            }));
        }

        sensorManager = getSystemService(SensorManager.class);
        rotationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        if (rotationSensor == null) rotationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        gyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        shutter = new MediaActionSound();
        shutter.load(MediaActionSound.SHUTTER_CLICK);

        buildLayout();
        updateCounter();
    }

    private void buildLayout() {
        float dp = getResources().getDisplayMetrics().density;
        int screenW = getResources().getDisplayMetrics().widthPixels;

        FrameLayout root = new FrameLayout(this);
        root.setBackgroundColor(Color.BLACK);

        FrameLayout viewfinder = new FrameLayout(this);
        preview = new TextureView(this);
        overlay = new GuideOverlayView(this);
        overlay.setTargets(targets, spec.hFovDeg);
        viewfinder.addView(preview, new FrameLayout.LayoutParams(-1, -1));
        viewfinder.addView(overlay, new FrameLayout.LayoutParams(-1, -1));
        root.addView(viewfinder, new FrameLayout.LayoutParams(screenW, screenW * 4 / 3, Gravity.CENTER));

        u = new Ui(this);

        // Phần trên: đang ở hàng nào, cần làm gì, và ba điều kiện để chụp.
        LinearLayout top = new LinearLayout(this);
        top.setOrientation(LinearLayout.VERTICAL);
        top.setGravity(Gravity.CENTER_HORIZONTAL);
        top.setBackground(new GradientDrawable(GradientDrawable.Orientation.TOP_BOTTOM,
                new int[]{0xE6000000, 0x99000000, 0x00000000}));
        top.setPadding(u.px(20), u.px(16), u.px(20), u.px(28));
        stepLabel = u.overline("");
        stepLabel.setTextColor(Ui.ACCENT);
        top.addView(stepLabel);
        instruction = u.bold("", 21, Color.WHITE);
        instruction.setGravity(Gravity.CENTER);
        instruction.setShadowLayer(6, 0, 1, Color.BLACK);
        top.addView(instruction, u.lp(-2, -2, 6));
        LinearLayout chips = u.row();
        chipAim = u.chip("Ngắm", Color.WHITE);
        chipLevel = u.chip("Thẳng", Color.WHITE);
        chipStill = u.chip("Đứng yên", Color.WHITE);
        // Sáng xanh khi vừa giữ được khung cầu nối, cam khi khung bị bỏ vì lia nhanh quá.
        chipBridge = u.chip("Nối", Ui.TEXT_2);
        chipBridge.setVisibility(bridges != null ? View.VISIBLE : View.GONE);
        for (TextView t : new TextView[]{chipAim, chipLevel, chipStill, chipBridge}) {
            LinearLayout.LayoutParams l = new LinearLayout.LayoutParams(-2, -2);
            l.leftMargin = u.px(3);
            l.rightMargin = u.px(3);
            chips.addView(t, l);
        }
        top.addView(chips, u.lp(-2, -2, 12));
        details = u.text("", 12, Ui.TEXT_2);
        details.setGravity(Gravity.CENTER);
        details.setVisibility(View.GONE);
        top.addView(details, u.lp(-2, -2, 8));
        // Chạm vào phần trên để hiện số liệu chi tiết (lệch, nghiêng, rung, tốc trập).
        top.setOnClickListener(v -> details.setVisibility(details.getVisibility() == View.VISIBLE ? View.GONE : View.VISIBLE));
        Ui.padForSystemBars(top, true, false);
        root.addView(top, new FrameLayout.LayoutParams(-1, -2, Gravity.TOP));

        // Phần dưới: bản đồ các tấm, đếm số tấm, nút thoát và nút ghép.
        LinearLayout bottom = new LinearLayout(this);
        bottom.setOrientation(LinearLayout.VERTICAL);
        bottom.setBackground(new GradientDrawable(GradientDrawable.Orientation.BOTTOM_TOP,
                new int[]{0xF0000000, 0xA6000000, 0x00000000}));
        bottom.setPadding(u.px(16), u.px(28), u.px(16), u.px(16));
        Ui.padForSystemBars(bottom, false, true);
        rowProgress = new RowProgressView(this);
        rowProgress.setTargets(targets);
        bottom.addView(rowProgress, new LinearLayout.LayoutParams(-1, -2));

        LinearLayout bar = u.row();
        Button cancel = u.overlay("Thoát");
        cancel.setOnClickListener(v -> confirmExit());
        bar.addView(cancel);
        counter = u.bold("", 16, Color.WHITE);
        counter.setGravity(Gravity.CENTER);
        bar.addView(counter, new LinearLayout.LayoutParams(0, -2, 1));
        stitchButton = u.primary("Ghép");
        stitchButton.setOnClickListener(v -> openStitcher());
        bar.addView(stitchButton);
        bottom.addView(bar, u.lp(-1, -2, 14));
        root.addView(bottom, new FrameLayout.LayoutParams(-1, -2, Gravity.BOTTOM));
        setContentView(root);

        preview.setSurfaceTextureListener(new TextureView.SurfaceTextureListener() {
            @Override
            public void onSurfaceTextureAvailable(SurfaceTexture st, int w, int h) {
                if (resumed) openCamera();
            }

            @Override
            public void onSurfaceTextureSizeChanged(SurfaceTexture st, int w, int h) {}

            @Override
            public boolean onSurfaceTextureDestroyed(SurfaceTexture st) {
                return true;
            }

            @Override
            public void onSurfaceTextureUpdated(SurfaceTexture st) {}
        });
    }

    @Override
    protected void onResume() {
        super.onResume();
        resumed = true;
        cameraThread = new HandlerThread("camera");
        cameraThread.start();
        cameraHandler = new Handler(cameraThread.getLooper());
        bridgeThread = new HandlerThread("bridge");
        bridgeThread.start();
        bridgeHandler = new Handler(bridgeThread.getLooper());
        if (rotationSensor != null) {
            sensorManager.registerListener(this, rotationSensor, SensorManager.SENSOR_DELAY_GAME);
        }
        if (gyroSensor != null) {
            sensorManager.registerListener(this, gyroSensor, SensorManager.SENSOR_DELAY_FASTEST);
        }
        if (preview.isAvailable()) openCamera();
    }

    @Override
    protected void onPause() {
        resumed = false;
        sensorManager.unregisterListener(this);
        closeCamera();
        if (cameraThread != null) {
            cameraThread.quitSafely();
            try {
                cameraThread.join();
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }
            cameraThread = null;
            cameraHandler = null;
        }
        if (bridgeThread != null) {
            bridgeThread.quitSafely();
            bridgeThread = null;
            bridgeHandler = null;
        }
        busy = false;
        clearPending();
        holdStart = 0;
        super.onPause();
    }

    @Override
    protected void onDestroy() {
        if (shutter != null) shutter.release();
        if (bridges != null) bridges.shutdown();
        if (session != null && session.frames.isEmpty()) session.deleteAll();
        super.onDestroy();
    }

    // ---------------------------------------------------------------- camera

    @SuppressLint("MissingPermission")
    private void openCamera() {
        if (camera != null || cameraHandler == null) return;
        try {
            cameraManager.openCamera(spec.id, new CameraDevice.StateCallback() {
                @Override
                public void onOpened(CameraDevice c) {
                    camera = c;
                    createSession();
                }

                @Override
                public void onDisconnected(CameraDevice c) {
                    c.close();
                    camera = null;
                }

                @Override
                public void onError(CameraDevice c, int error) {
                    c.close();
                    camera = null;
                    runOnUiThread(() -> {
                        Toast.makeText(CaptureActivity.this, "Lỗi camera " + error, Toast.LENGTH_LONG).show();
                        finish();
                    });
                }
            }, cameraHandler);
        } catch (CameraAccessException e) {
            Toast.makeText(this, "Không mở được camera", Toast.LENGTH_LONG).show();
            finish();
        }
    }

    private void createSession() {
        try {
            SurfaceTexture st = preview.getSurfaceTexture();
            st.setDefaultBufferSize(spec.previewSize.getWidth(), spec.previewSize.getHeight());
            previewSurface = new Surface(st);
            jpegReader = ImageReader.newInstance(spec.jpegSize.getWidth(), spec.jpegSize.getHeight(), ImageFormat.JPEG, 2);
            jpegReader.setOnImageAvailableListener(this::onJpegAvailable, cameraHandler);

            List<Surface> outputs = new ArrayList<>(Arrays.asList(previewSurface, jpegReader.getSurface()));
            if (bridges != null && bridgeStream && bridgeHandler != null) {
                bridgeReader = ImageReader.newInstance(spec.bridgeSize.getWidth(), spec.bridgeSize.getHeight(),
                        ImageFormat.YUV_420_888, 3);
                bridgeReader.setOnImageAvailableListener(this::onBridgeImage, bridgeHandler);
                outputs.add(bridgeReader.getSurface());
            }

            //noinspection deprecation
            camera.createCaptureSession(outputs,
                    new CameraCaptureSession.StateCallback() {
                        @Override
                        public void onConfigured(CameraCaptureSession s) {
                            captureSession = s;
                            try {
                                previewBuilder = camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                                previewBuilder.addTarget(previewSurface);
                                if (bridgeReader != null) previewBuilder.addTarget(bridgeReader.getSurface());
                                applyLocks(previewBuilder);
                                s.setRepeatingRequest(previewBuilder.build(), previewCallback, cameraHandler);
                            } catch (CameraAccessException | IllegalStateException e) {
                                Log.e(TAG, "preview", e);
                            }
                        }

                        @Override
                        public void onConfigureFailed(CameraCaptureSession s) {
                            if (bridgeReader != null) {
                                // Máy không chạy được thêm luồng YUV: chụp như cũ, không có khung cầu nối.
                                Log.w(TAG, "không chạy được luồng khung cầu nối " + spec.bridgeSize);
                                bridgeStream = false;
                                bridgeReader.close();
                                bridgeReader = null;
                                createSession();
                                return;
                            }
                            runOnUiThread(() -> Toast.makeText(CaptureActivity.this,
                                    "Không cấu hình được camera", Toast.LENGTH_LONG).show());
                        }
                    }, cameraHandler);
        } catch (CameraAccessException | IllegalStateException e) {
            Log.e(TAG, "createSession", e);
        }
    }

    private void applyLocks(CaptureRequest.Builder b) {
        // Khóa sáng và cân bằng trắng sau tấm đầu tiên để các tấm cùng tông màu.
        b.set(CaptureRequest.CONTROL_AE_LOCK, exposureLocked);
        b.set(CaptureRequest.CONTROL_AWB_LOCK, exposureLocked);
        if (spec.ois) {
            b.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_ON);
        }
        // Khóa nét theo tấm đầu tiên để tiêu cự (focus breathing) và độ nét không đổi giữa các tấm.
        Float focus = lockedFocus;
        if (focus != null) {
            b.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
            b.set(CaptureRequest.LENS_FOCUS_DISTANCE, focus);
        } else {
            b.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);
        }
    }

    private void closeCamera() {
        if (captureSession != null) {
            captureSession.close();
            captureSession = null;
        }
        if (camera != null) {
            camera.close();
            camera = null;
        }
        if (jpegReader != null) {
            jpegReader.close();
            jpegReader = null;
        }
        if (bridgeReader != null) {
            bridgeReader.close();
            bridgeReader = null;
        }
    }

    private void onBridgeImage(ImageReader reader) {
        try (Image img = reader.acquireLatestImage()) {
            if (img == null || bridges == null) return;
            bridges.onImage(img, previewExposureNs, previewSkewNs, busy);
        } catch (IllegalStateException e) {
            Log.w(TAG, "bridge", e);
        }
    }

    /** Theo dõi tốc trập của khung xem trước để tính ngưỡng rung. */
    private final CameraCaptureSession.CaptureCallback previewCallback = new CameraCaptureSession.CaptureCallback() {
        @Override
        public void onCaptureCompleted(CameraCaptureSession s, CaptureRequest request, TotalCaptureResult result) {
            Long exp = result.get(CaptureResult.SENSOR_EXPOSURE_TIME);
            if (exp != null && exp > 0) previewExposureNs = exp;
            Long skew = result.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW);
            if (skew != null && skew > 0) previewSkewNs = skew;
            previewOis = oisActive(result);
        }
    };

    /** Đo rung trong đúng lúc phơi sáng; với tấm đầu tiên thì khóa nét theo tấm đó. */
    private final CameraCaptureSession.CaptureCallback stillCallback = new CameraCaptureSession.CaptureCallback() {
        @Override
        public void onCaptureCompleted(CameraCaptureSession s, CaptureRequest request, TotalCaptureResult result) {
            long completeNs = SystemClock.elapsedRealtimeNanos();
            Long ts = result.get(CaptureResult.SENSOR_TIMESTAMP);
            Long exp = result.get(CaptureResult.SENSOR_EXPOSURE_TIME);
            Long skew = result.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW);
            boolean ois = oisActive(result);
            runOnUiThread(() -> onExposureMeasured(ts, exp, skew, completeNs, ois));
            lockFocusFrom(result);
        }

        @Override
        public void onCaptureFailed(CameraCaptureSession s, CaptureRequest request, CaptureFailure failure) {
            runOnUiThread(() -> {
                clearPending();
                busy = false;
                holdStart = 0;
            });
        }
    };

    private static boolean oisActive(TotalCaptureResult result) {
        Integer mode = result.get(CaptureResult.LENS_OPTICAL_STABILIZATION_MODE);
        return mode != null && mode == CaptureResult.LENS_OPTICAL_STABILIZATION_MODE_ON;
    }

    /** Ghi lại khoảng lấy nét của tấm đầu tiên rồi chuyển xem trước sang nét cố định. */
    private void lockFocusFrom(TotalCaptureResult result) {
        if (lockedFocus != null || !spec.manualFocus) return;
        Float d = result.get(CaptureResult.LENS_FOCUS_DISTANCE);
        if (d == null) return;
        lockedFocus = d;
        Log.i(TAG, "Khóa nét ở " + d + " diopter");
        try {
            if (captureSession != null && previewBuilder != null) {
                applyLocks(previewBuilder);
                captureSession.setRepeatingRequest(previewBuilder.build(), previewCallback, cameraHandler);
            }
        } catch (CameraAccessException | IllegalStateException e) {
            Log.e(TAG, "focus lock", e);
        }
    }

    private void takePicture(int targetIndex, float[] deviceToWorld) {
        busy = true;
        clearPending();
        pendingTarget = targetIndex;
        pendingPose = deviceToWorld;
        pendingSubmitNs = SystemClock.elapsedRealtimeNanos();
        Handler h = cameraHandler;
        if (h == null) {
            busy = false;
            return;
        }
        h.post(() -> {
            try {
                if (camera == null || captureSession == null) {
                    busy = false;
                    return;
                }
                CaptureRequest.Builder b = camera.createCaptureRequest(CameraDevice.TEMPLATE_STILL_CAPTURE);
                b.addTarget(jpegReader.getSurface());
                b.addTarget(previewSurface);
                b.set(CaptureRequest.JPEG_ORIENTATION, spec.sensorOrientation);
                b.set(CaptureRequest.JPEG_QUALITY, (byte) 95);
                applyLocks(b);
                captureSession.capture(b.build(), stillCallback, cameraHandler);
            } catch (CameraAccessException | IllegalStateException e) {
                Log.e(TAG, "capture", e);
                busy = false;
            }
        });
    }

    private void onJpegAvailable(ImageReader reader) {
        String name = String.format(Locale.US, "frame_%02d.jpg", frameCounter++);
        File file = new File(session.dir, name);
        try (Image img = reader.acquireNextImage()) {
            if (img == null) return;
            ByteBuffer buf = img.getPlanes()[0].getBuffer();
            byte[] bytes = new byte[buf.remaining()];
            buf.get(bytes);
            try (FileOutputStream out = new FileOutputStream(file)) {
                out.write(bytes);
            }
        } catch (IOException e) {
            Log.e(TAG, "save", e);
            runOnUiThread(() -> {
                Toast.makeText(this, "Không lưu được ảnh", Toast.LENGTH_SHORT).show();
                busy = false;
            });
            return;
        }
        runOnUiThread(() -> onFrameSaved(name));
    }

    private void onFrameSaved(String name) {
        if (pendingTarget < 0) {
            new File(session.dir, name).delete();
            busy = false;
            return;
        }
        pendingFile = name;
        finishCapture();
    }

    /** Chạy trên luồng giao diện khi biết thời điểm phơi sáng của tấm vừa chụp. */
    private void onExposureMeasured(Long ts, Long exp, Long skew, long completeNs, boolean ois) {
        if (pendingTarget < 0) return;
        long start, end;
        if (spec.realtimeTimestamps && ts != null) {
            start = ts;
            end = ts + (exp != null ? exp : previewExposureNs) + (skew != null ? skew : 0);
        } else {
            // Không so được đồng hồ: lấy cả khoảng từ lúc gửi lệnh tới lúc chụp xong (khắt khe hơn).
            start = pendingSubmitNs;
            end = completeNs;
        }
        double swept = motion.sweptRad(start, end);
        double rawPx = swept < 0 ? 0 : swept * spec.focalPx;
        double blurPx = rawPx * (ois ? OIS_RESIDUAL : 1);
        float[] pose = motion.poseAt((start + end) / 2);
        pendingExposure = new Exposure(blurPx, pose != null ? pose : pendingPose);
        Log.i(TAG, String.format(Locale.US, "tấm %d: phơi sáng %.1f ms, máy quét %.2f px, OIS %s, nhòe còn %.2f px",
                pendingTarget, (end - start) / 1e6, rawPx, ois ? "bật" : "tắt", blurPx));
        finishCapture();
    }

    /** Khi đã có cả ảnh và kết quả đo rung: nhận tấm, hoặc bỏ đi để chụp lại nếu bị nhòe. */
    private void finishCapture() {
        if (pendingFile == null || pendingExposure == null) return;
        String name = pendingFile;
        Exposure e = pendingExposure;
        int target = pendingTarget;
        clearPending();
        holdStart = 0;
        busy = false;

        if (e.blurPx > BLUR_REJECT_PX) {
            new File(session.dir, name).delete();
            lastRejectPx = e.blurPx;
            rejectNoticeUntil = SystemClock.uptimeMillis() + REJECT_NOTICE_MS;
            return;
        }

        shutter.play(MediaActionSound.SHUTTER_CLICK);
        targets.get(target).done = true;
        float[] rotation = Rot.cameraToPano(e.pose);
        session.frames.add(new Session.Frame(name, target, rotation));
        if (bridges != null) {
            bridges.onStill(session.frames.size() - 1, rotation);
            addTrail(rotation, true);
        }
        try {
            session.save();
        } catch (IOException ex) {
            Log.e(TAG, "session", ex);
        }

        if (!exposureLocked) {
            exposureLocked = true;
            Handler h = cameraHandler;
            if (h != null) h.post(() -> {
                try {
                    if (captureSession != null && previewBuilder != null) {
                        applyLocks(previewBuilder);
                        captureSession.setRepeatingRequest(previewBuilder.build(), previewCallback, cameraHandler);
                    }
                } catch (CameraAccessException | IllegalStateException ex) {
                    Log.e(TAG, "lock", ex);
                }
            });
        }
        updateCounter();
        if (nextTarget() < 0) openStitcher();
    }

    // ---------------------------------------------------------------- guidance

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            float x = event.values[0], y = event.values[1], z = event.values[2];
            double w = Math.sqrt(x * x + y * y + z * z);
            motion.addGyro(event.timestamp, w);
            // Làm mượt theo thời gian (~40 ms) vì tần số mẫu khác nhau giữa các máy.
            double dt = lastGyroNs == 0 ? 0.04 : Math.min(0.2, (event.timestamp - lastGyroNs) * 1e-9);
            lastGyroNs = event.timestamp;
            angularSpeed += (Math.toDegrees(w) - angularSpeed) * (1 - Math.exp(-dt / 0.04));
            // Một cú rung vượt ngưỡng là phải đếm lại thời gian giữ yên.
            if (!busy && angularSpeed >= steadyLimit()) holdStart = 0;
            return;
        }
        float[] r = new float[9];
        SensorManager.getRotationMatrixFromVector(r, event.values);
        if (yawAlign == null) {
            // Chỉ bắt đầu khi người dùng cầm máy dọc, nhìn ngang về phía trước.
            float[] fwd = Rot.apply(r, 0, 0, -1);
            float[] rgt = Rot.apply(r, 1, 0, 0);
            double pitch = Math.toDegrees(Math.asin(Math.max(-1, Math.min(1, fwd[2]))));
            double roll = Math.toDegrees(Math.asin(Math.max(-1, Math.min(1, rgt[2]))));
            if (Math.abs(pitch) > START_PITCH_DEG || Math.abs(roll) > START_PITCH_DEG) {
                stepLabel.setText(handheld ? "BẮT ĐẦU · CẦM TAY" : "BẮT ĐẦU");
                instruction.setText(handheld
                        ? "Cầm máy dọc ngang tầm mắt, kẹp khuỷu tay sát người"
                        : "Cầm máy dọc, hướng ra phía trước ngang tầm mắt");
                details.setText(handheld ? "Khi xoay, xoay cả người và bước chân quanh chỗ đặt máy." : "");
                overlay.update(null, -1, 0);
                return;
            }
            yawAlign = Rot.rotZ(Math.atan2(fwd[0], fwd[1]));
        }
        float[] a = Rot.mul(yawAlign, r);
        motion.addPose(event.timestamp, a);
        float[] forward = Rot.apply(a, 0, 0, -1);
        float[] right = Rot.apply(a, 1, 0, 0);


        int best = -1;
        double bestAngle = Double.MAX_VALUE;
        // Tấm đầu tiên luôn là tấm giữa hàng ngang để làm mốc; sau đó chụp theo thứ tự nào cũng được.
        int limit = session.frames.isEmpty() ? 1 : targets.size();
        for (int i = 0; i < limit; i++) {
            CapturePlan.Target t = targets.get(i);
            if (t.done) continue;
            double ang = Rot.angleDeg(forward, t.dir);
            if (ang < bestAngle) {
                bestAngle = ang;
                best = i;
            }
        }
        int next = nextTarget();
        if (best < 0) {
            overlay.update(a, -1, 0);
            return;
        }

        CapturePlan.Target bt = targets.get(best);
        boolean pole = bt.row >= 3;
        double roll = Math.toDegrees(Math.asin(Math.max(-1, Math.min(1, right[2]))));
        boolean rollOk = pole || Math.abs(roll) < ROLL_DEG;
        boolean aligned = bestAngle < ALIGN_DEG && rollOk;
        double speedLimit = steadyLimit();
        boolean steady = angularSpeed < speedLimit;

        float hold = 0;
        if (aligned && steady && !busy && camera != null) {
            long now = SystemClock.uptimeMillis();
            if (holdStart == 0) holdStart = now;
            hold = Math.min(1f, (now - holdStart) / (float) SETTLE_MS);
            if (hold >= 1f) takePicture(best, a);
        } else if (!busy) {
            holdStart = 0;
        }

        lastBestAngle = bestAngle;
        int highlight = bestAngle < 15 ? best : next;
        overlay.update(a, highlight, hold);
        rowProgress.setHighlight(highlight);
        overlay.setStability((float) (angularSpeed / speedLimit), aligned && steady);
        updateHint(targets.get(highlight), bestAngle < 15, aligned, rollOk, steady, roll, speedLimit, hold);
    }

    private void updateHint(CapturePlan.Target t, boolean near, boolean aligned, boolean rollOk, boolean steady,
                            double roll, double limit, float hold) {
        String text;
        if (busy) {
            text = "Đang chụp… giữ yên";
        } else if (SystemClock.uptimeMillis() < rejectNoticeUntil) {
            text = "Tấm vừa rồi bị rung, giữ yên để chụp lại";
        } else if (!near && panningTooFast()) {
            text = "Lia chậm lại để giữ khung nối";
        } else if (near && !rollOk) {
            // roll > 0: cạnh phải đang cao hơn cạnh trái.
            text = roll > 0 ? "Hạ cạnh phải xuống cho thẳng" : "Hạ cạnh trái xuống cho thẳng";
        } else if (near && !aligned) {
            text = "Đưa vòng tròn vào chấm vàng";
        } else if (near && !steady) {
            text = "Dừng lại, giữ yên máy";
        } else if (near) {
            text = String.format(Locale.US, "Giữ yên… %.1f s", (1 - hold) * SETTLE_MS / 1000.0);
        } else if (t.row == 3) {
            text = "Ngửa máy lên chụp đỉnh trời";
        } else if (t.row == 4) {
            text = "Úp máy xuống chụp mặt đất";
        } else {
            text = "Xoay theo mũi tên tới chấm vàng";
        }
        instruction.setText(text);

        int rowDone = 0, rowTotal = 0;
        for (CapturePlan.Target x : targets) {
            if (x.row != t.row) continue;
            rowTotal++;
            if (x.done) rowDone++;
        }
        stepLabel.setText(rowTotal > 1
                ? String.format(Locale.US, "%s · %d/%d", CapturePlan.ROW_NAMES[t.row], rowDone, rowTotal)
                : CapturePlan.ROW_NAMES[t.row]);

        boolean aimOk = near && lastBestAngle < ALIGN_DEG;
        paintChip(chipAim, aimOk, near);
        paintChip(chipLevel, rollOk, near);
        paintChip(chipStill, steady, true);
        // OIS gộp vào chip đứng yên cho hàng chip vừa màn hình.
        chipStill.setText(previewOis ? "Đứng yên · OIS" : "Đứng yên");
        paintBridgeChip();

        String light = limit <= MIN_STEADY_DEG_PER_SEC ? "\nThiếu sáng: nên tựa máy vào vật cố định" : "";
        String bridgeInfo = bridges == null ? "" : bridgeReader == null && !bridgeStream ? " · không có khung nối"
                : String.format(Locale.US, " · khung nối %d (nhòe bỏ %d)", bridges.kept(), bridges.blurred());
        details.setText(String.format(Locale.US, "lệch %.1f° · nghiêng %.1f° · rung %.1f/%.1f°/s · 1/%d s%s%s",
                lastBestAngle, roll, angularSpeed, limit, Math.round(1e9 / previewExposureNs), light, bridgeInfo));
    }

    /** Xanh khi đạt, trắng mờ khi chưa; relevant = false thì chưa xét (đang ở xa mục tiêu). */
    private void paintChip(TextView chip, boolean ok, boolean relevant) {
        int color = !relevant ? 0x80FFFFFF : ok ? Ui.GOOD : Color.WHITE;
        chip.setTextColor(color);
        chip.setBackground(u.round(ok && relevant ? 0x334CAF50 : 0x33FFFFFF, 10));
    }

    /** Mấy khung cầu nối gần nhất đều bị bỏ vì nhòe (lia nhanh quá), và mới xảy ra. */
    private boolean panningTooFast() {
        return bridges != null && bridges.blurStreak() >= BRIDGE_BLUR_WARN
                && SystemClock.uptimeMillis() - bridges.lastBlurMs() < 1000;
    }

    private void paintBridgeChip() {
        if (bridges == null) return;
        if (bridgeReader == null && !bridgeStream) {
            chipBridge.setVisibility(View.GONE);
            return;
        }
        long now = SystemClock.uptimeMillis();
        boolean fresh = now - bridges.lastKeptMs() < 500;
        boolean blurry = bridges.blurStreak() > 0 && now - bridges.lastBlurMs() < 800;
        int color = blurry ? Ui.WARN : fresh ? Ui.GOOD : 0x80FFFFFF;
        int bg = blurry ? 0x33FFB74D : fresh ? 0x334CAF50 : 0x33FFFFFF;
        chipBridge.setTextColor(color);
        chipBridge.setBackground(u.round(bg, 10));
    }

    /** Thêm một điểm vào vệt đường lia; rotation: camera -> toàn cảnh. anchor: tấm chính. */
    private void addTrail(float[] rotation, boolean anchor) {
        // Trục nhìn của camera trong hệ toàn cảnh là cột 3; đổi về hệ thế giới (x Đông, y hướng đầu, z lên).
        float[] dir = {rotation[2], rotation[8], -rotation[5]};
        boolean linked = lastTrailDir != null && Rot.angleDeg(lastTrailDir, dir) <= BRIDGE_LINK_DEG;
        lastTrailDir = dir;
        double yaw = Math.toDegrees(Math.atan2(dir[0], dir[1]));
        double pitch = Math.toDegrees(Math.asin(Math.max(-1, Math.min(1, dir[2]))));
        rowProgress.addTrailPoint(yaw, pitch, linked, anchor);
    }

    private void confirmExit() {
        if (session.frames.isEmpty()) {
            finish();
            return;
        }
        new AlertDialog.Builder(this)
                .setTitle("Dừng chụp?")
                .setMessage(String.format(Locale.US,
                        "Đã chụp %d/%d tấm. Các tấm này được giữ lại, bạn có thể ghép sau ở trang chính.",
                        session.frames.size(), targets.size()))
                .setPositiveButton("Dừng", (d, w) -> finish())
                .setNegativeButton("Chụp tiếp", null)
                .show();
    }

    @Override
    public void onBackPressed() {
        confirmExit();
    }

    private void clearPending() {
        pendingTarget = -1;
        pendingPose = null;
        pendingFile = null;
        pendingExposure = null;
    }

    /** Tốc độ xoay tối đa (độ/giây) để nhòe không quá BLUR_TRIGGER_PX với tốc trập hiện tại. */
    private double steadyLimit() {
        double exposureSec = previewExposureNs * 1e-9;
        double residual = previewOis ? OIS_RESIDUAL : 1;
        double limit = Math.toDegrees(BLUR_TRIGGER_PX / (spec.focalPx * exposureSec * residual));
        return Math.max(MIN_STEADY_DEG_PER_SEC, Math.min(MAX_STEADY_DEG_PER_SEC, limit));
    }

    private int nextTarget() {
        for (int i = 0; i < targets.size(); i++) if (!targets.get(i).done) return i;
        return -1;
    }

    private void updateCounter() {
        int done = session.frames.size();
        counter.setText(String.format(Locale.US, "%d / %d tấm", done, targets.size()));
        Ui.setEnabled(stitchButton, done >= 2);
        rowProgress.invalidate();
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}

    private void openStitcher() {
        if (session.frames.size() < 2) return;
        Intent i = new Intent(this, StitchActivity.class);
        i.putExtra(StitchActivity.EXTRA_SESSION_DIR, session.dir.getAbsolutePath());
        startActivity(i);
        finish();
    }
}
