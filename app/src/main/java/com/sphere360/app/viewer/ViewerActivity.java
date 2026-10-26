package com.sphere360.app.viewer;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.ImageDecoder;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.net.Uri;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.util.Log;
import android.util.Size;
import android.view.GestureDetector;
import android.view.Gravity;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.Surface;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.TextView;

import java.io.File;
import java.io.IOException;

import com.sphere360.app.ui.Ui;
import com.sphere360.app.util.Rot;

/** Xem ảnh cầu 360°: kéo để nhìn quanh, chụm hai ngón để phóng to, hoặc xoay điện thoại (con quay). */
public class ViewerActivity extends Activity implements SensorEventListener, PanoRenderer.Listener {

    public static final String EXTRA_PATH = "path";
    private static final String TAG = "Viewer";
    /** Giới hạn độ phân giải nạp lên GPU (8192×4096 ≈ 170 MB kể cả mipmap). */
    private static final int MAX_VIEW_WIDTH = 8192;
    private static final float DEFAULT_FOV = 75;

    private GLSurfaceView glView;
    private PanoRenderer renderer;
    private TextView status;
    private Button gyroButton;

    private SensorManager sensorManager;
    private Sensor rotationSensor;
    private boolean gyro;
    private float[] deviceToWorld;
    // Lọc rung tay cho chế độ con quay (xem OrientationFilter).
    private final OrientationFilter filter = new OrientationFilter();
    // Lấy mẫu cảm biến dày (~200 Hz) để bộ lọc có đủ mẫu; màn hình vẫn chỉ vẽ theo nhịp khung hình.
    private static final int SENSOR_PERIOD_US = 5000;
    private double gyroOffset;

    private double yawDeg;
    private double pitchDeg;
    private float vFovDeg = DEFAULT_FOV;
    private volatile boolean loading;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        float dp = getResources().getDisplayMetrics().density;

        FrameLayout root = new FrameLayout(this);
        root.setBackgroundColor(Color.BLACK);

        renderer = new PanoRenderer(this);
        glView = new GLSurfaceView(this);
        glView.setEGLContextClientVersion(3);
        glView.setPreserveEGLContextOnPause(true);
        glView.setRenderer(renderer);
        glView.setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);
        root.addView(glView, new FrameLayout.LayoutParams(-1, -1));

        status = new TextView(this);
        status.setTextColor(Color.WHITE);
        status.setTextSize(16);
        status.setText("Đang tải ảnh…");
        root.addView(status, new FrameLayout.LayoutParams(-2, -2, Gravity.CENTER));

        LinearLayout bar = new LinearLayout(this);
        bar.setGravity(Gravity.CENTER_VERTICAL);
        bar.setPadding((int) (12 * dp), (int) (8 * dp), (int) (12 * dp), (int) (24 * dp));
        Ui.padForSystemBars(bar, true, false);
        bar.setBackground(new android.graphics.drawable.GradientDrawable(
                android.graphics.drawable.GradientDrawable.Orientation.TOP_BOTTOM, new int[]{0xB3000000, 0x00000000}));
        Ui vu = new Ui(this);
        Button close = vu.overlay("Đóng");
        close.setOnClickListener(v -> finish());
        bar.addView(close);
        TextView title = new TextView(this);
        title.setText("Xem 360°");
        title.setTextColor(Color.WHITE);
        title.setTextSize(18);
        title.setGravity(Gravity.CENTER);
        bar.addView(title, new LinearLayout.LayoutParams(0, -2, 1));
        gyroButton = vu.overlay("");
        gyroButton.setOnClickListener(v -> setGyro(!gyro));
        bar.addView(gyroButton);
        root.addView(bar, new FrameLayout.LayoutParams(-1, -2, Gravity.TOP));

        setContentView(root);

        sensorManager = getSystemService(SensorManager.class);
        rotationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        if (rotationSensor == null) rotationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        if (rotationSensor == null) gyroButton.setVisibility(View.GONE);
        updateGyroButton();
        setupGestures();
        updateView();
    }

    @Override
    protected void onResume() {
        super.onResume();
        glView.onResume();
        if (rotationSensor != null) {
            filter.reset();
            sensorManager.registerListener(this, rotationSensor, SENSOR_PERIOD_US);
        }
    }

    @Override
    protected void onPause() {
        sensorManager.unregisterListener(this);
        glView.onPause();
        super.onPause();
    }

    // ---------------------------------------------------------------- nạp ảnh

    @Override
    public void onMaxTextureSize(int size) {
        // Gọi trên luồng GL mỗi khi có context mới (texture cũ đã mất), nên nạp lại ảnh.
        if (loading) return;
        loading = true;
        int cap = Math.min(MAX_VIEW_WIDTH, size);
        new Thread(() -> load(cap), "viewer-load").start();
    }

    private void load(int cap) {
        try {
            String path = getIntent().getStringExtra(EXTRA_PATH);
            Uri uri = getIntent().getData();
            ImageDecoder.Source src = path != null
                    ? ImageDecoder.createSource(new File(path))
                    : ImageDecoder.createSource(getContentResolver(), uri);
            final Size[] original = new Size[1];
            Bitmap bmp = ImageDecoder.decodeBitmap(src, (decoder, info, s) -> {
                Size sz = info.getSize();
                original[0] = sz;
                int w = sz.getWidth(), h = sz.getHeight();
                double scale = Math.min(1.0, Math.min(cap / (double) w, cap / (double) h));
                decoder.setTargetSize(Math.max(1, (int) (w * scale)), Math.max(1, (int) (h * scale)));
                decoder.setAllocator(ImageDecoder.ALLOCATOR_SOFTWARE);
            });
            Size o = original[0];
            boolean equirect = o != null && Math.abs(o.getWidth() - 2 * o.getHeight()) <= o.getWidth() / 50;
            Log.i(TAG, "loaded " + o + " -> " + bmp.getWidth() + "x" + bmp.getHeight());
            glView.queueEvent(() -> renderer.setBitmap(bmp));
            glView.requestRender();
            runOnUiThread(() -> {
                if (equirect) {
                    status.setVisibility(View.GONE);
                } else {
                    status.setText("Ảnh không có tỉ lệ 2:1 nên không phải ảnh cầu đầy đủ, sẽ bị méo.");
                }
            });
        } catch (IOException | RuntimeException | OutOfMemoryError e) {
            Log.e(TAG, "load", e);
            runOnUiThread(() -> status.setText("Không mở được ảnh: " + e.getMessage()));
        } finally {
            loading = false;
        }
    }

    // ---------------------------------------------------------------- điều khiển

    private void setupGestures() {
        GestureDetector gestures = new GestureDetector(this, new GestureDetector.SimpleOnGestureListener() {
            @Override
            public boolean onDown(MotionEvent e) {
                return true;
            }

            @Override
            public boolean onScroll(MotionEvent e1, MotionEvent e2, float dx, float dy) {
                double degPerPx = vFovDeg / Math.max(1, glView.getHeight());
                if (gyro) {
                    // Hướng xem = hướng máy - offset, nên kéo sang trái (dx > 0) thì giảm offset.
                    gyroOffset -= Math.toRadians(dx * degPerPx);
                } else {
                    yawDeg += dx * degPerPx;
                    pitchDeg = Math.max(-90, Math.min(90, pitchDeg - dy * degPerPx));
                }
                updateView();
                return true;
            }

            @Override
            public boolean onDoubleTap(MotionEvent e) {
                vFovDeg = DEFAULT_FOV;
                if (!gyro) pitchDeg = 0;
                updateView();
                return true;
            }
        });
        ScaleGestureDetector scale = new ScaleGestureDetector(this, new ScaleGestureDetector.SimpleOnScaleGestureListener() {
            @Override
            public boolean onScale(ScaleGestureDetector d) {
                vFovDeg = Math.max(20, Math.min(110, vFovDeg / d.getScaleFactor()));
                updateView();
                return true;
            }
        });
        glView.setOnTouchListener((v, e) -> {
            scale.onTouchEvent(e);
            if (!scale.isInProgress()) gestures.onTouchEvent(e);
            return true;
        });
    }

    private void setGyro(boolean on) {
        if (on == gyro) return;
        if (on) {
            if (deviceToWorld != null) {
                // Giữ nguyên hướng đang xem khi bật con quay.
                float[] f = Rot.apply(deviceToWorld, 0, 0, -1);
                gyroOffset = Math.atan2(f[0], f[1]) - Math.toRadians(yawDeg);
            }
        } else {
            float[] m = currentMatrix();
            yawDeg = Math.toDegrees(Math.atan2(m[2], m[8]));
            pitchDeg = Math.toDegrees(-Math.asin(Math.max(-1, Math.min(1, m[5]))));
        }
        gyro = on;
        updateGyroButton();
        updateView();
    }

    private void updateGyroButton() {
        gyroButton.setText(gyro ? "Con quay: Bật" : "Con quay: Tắt");
    }

    private float[] currentMatrix() {
        if (gyro && deviceToWorld != null) {
            float[] a = Rot.mul(Rot.rotZ(gyroOffset), deviceToWorld);
            return Rot.cameraToPano(a);
        }
        double y = Math.toRadians(yawDeg), p = Math.toRadians(pitchDeg);
        float cy = (float) Math.cos(y), sy = (float) Math.sin(y);
        float cp = (float) Math.cos(p), sp = (float) Math.sin(p);
        float[] ry = {cy, 0, sy, 0, 1, 0, -sy, 0, cy};
        float[] rx = {1, 0, 0, 0, cp, -sp, 0, sp, cp};
        return Rot.mul(ry, rx);
    }

    private void updateView() {
        renderer.setView(currentMatrix(), vFovDeg);
        glView.requestRender();
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        float[] r = new float[9];
        // Phóng to thì rung tay hiện rõ hơn trên màn hình, nên lọc mạnh hơn theo tỉ lệ góc nhìn.
        double zoom = Math.max(0.3, Math.min(1.5, vFovDeg / DEFAULT_FOV));
        SensorManager.getRotationMatrixFromVector(r, filter.filter(event.values, event.timestamp, zoom));
        float[] out = new float[9];
        int rot = getDisplay() != null ? getDisplay().getRotation() : Surface.ROTATION_0;
        switch (rot) {
            case Surface.ROTATION_90:
                SensorManager.remapCoordinateSystem(r, SensorManager.AXIS_Y, SensorManager.AXIS_MINUS_X, out);
                break;
            case Surface.ROTATION_180:
                SensorManager.remapCoordinateSystem(r, SensorManager.AXIS_MINUS_X, SensorManager.AXIS_MINUS_Y, out);
                break;
            case Surface.ROTATION_270:
                SensorManager.remapCoordinateSystem(r, SensorManager.AXIS_MINUS_Y, SensorManager.AXIS_X, out);
                break;
            default:
                out = r;
        }
        deviceToWorld = out;
        if (gyro) updateView();
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}
}
