package com.sphere360.app.stitch;

import android.app.Activity;
import android.content.ContentResolver;
import android.content.ContentValues;
import android.content.Intent;
import android.content.res.ColorStateList;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.provider.MediaStore;
import android.util.Log;
import android.view.Gravity;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.ScrollView;
import android.widget.TextView;

import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.util.Locale;

import com.sphere360.app.data.Session;
import com.sphere360.app.data.Settings;
import com.sphere360.app.home.MainActivity;
import com.sphere360.app.ui.Ui;
import com.sphere360.app.viewer.ViewerActivity;

/** Ghép ảnh cầu trong nền, lưu vào Thư viện ảnh (Pictures/Sphere360) kèm metadata 360. */
public class StitchActivity extends Activity {

    public static final String EXTRA_SESSION_DIR = "session_dir";
    private static final String TAG = "Stitch";

    /** Việc ghép chạy độc lập với vòng đời Activity để không bị mất khi xoay/thoát màn hình. */
    private static final class Job {
        final String dir;
        volatile int percent;
        volatile String stage = "Đang chuẩn bị";
        volatile String error;
        volatile boolean finished;
        volatile Uri savedUri;
        volatile int width;
        volatile long elapsedMs;
        final long startedAt = System.currentTimeMillis();

        Job(String dir) {
            this.dir = dir;
        }
    }

    private static volatile Job current;

    /** Các bước của bộ ghép, kèm % mà bộ ghép báo khi bắt đầu mỗi bước. */
    private static final int[] STEP_AT = {0, 22, 30, 38, 45, 50, 58, 96, 100};
    private static final String[] STEP_NAMES = {"Tìm điểm đặc trưng", "So khớp các ảnh", "Căn chỉnh góc",
            "Xử lý thị sai", "Cân bằng sáng", "Tìm đường nối", "Dựng ảnh cầu", "Ghi file ảnh",
            "Lưu vào Thư viện ảnh"};

    private final Handler ui = new Handler(Looper.getMainLooper());
    private Ui u;
    private Job job;
    private TextView title;
    private TextView subtitle;
    private TextView percentText;
    private ProgressBar progress;
    private LinearLayout stepsCard;
    private TextView[] stepIcons;
    private TextView[] stepLabels;
    private LinearLayout resultCard;
    private ImageView thumbnail;
    private TextView resultText;
    private LinearLayout actions;
    private boolean shownResult;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        u = new Ui(this);
        Ui.paintBars(this);
        String dir = getIntent().getStringExtra(EXTRA_SESSION_DIR);
        buildLayout();

        Job running = current;
        boolean sameDir = running != null && running.dir.equals(dir);
        // Chỉ dùng lại việc cũ khi nó đang chạy dở, hoặc khi màn hình được dựng lại (xoay, quay về app).
        // Bấm "Ghép lại" luôn ghép mới.
        if (sameDir && (!running.finished || savedInstanceState != null)) {
            job = running;
        } else if (running != null && !running.finished) {
            title.setText("Đang bận");
            subtitle.setText("App đang ghép một lượt khác. Chờ xong rồi thử lại nhé.");
            stepsCard.setVisibility(View.GONE);
            showActions(false, false);
            return;
        } else {
            job = start(dir);
        }
        ui.post(poll);
    }

    @Override
    protected void onDestroy() {
        ui.removeCallbacksAndMessages(null);
        super.onDestroy();
    }

    private void buildLayout() {
        ScrollView scroll = new ScrollView(this);
        scroll.setBackgroundColor(Ui.BG);
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setPadding(u.px(20), u.px(16), u.px(20), u.px(24));
        scroll.addView(root);
        Ui.padForSystemBars(root, true, true);

        title = u.bold("Đang ghép ảnh cầu", 26, Ui.TEXT);
        root.addView(title);
        subtitle = u.text("Cứ để máy yên và giữ app mở. Thường mất khoảng một phút.", 14, Ui.TEXT_2);
        root.addView(subtitle, u.lp(-1, -2, 6));

        // Tiến độ: phần trăm lớn, thanh tiến độ và danh sách các bước.
        stepsCard = u.card();
        LinearLayout head = u.row();
        percentText = u.bold("0%", 34, Ui.TEXT);
        head.addView(percentText, new LinearLayout.LayoutParams(0, -2, 1));
        stepsCard.addView(head);
        progress = new ProgressBar(this, null, android.R.attr.progressBarStyleHorizontal);
        progress.setMax(100);
        progress.setProgressTintList(ColorStateList.valueOf(Ui.ACCENT));
        progress.setProgressBackgroundTintList(ColorStateList.valueOf(Ui.SURFACE_2));
        stepsCard.addView(progress, u.lp(-1, u.px(8), 8));

        stepIcons = new TextView[STEP_NAMES.length];
        stepLabels = new TextView[STEP_NAMES.length];
        for (int i = 0; i < STEP_NAMES.length; i++) {
            LinearLayout r = u.row();
            stepIcons[i] = u.bold("", 15, Ui.TEXT_3);
            stepIcons[i].setGravity(Gravity.CENTER);
            r.addView(stepIcons[i], new LinearLayout.LayoutParams(u.px(28), -2));
            stepLabels[i] = u.text(STEP_NAMES[i], 15, Ui.TEXT_3);
            r.addView(stepLabels[i], new LinearLayout.LayoutParams(0, -2, 1));
            stepsCard.addView(r, u.lp(-1, -2, i == 0 ? 16 : 8));
        }
        root.addView(stepsCard, u.lp(-1, -2, 20));

        // Kết quả: ảnh thu nhỏ và thông tin, hiện khi xong.
        resultCard = u.card();
        resultCard.setPadding(0, 0, 0, u.px(16));
        resultCard.setVisibility(View.GONE);
        thumbnail = new ImageView(this);
        thumbnail.setScaleType(ImageView.ScaleType.CENTER_CROP);
        thumbnail.setBackground(u.round(Ui.SURFACE_2, 20));
        thumbnail.setClipToOutline(true);
        thumbnail.setOnClickListener(v -> openImage());
        resultCard.addView(thumbnail, new LinearLayout.LayoutParams(-1, u.px(190)));
        resultText = u.text("", 14, Ui.TEXT_2);
        resultText.setPadding(u.px(16), u.px(12), u.px(16), 0);
        resultCard.addView(resultText);
        root.addView(resultCard, u.lp(-1, -2, 20));

        actions = new LinearLayout(this);
        actions.setOrientation(LinearLayout.VERTICAL);
        root.addView(actions, u.lp(-1, -2, 16));
        showActions(false, false);

        setContentView(scroll);
    }

    /** Nút ở cuối trang: khi xong có Xem/Chia sẻ; khi lỗi có Thử lại; luôn có Về thư viện. */
    private void showActions(boolean done, boolean failed) {
        actions.removeAllViews();
        if (done) {
            Button view = u.primary("Xem 360°");
            view.setTextSize(17);
            view.setOnClickListener(v -> openImage());
            actions.addView(view, new LinearLayout.LayoutParams(-1, u.px(56)));
            Button share = u.secondary("Chia sẻ");
            share.setOnClickListener(v -> share());
            Ui.setEnabled(share, job != null && job.savedUri != null);
            actions.addView(share, u.lp(-1, u.px(52), 10));
        } else if (failed) {
            Button retry = u.primary("Thử ghép lại");
            retry.setOnClickListener(v -> {
                if (job == null) return;
                job = start(job.dir);
                shownResult = false;
                title.setText("Đang ghép ảnh cầu");
                subtitle.setTextColor(Ui.TEXT_2);
                subtitle.setText("Cứ để máy yên và giữ app mở. Thường mất khoảng một phút.");
                showActions(false, false);
                ui.post(poll);
            });
            actions.addView(retry, new LinearLayout.LayoutParams(-1, u.px(56)));
        }
        Button back = u.ghost("Về thư viện", Ui.TEXT);
        back.setOnClickListener(v -> {
            startActivity(new Intent(this, MainActivity.class).addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP));
            finish();
        });
        actions.addView(back, u.lp(-1, u.px(52), done || failed ? 10 : 0));
    }

    private void paintSteps(int percent, boolean finished, boolean failed) {
        int current = 0;
        for (int i = 0; i < STEP_AT.length; i++) if (percent >= STEP_AT[i]) current = i;
        for (int i = 0; i < STEP_NAMES.length; i++) {
            boolean done = i < current || (finished && !failed);
            boolean active = i == current && !(finished && !failed);
            if (done) {
                stepIcons[i].setText("✓");
                stepIcons[i].setTextColor(Ui.GOOD);
                stepLabels[i].setTextColor(Ui.TEXT_2);
            } else if (active && failed) {
                stepIcons[i].setText("✕");
                stepIcons[i].setTextColor(Ui.DANGER);
                stepLabels[i].setTextColor(Ui.DANGER);
            } else if (active) {
                stepIcons[i].setText("●");
                stepIcons[i].setTextColor(Ui.ACCENT);
                stepLabels[i].setTextColor(Ui.TEXT);
            } else {
                stepIcons[i].setText("○");
                stepIcons[i].setTextColor(Ui.TEXT_3);
                stepLabels[i].setTextColor(Ui.TEXT_3);
            }
        }
    }

    private final Runnable poll = new Runnable() {
        @Override
        public void run() {
            if (job == null) return;
            int pct = job.percent;
            progress.setProgress(pct);
            percentText.setText(pct + "%");
            if (!job.finished) {
                long sec = (System.currentTimeMillis() - job.startedAt) / 1000;
                subtitle.setText(String.format(Locale.US,
                        "Cứ để máy yên và giữ app mở. Đã chạy %d giây, thường mất khoảng một phút.", sec));
                paintSteps(pct, false, false);
                ui.postDelayed(this, 300);
                return;
            }
            if (shownResult) return;
            shownResult = true;
            if (job.error != null) {
                paintSteps(pct, true, true);
                title.setText("Ghép chưa thành công");
                subtitle.setTextColor(Ui.DANGER);
                subtitle.setText(job.error);
                showActions(false, true);
                return;
            }
            paintSteps(100, true, false);
            stepsCard.setVisibility(View.GONE);
            title.setText("Xong rồi!");
            subtitle.setText("Ảnh cầu đã lưu vào Thư viện ảnh › Pictures › Sphere360.");
            resultText.setText(String.format(Locale.US, "%d×%d · ghép trong %d giây · chạm ảnh để xem 360°",
                    job.width, job.width / 2, job.elapsedMs / 1000));
            resultCard.setVisibility(View.VISIBLE);
            showThumbnail(new File(job.dir, "pano.jpg"));
            showActions(true, false);
        }
    };

    private void showThumbnail(File f) {
        BitmapFactory.Options o = new BitmapFactory.Options();
        o.inJustDecodeBounds = true;
        BitmapFactory.decodeFile(f.getAbsolutePath(), o);
        int sample = 1;
        while (o.outWidth / sample > 2048) sample *= 2;
        o = new BitmapFactory.Options();
        o.inSampleSize = sample;
        Bitmap bmp = BitmapFactory.decodeFile(f.getAbsolutePath(), o);
        if (bmp != null) thumbnail.setImageBitmap(bmp);
    }

    private void openImage() {
        if (job == null) return;
        Intent i = new Intent(this, ViewerActivity.class);
        i.putExtra(ViewerActivity.EXTRA_PATH, new File(job.dir, "pano.jpg").getAbsolutePath());
        startActivity(i);
    }

    /** Mở bảng chia sẻ của Android với ảnh đã lưu trong Thư viện (có metadata 360°). */
    private void share() {
        if (job == null || job.savedUri == null) return;
        Intent send = new Intent(Intent.ACTION_SEND);
        send.setType("image/jpeg");
        send.putExtra(Intent.EXTRA_STREAM, job.savedUri);
        send.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
        startActivity(Intent.createChooser(send, "Chia sẻ ảnh 360°"));
    }

    private Job start(String dir) {
        Job j = new Job(dir);
        current = j;
        ContentResolver resolver = getContentResolver();
        android.content.SharedPreferences prefs = getSharedPreferences(Settings.PREFS, MODE_PRIVATE);
        int mode = prefs.getInt(Settings.PREF_OUTPUT_WIDTH, Settings.DEFAULT_OUTPUT_WIDTH);
        boolean deghost = prefs.getBoolean(Settings.PREF_DEGHOST, true);

        Thread t = new Thread(null, () -> run(j, resolver, mode, deghost), "stitch", 64L << 20);
        t.setPriority(Thread.MAX_PRIORITY);
        t.start();
        return j;
    }

    private static void run(Job j, ContentResolver resolver, int mode, boolean deghost) {
        long t0 = System.currentTimeMillis();
        try {
            Session s = Session.load(new File(j.dir));
            int n = s.frames.size();
            String[] paths = new String[n];
            float[] rot = new float[9 * n];
            for (int i = 0; i < n; i++) {
                Session.Frame f = s.frames.get(i);
                paths[i] = new File(s.dir, f.file).getAbsolutePath();
                System.arraycopy(f.rotation, 0, rot, 9 * i, 9);
            }
            int nativeWidth = s.nativeOutputWidth();
            int width = mode <= 0 ? nativeWidth : Math.min(mode, nativeWidth);
            j.width = width;
            File out = s.panoramaFile();

            String err = NativeStitcher.stitch(paths, rot, s.focalPx, width, 95, deghost, out.getAbsolutePath(),
                    (percent, stage) -> {
                        j.percent = percent;
                        j.stage = stage;
                    });
            if (err != null) {
                j.error = err;
                return;
            }

            j.stage = "Đang lưu vào Thư viện ảnh";
            Uri uri = saveToGallery(resolver, out, width, n, new File(j.dir).getName());
            j.savedUri = uri;
            s.savedUri = uri.toString();
            s.savedWidth = width;
            s.save();
        } catch (Throwable e) {
            Log.e(TAG, "stitch", e);
            j.error = e.getMessage() != null ? e.getMessage() : e.toString();
        } finally {
            j.elapsedMs = System.currentTimeMillis() - t0;
            j.finished = true;
        }
    }

    private static Uri saveToGallery(ContentResolver resolver, File pano, int width, int sources, String name)
            throws IOException {
        ContentValues v = new ContentValues();
        v.put(MediaStore.Images.Media.DISPLAY_NAME, "Sphere360_" + name + ".jpg");
        v.put(MediaStore.Images.Media.MIME_TYPE, "image/jpeg");
        v.put(MediaStore.Images.Media.RELATIVE_PATH, "Pictures/Sphere360");
        v.put(MediaStore.Images.Media.IS_PENDING, 1);
        Uri uri = resolver.insert(MediaStore.Images.Media.EXTERNAL_CONTENT_URI, v);
        if (uri == null) throw new IOException("Không tạo được ảnh trong Thư viện");
        try (OutputStream os = resolver.openOutputStream(uri)) {
            if (os == null) throw new IOException("Không ghi được ảnh");
            GPanoWriter.copyWithXmp(pano, width, width / 2, sources, os);
        } catch (IOException e) {
            resolver.delete(uri, null, null);
            throw e;
        }
        v.clear();
        v.put(MediaStore.Images.Media.IS_PENDING, 0);
        resolver.update(uri, v, null, null);
        return uri;
    }
}
