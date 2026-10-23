package com.sphere360.app.home;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.graphics.drawable.ColorDrawable;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.LruCache;
import android.view.Gravity;
import android.view.View;
import android.view.Window;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.sphere360.app.capture.CaptureActivity;
import com.sphere360.app.data.Session;
import com.sphere360.app.data.Settings;
import com.sphere360.app.probe.ArProbeActivity;
import com.sphere360.app.stitch.StitchActivity;
import com.sphere360.app.ui.Ui;
import com.sphere360.app.viewer.ViewerActivity;

/** Trang chính: chọn cách chụp, bắt đầu chụp, thư viện các lượt chụp; cài đặt nằm trong hộp thoại. */
public class MainActivity extends Activity {

    private static final int REQ_CAMERA = 1;

    private static final int[] WIDTHS = {8192, 12288, 16384, 0};
    private static final String[] WIDTH_LABELS = {"8K · 8192×4096", "12K · 12288×6144", "16K · 16384×8192",
            "Tối đa theo camera"};
    private static final String[] WIDTH_NOTES = {"Nhanh, trộn mịn nhất. Khuyên dùng.", "Nét hơn, ghép lâu hơn.",
            "Rất lớn, ghép lâu.", "1 pixel gốc = 1 pixel ảnh cầu. File rất lớn."};

    /** Ảnh thu nhỏ đã giải mã, theo đường dẫn + thời điểm sửa file. */
    private static final LruCache<String, Bitmap> THUMBS = new LruCache<String, Bitmap>(24 * 1024 * 1024) {
        @Override
        protected int sizeOf(String key, Bitmap b) {
            return b.getByteCount();
        }
    };

    private final ExecutorService loader = Executors.newSingleThreadExecutor();
    private final Handler ui = new Handler(Looper.getMainLooper());
    private Ui u;
    private SharedPreferences prefs;
    private LinearLayout list;
    private TextView libraryCount;
    private TextView modeNote;
    private TextView[] modeTabs;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        u = new Ui(this);
        Ui.paintBars(this);
        prefs = getSharedPreferences(Settings.PREFS, MODE_PRIVATE);

        ScrollView scroll = new ScrollView(this);
        scroll.setBackgroundColor(Ui.BG);
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setPadding(u.px(20), u.px(16), u.px(20), u.px(24));
        scroll.addView(root);
        Ui.padForSystemBars(root, true, true);

        // Đầu trang: tên app và nút cài đặt.
        LinearLayout header = u.row();
        LinearLayout titles = new LinearLayout(this);
        titles.setOrientation(LinearLayout.VERTICAL);
        titles.addView(u.bold("Sphere360", 28, Ui.TEXT));
        titles.addView(u.text("Ảnh cầu 360° đủ cả trời và đất", 14, Ui.TEXT_2));
        header.addView(titles, new LinearLayout.LayoutParams(0, -2, 1));
        Button settings = u.secondary("Cài đặt");
        settings.setOnClickListener(v -> showSettings());
        header.addView(settings);
        root.addView(header);

        root.addView(captureCard(), u.lp(-1, -2, 24));
        root.addView(tipsCard(), u.lp(-1, -2, 12));

        LinearLayout libHeader = u.row();
        libHeader.addView(u.bold("Ảnh của bạn", 20, Ui.TEXT), new LinearLayout.LayoutParams(0, -2, 1));
        libraryCount = u.text("", 14, Ui.TEXT_3);
        libHeader.addView(libraryCount);
        root.addView(libHeader, u.lp(-1, -2, 32));

        list = new LinearLayout(this);
        list.setOrientation(LinearLayout.VERTICAL);
        root.addView(list, u.lp(-1, -2, 4));

        setContentView(scroll);
    }

    @Override
    protected void onResume() {
        super.onResume();
        refreshList();
    }

    @Override
    protected void onDestroy() {
        loader.shutdownNow();
        ui.removeCallbacksAndMessages(null);
        super.onDestroy();
    }

    // ---------------------------------------------------------------- chụp

    private View captureCard() {
        LinearLayout card = u.card();
        card.addView(u.overline("Chụp ảnh mới"));
        card.addView(u.bold("Bạn chụp theo cách nào?", 18, Ui.TEXT), u.lp(-1, -2, 6));

        // Hai lựa chọn kiểu tab: kê máy / cầm tay.
        LinearLayout seg = u.row();
        seg.setBackground(u.round(Ui.SURFACE_2, 14));
        seg.setPadding(u.px(4), u.px(4), u.px(4), u.px(4));
        String[] labels = {"Kê máy cố định", "Cầm tay"};
        modeTabs = new TextView[2];
        for (int i = 0; i < 2; i++) {
            TextView t = u.bold(labels[i], 15, Ui.TEXT_2);
            t.setGravity(Gravity.CENTER);
            t.setPadding(0, u.px(10), 0, u.px(10));
            final boolean handheld = i == 1;
            t.setOnClickListener(v -> {
                prefs.edit().putBoolean(Settings.PREF_HANDHELD, handheld).apply();
                updateModeTabs();
            });
            modeTabs[i] = t;
            seg.addView(t, new LinearLayout.LayoutParams(0, -2, 1));
        }
        card.addView(seg, u.lp(-1, -2, 14));

        modeNote = u.text("", 14, Ui.TEXT_2);
        card.addView(modeNote, u.lp(-1, -2, 12));
        updateModeTabs();

        Button capture = u.primary("Bắt đầu chụp");
        capture.setTextSize(17);
        capture.setOnClickListener(v -> startCapture());
        card.addView(capture, u.lp(-1, u.px(56), 18));
        return card;
    }

    private void updateModeTabs() {
        boolean handheld = prefs.getBoolean(Settings.PREF_HANDHELD, false);
        for (int i = 0; i < 2; i++) {
            boolean on = (i == 1) == handheld;
            modeTabs[i].setBackground(on ? u.round(Ui.TEXT, 11) : null);
            modeTabs[i].setTextColor(on ? Ui.BG : Ui.TEXT_2);
        }
        modeNote.setText(handheld
                ? "Xoay cả người, kẹp khuỷu tay sát người và giữ máy sát mặt. App chụp dày hơn (khoảng 34 tấm) để chỗ nối ít lệch."
                : "Xoay máy quanh một chỗ kê (hộp, gậy, chân máy) sao cho ống kính gần như đứng yên. Đẹp nhất, khoảng 27 tấm.");
    }

    private View tipsCard() {
        LinearLayout card = u.card();
        card.setPadding(u.px(18), u.px(14), u.px(18), u.px(14));
        LinearLayout head = u.row();
        head.addView(u.bold("Mẹo để ảnh đẹp", 15, Ui.TEXT), new LinearLayout.LayoutParams(0, -2, 1));
        TextView arrow = u.text("Xem", 14, Ui.ACCENT);
        head.addView(arrow);
        card.addView(head);

        TextView body = u.text(
                "•  Tắt quạt, chờ không có người đi qua. Vật chuyển động sẽ bị nhòe hoặc cắt đôi.\n" +
                "•  Tấm đầu tiên hướng vào chỗ cách 2–3 m: app khóa nét, sáng và màu theo tấm này.\n" +
                "•  Đưa vòng tròn vào chấm vàng rồi đứng yên tới khi vòng xanh chạy hết.\n" +
                "•  Tránh để vật gần hơn 1 m nằm ở mép khung, chỗ đó dễ lệch nhất.", 14, Ui.TEXT_2);
        body.setVisibility(View.GONE);
        card.addView(body, u.lp(-1, -2, 10));
        card.setOnClickListener(v -> {
            boolean show = body.getVisibility() != View.VISIBLE;
            body.setVisibility(show ? View.VISIBLE : View.GONE);
            arrow.setText(show ? "Ẩn" : "Xem");
        });
        return card;
    }

    private void startCapture() {
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{Manifest.permission.CAMERA}, REQ_CAMERA);
            return;
        }
        startActivity(new Intent(this, CaptureActivity.class));
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        if (requestCode == REQ_CAMERA && grantResults.length > 0
                && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            startActivity(new Intent(this, CaptureActivity.class));
        } else if (requestCode == REQ_CAMERA) {
            Toast.makeText(this, "Cần quyền camera để chụp", Toast.LENGTH_LONG).show();
        }
    }

    // ---------------------------------------------------------------- thư viện

    private void refreshList() {
        list.removeAllViews();
        List<Session> sessions = Session.listAll(this);
        libraryCount.setText(sessions.isEmpty() ? "" : sessions.size() + " lượt");
        if (sessions.isEmpty()) {
            LinearLayout empty = u.card();
            empty.setGravity(Gravity.CENTER_HORIZONTAL);
            TextView big = u.bold("Chưa có ảnh nào", 17, Ui.TEXT);
            big.setGravity(Gravity.CENTER);
            empty.addView(big);
            TextView small = u.text("Bấm \"Bắt đầu chụp\" ở trên. Ảnh ghép xong sẽ hiện ở đây và trong Thư viện ảnh.",
                    14, Ui.TEXT_2);
            small.setGravity(Gravity.CENTER);
            empty.addView(small, u.lp(-1, -2, 6));
            list.addView(empty, u.lp(-1, -2, 8));
            return;
        }
        for (Session s : sessions) list.addView(sessionCard(s), u.lp(-1, -2, 12));
    }

    private View sessionCard(Session s) {
        LinearLayout card = u.card();
        card.setPadding(0, 0, 0, u.px(14));
        final boolean stitched = s.panoramaFile().exists() || s.savedUri != null;
        final boolean canStitch = s.frames.size() >= 2;

        // Ảnh thu nhỏ: ảnh cầu nếu đã ghép, không thì tấm đầu tiên.
        FrameLayout media = new FrameLayout(this);
        media.setBackground(u.round(Ui.SURFACE_2, 20));
        media.setClipToOutline(true);
        ImageView thumb = new ImageView(this);
        thumb.setScaleType(ImageView.ScaleType.CENTER_CROP);
        media.addView(thumb, new FrameLayout.LayoutParams(-1, -1));
        File src = s.panoramaFile().exists() ? s.panoramaFile()
                : (s.frames.isEmpty() ? null : new File(s.dir, s.frames.get(0).file));
        if (src != null) loadThumb(src, thumb);
        if (stitched) {
            TextView badge = u.bold("360°", 12, Color.WHITE);
            badge.setBackground(u.round(0x99000000, 10));
            badge.setPadding(u.px(10), u.px(4), u.px(10), u.px(4));
            FrameLayout.LayoutParams blp = new FrameLayout.LayoutParams(-2, -2, Gravity.TOP | Gravity.START);
            blp.setMargins(u.px(12), u.px(12), 0, 0);
            media.addView(badge, blp);
            media.setOnClickListener(v -> openViewer(s));
        }
        card.addView(media, new LinearLayout.LayoutParams(-1, u.px(stitched ? 180 : 120)));

        LinearLayout info = new LinearLayout(this);
        info.setOrientation(LinearLayout.VERTICAL);
        info.setPadding(u.px(16), u.px(12), u.px(16), 0);
        LinearLayout titleRow = u.row();
        titleRow.addView(u.bold(s.title(), 16, Ui.TEXT), new LinearLayout.LayoutParams(0, -2, 1));
        TextView chip;
        if (s.savedUri != null) {
            chip = u.chip(s.savedWidth >= 1000 ? "Đã ghép · " + (s.savedWidth / 1024) + "K" : "Đã ghép", Ui.GOOD);
        } else if (canStitch) {
            chip = u.chip("Chưa ghép", Ui.WARN);
        } else {
            chip = u.chip("Thiếu ảnh", Ui.DANGER);
        }
        titleRow.addView(chip);
        info.addView(titleRow);
        info.addView(u.text(String.format(Locale.US, "%d / %d tấm", s.frames.size(), s.targetCount), 13, Ui.TEXT_3),
                u.lp(-1, -2, 2));

        LinearLayout actions = u.row();
        if (stitched) {
            Button view = u.primary("Xem 360°");
            view.setOnClickListener(v -> openViewer(s));
            actions.addView(view, new LinearLayout.LayoutParams(0, -2, 1.3f));
            Button again = u.secondary("Ghép lại");
            Ui.setEnabled(again, canStitch);
            again.setOnClickListener(v -> openStitcher(s));
            actions.addView(again, u.weight(8));
        } else {
            Button stitch = u.primary("Ghép ảnh");
            Ui.setEnabled(stitch, canStitch);
            stitch.setOnClickListener(v -> openStitcher(s));
            actions.addView(stitch, new LinearLayout.LayoutParams(0, -2, 1.3f));
        }
        Button del = u.ghost("Xóa", Ui.DANGER);
        del.setOnClickListener(v -> confirmDelete(s));
        LinearLayout.LayoutParams dlp = new LinearLayout.LayoutParams(-2, -2);
        dlp.leftMargin = u.px(8);
        actions.addView(del, dlp);
        info.addView(actions, u.lp(-1, -2, 14));
        card.addView(info);
        return card;
    }

    private void loadThumb(File f, ImageView target) {
        final String key = f.getAbsolutePath() + "@" + f.lastModified();
        Bitmap cached = THUMBS.get(key);
        if (cached != null) {
            target.setImageBitmap(cached);
            return;
        }
        target.setTag(key);
        loader.execute(() -> {
            BitmapFactory.Options o = new BitmapFactory.Options();
            o.inJustDecodeBounds = true;
            BitmapFactory.decodeFile(f.getAbsolutePath(), o);
            int sample = 1;
            while (o.outWidth / (sample * 2) >= 1024) sample *= 2;
            o = new BitmapFactory.Options();
            o.inSampleSize = sample;
            Bitmap b = BitmapFactory.decodeFile(f.getAbsolutePath(), o);
            if (b == null) return;
            THUMBS.put(key, b);
            ui.post(() -> {
                if (key.equals(target.getTag())) target.setImageBitmap(b);
            });
        });
    }

    private void openViewer(Session s) {
        Intent i = new Intent(this, ViewerActivity.class);
        if (s.panoramaFile().exists()) {
            i.putExtra(ViewerActivity.EXTRA_PATH, s.panoramaFile().getAbsolutePath());
        } else {
            i.setData(Uri.parse(s.savedUri));
            i.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
        }
        startActivity(i);
    }

    private void openStitcher(Session s) {
        Intent i = new Intent(this, StitchActivity.class);
        i.putExtra(StitchActivity.EXTRA_SESSION_DIR, s.dir.getAbsolutePath());
        startActivity(i);
    }

    private void confirmDelete(Session s) {
        new AlertDialog.Builder(this)
                .setTitle("Xóa lượt chụp " + s.title() + "?")
                .setMessage("Các ảnh gốc của lượt này sẽ bị xóa. Ảnh cầu đã lưu trong Thư viện ảnh vẫn giữ nguyên.")
                .setPositiveButton("Xóa", (d, w) -> {
                    s.deleteAll();
                    refreshList();
                })
                .setNegativeButton("Thôi", null)
                .show();
    }

    // ---------------------------------------------------------------- cài đặt

    private void showSettings() {
        Dialog d = new Dialog(this);
        d.requestWindowFeature(Window.FEATURE_NO_TITLE);
        ScrollView scroll = new ScrollView(this);
        LinearLayout box = new LinearLayout(this);
        box.setOrientation(LinearLayout.VERTICAL);
        box.setPadding(u.px(20), u.px(20), u.px(20), u.px(16));
        box.setBackground(u.round(Ui.SURFACE, 24));
        scroll.addView(box);

        box.addView(u.bold("Cài đặt", 20, Ui.TEXT));

        box.addView(u.overline("Kích thước ảnh xuất"), u.lp(-1, -2, 18));
        LinearLayout[] options = new LinearLayout[WIDTHS.length];
        for (int i = 0; i < WIDTHS.length; i++) {
            LinearLayout opt = new LinearLayout(this);
            opt.setOrientation(LinearLayout.VERTICAL);
            opt.setPadding(u.px(14), u.px(10), u.px(14), u.px(10));
            opt.addView(u.bold(WIDTH_LABELS[i], 15, Ui.TEXT));
            opt.addView(u.text(WIDTH_NOTES[i], 13, Ui.TEXT_2));
            final int w = WIDTHS[i];
            opt.setOnClickListener(v -> {
                prefs.edit().putInt(Settings.PREF_OUTPUT_WIDTH, w).apply();
                paintOptions(options);
            });
            options[i] = opt;
            box.addView(opt, u.lp(-1, -2, 8));
        }
        paintOptions(options);

        box.addView(u.overline("Xử lý thị sai"), u.lp(-1, -2, 20));
        Switch deghost = new Switch(this);
        deghost.setText("Uốn ảnh cho khớp ở chỗ nối");
        deghost.setTextColor(Ui.TEXT);
        deghost.setTextSize(15);
        deghost.setChecked(prefs.getBoolean(Settings.PREF_DEGHOST, true));
        deghost.setOnCheckedChangeListener((b, on) -> prefs.edit().putBoolean(Settings.PREF_DEGHOST, on).apply());
        box.addView(deghost, u.lp(-1, -2, 8));
        box.addView(u.text("Bật: vật ở gần ít bị nhân đôi. Tắt: giữ nguyên hình học, chỗ lệch nhiều có thể chồng hình. " +
                "Đổi xong bấm \"Ghép lại\" để so.", 13, Ui.TEXT_2), u.lp(-1, -2, 4));

        box.addView(u.overline("Thử nghiệm"), u.lp(-1, -2, 20));
        Button probe = u.secondary("Thử ARCore (vị trí + độ sâu)");
        probe.setOnClickListener(v -> {
            d.dismiss();
            startActivity(new Intent(this, ArProbeActivity.class));
        });
        box.addView(probe, u.lp(-1, -2, 8));

        Button done = u.primary("Xong");
        done.setOnClickListener(v -> d.dismiss());
        box.addView(done, u.lp(-1, -2, 20));

        d.setContentView(scroll);
        Window w = d.getWindow();
        if (w != null) {
            w.setBackgroundDrawable(new ColorDrawable(Color.TRANSPARENT));
            w.setLayout((int) (getResources().getDisplayMetrics().widthPixels * 0.92), -2);
        }
        d.show();
    }

    private void paintOptions(LinearLayout[] options) {
        int selected = prefs.getInt(Settings.PREF_OUTPUT_WIDTH, Settings.DEFAULT_OUTPUT_WIDTH);
        for (int i = 0; i < options.length; i++) {
            boolean on = WIDTHS[i] == selected;
            options[i].setBackground(on ? u.outline(0x1FFFC83D, Ui.ACCENT, 14) : u.outline(Ui.SURFACE_2, Ui.SURFACE_2, 14));
        }
    }
}
