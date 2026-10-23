package com.sphere360.app.ui;

import android.app.Activity;
import android.content.Context;
import android.content.res.ColorStateList;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.Drawable;
import android.graphics.drawable.GradientDrawable;
import android.graphics.drawable.RippleDrawable;
import android.view.Gravity;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

/** Màu và các thành phần giao diện dùng chung: nút bo tròn, thẻ, nhãn trạng thái. */
public final class Ui {

    public static final int BG = 0xFF0E1013;
    public static final int SURFACE = 0xFF181B20;
    public static final int SURFACE_2 = 0xFF22262D;
    public static final int OUTLINE = 0xFF2E333B;
    public static final int TEXT = 0xFFF3F4F6;
    public static final int TEXT_2 = 0xFFA7ADB7;
    public static final int TEXT_3 = 0xFF6E7580;
    /** Vàng: trùng màu chấm mục tiêu trên màn hình chụp. */
    public static final int ACCENT = 0xFFFFC83D;
    public static final int ON_ACCENT = 0xFF231A00;
    public static final int GOOD = 0xFF4CAF50;
    public static final int WARN = 0xFFFFB74D;
    public static final int DANGER = 0xFFEF5350;

    public final Context ctx;
    public final float dp;

    public Ui(Context ctx) {
        this.ctx = ctx;
        dp = ctx.getResources().getDisplayMetrics().density;
    }

    public int px(float v) {
        return Math.round(v * dp);
    }

    /** Thanh trạng thái và thanh điều hướng cùng màu nền. */
    public static void paintBars(Activity a) {
        a.getWindow().setStatusBarColor(BG);
        a.getWindow().setNavigationBarColor(BG);
    }

    /**
     * Android 15 vẽ app tràn dưới thanh trạng thái và thanh điều hướng: cộng thêm khoảng đó vào padding
     * (chỉ cạnh nào được chọn) để nội dung không bị che.
     */
    public static void padForSystemBars(View v, boolean top, boolean bottom) {
        final int l = v.getPaddingLeft(), t = v.getPaddingTop(), r = v.getPaddingRight(), b = v.getPaddingBottom();
        v.setOnApplyWindowInsetsListener((view, insets) -> {
            int it, ib;
            if (android.os.Build.VERSION.SDK_INT >= 30) {
                android.graphics.Insets bars = insets.getInsets(android.view.WindowInsets.Type.systemBars());
                it = bars.top;
                ib = bars.bottom;
            } else {
                it = insets.getSystemWindowInsetTop();
                ib = insets.getSystemWindowInsetBottom();
            }
            view.setPadding(l, t + (top ? it : 0), r, b + (bottom ? ib : 0));
            return insets;
        });
        v.requestApplyInsets();
    }

    public GradientDrawable round(int color, float radiusDp) {
        GradientDrawable g = new GradientDrawable();
        g.setColor(color);
        g.setCornerRadius(radiusDp * dp);
        return g;
    }

    public GradientDrawable outline(int fill, int stroke, float radiusDp) {
        GradientDrawable g = round(fill, radiusDp);
        g.setStroke(px(1), stroke);
        return g;
    }

    private Drawable ripple(Drawable content, float radiusDp) {
        return new RippleDrawable(ColorStateList.valueOf(0x33FFFFFF), content, round(Color.WHITE, radiusDp));
    }

    public TextView text(String s, float sp, int color) {
        TextView t = new TextView(ctx);
        t.setText(s);
        t.setTextSize(sp);
        t.setTextColor(color);
        t.setLineSpacing(0, 1.15f);
        return t;
    }

    public TextView bold(String s, float sp, int color) {
        TextView t = text(s, sp, color);
        t.setTypeface(Typeface.create("sans-serif-medium", Typeface.NORMAL));
        return t;
    }

    /** Tiêu đề nhỏ viết hoa phía trên một nhóm. */
    public TextView overline(String s) {
        TextView t = bold(s.toUpperCase(java.util.Locale.ROOT), 12, TEXT_3);
        t.setLetterSpacing(0.08f);
        return t;
    }

    private Button button(String label, int fill, int textColor, int strokeColor) {
        Button b = new Button(ctx);
        b.setText(label);
        b.setAllCaps(false);
        b.setTextSize(15);
        b.setTextColor(textColor);
        b.setTypeface(Typeface.create("sans-serif-medium", Typeface.NORMAL));
        b.setStateListAnimator(null);
        b.setMinHeight(0);
        b.setSingleLine(true);
        b.setMinimumHeight(0);
        b.setPadding(px(18), px(12), px(18), px(12));
        GradientDrawable shape = strokeColor != 0 ? outline(fill, strokeColor, 14) : round(fill, 14);
        b.setBackground(ripple(shape, 14));
        return b;
    }

    public Button primary(String label) {
        return button(label, ACCENT, ON_ACCENT, 0);
    }

    public Button secondary(String label) {
        return button(label, SURFACE_2, TEXT, 0);
    }

    public Button ghost(String label, int textColor) {
        return button(label, Color.TRANSPARENT, textColor, OUTLINE);
    }

    /** Nút mờ trên nền ảnh (màn hình chụp, trình xem). */
    public Button overlay(String label) {
        Button b = button(label, 0x66000000, Color.WHITE, 0x33FFFFFF);
        b.setPadding(px(16), px(10), px(16), px(10));
        return b;
    }

    public static void setEnabled(View v, boolean on) {
        v.setEnabled(on);
        v.setAlpha(on ? 1f : 0.38f);
    }

    public LinearLayout card() {
        LinearLayout c = new LinearLayout(ctx);
        c.setOrientation(LinearLayout.VERTICAL);
        c.setBackground(round(SURFACE, 20));
        c.setPadding(px(18), px(18), px(18), px(18));
        return c;
    }

    /** Nhãn trạng thái nhỏ có nền nhạt cùng màu chữ. */
    public TextView chip(String s, int color) {
        TextView t = bold(s, 12, color);
        t.setBackground(round((color & 0x00FFFFFF) | 0x26000000, 10));
        t.setPadding(px(10), px(4), px(10), px(4));
        return t;
    }

    public LinearLayout row() {
        LinearLayout r = new LinearLayout(ctx);
        r.setOrientation(LinearLayout.HORIZONTAL);
        r.setGravity(Gravity.CENTER_VERTICAL);
        return r;
    }

    public LinearLayout.LayoutParams lp(int w, int h, int topMarginDp) {
        LinearLayout.LayoutParams l = new LinearLayout.LayoutParams(w, h);
        l.topMargin = px(topMarginDp);
        return l;
    }

    public LinearLayout.LayoutParams weight(int leftMarginDp) {
        LinearLayout.LayoutParams l = new LinearLayout.LayoutParams(0, -2, 1);
        l.leftMargin = px(leftMarginDp);
        return l;
    }

    public View space(int dpWidth) {
        View v = new View(ctx);
        v.setLayoutParams(new LinearLayout.LayoutParams(px(dpWidth), 1));
        return v;
    }
}
