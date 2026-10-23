package com.sphere360.app.capture;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.view.View;

import java.util.List;

import com.sphere360.app.util.Rot;

/** Vẽ các điểm cần chụp lên khung ngắm, vòng tròn ngắm ở giữa và mũi tên chỉ hướng tấm kế tiếp. */
final class GuideOverlayView extends View {

    private final Paint ring = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint fill = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint arc = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final float dp;

    private float[] deviceToWorld;
    private List<CapturePlan.Target> targets;
    private int next = -1;
    private double hFovDeg = 60;
    private float hold;
    private float speedRatio = -1;
    private boolean ready;

    GuideOverlayView(Context ctx) {
        super(ctx);
        dp = ctx.getResources().getDisplayMetrics().density;
        ring.setStyle(Paint.Style.STROKE);
        ring.setStrokeWidth(3 * dp);
        arc.setStyle(Paint.Style.STROKE);
        arc.setStrokeWidth(6 * dp);
        arc.setColor(Color.rgb(76, 175, 80));
        arc.setStrokeCap(Paint.Cap.ROUND);
    }

    void setTargets(List<CapturePlan.Target> targets, double hFovDeg) {
        this.targets = targets;
        this.hFovDeg = hFovDeg;
    }

    /** speedRatio = tốc độ xoay / ngưỡng ổn định; ready = đã ngắm đúng và đủ yên. */
    void setStability(float speedRatio, boolean ready) {
        this.speedRatio = speedRatio;
        this.ready = ready;
    }

    void update(float[] deviceToWorld, int next, float hold) {
        this.deviceToWorld = deviceToWorld;
        this.next = next;
        this.hold = hold;
        invalidate();
    }

    @Override
    protected void onDraw(Canvas c) {
        float cx = getWidth() / 2f, cy = getHeight() / 2f;
        if (deviceToWorld == null || targets == null) return;
        float f = (float) ((getWidth() / 2.0) / Math.tan(Math.toRadians(hFovDeg / 2)));

        for (int i = 0; i < targets.size(); i++) {
            CapturePlan.Target t = targets.get(i);
            float[] d = Rot.applyT(deviceToWorld, t.dir);
            float forward = -d[2];
            if (forward < 0.15f) continue;
            float sx = cx + f * d[0] / forward;
            float sy = cy - f * d[1] / forward;
            if (t.done) {
                fill.setColor(Color.argb(200, 76, 175, 80));
                c.drawCircle(sx, sy, 9 * dp, fill);
            } else if (i == next) {
                fill.setColor(Color.argb(90, 255, 214, 0));
                c.drawCircle(sx, sy, 22 * dp, fill);
                ring.setColor(Color.rgb(255, 214, 0));
                c.drawCircle(sx, sy, 22 * dp, ring);
            } else {
                ring.setColor(Color.argb(200, 255, 255, 255));
                c.drawCircle(sx, sy, 14 * dp, ring);
            }
        }

        // Vòng ngắm: chuyển xanh khi đã ngắm đúng và đủ yên; vòng cung đếm thời gian giữ yên.
        ring.setColor(ready ? Color.rgb(76, 175, 80) : Color.WHITE);
        float r = 30 * dp;
        c.drawCircle(cx, cy, r, ring);
        c.drawLine(cx - 8 * dp, cy, cx + 8 * dp, cy, ring);
        c.drawLine(cx, cy - 8 * dp, cx, cy + 8 * dp, ring);
        if (hold > 0) {
            c.drawArc(new RectF(cx - r, cy - r, cx + r, cy + r), -90, 360 * hold, false, arc);
        }
        if (speedRatio >= 0) drawStabilityBar(c, cx, cy + r + 22 * dp);

        // Mũi tên khi tấm kế tiếp nằm ngoài khung.
        if (next >= 0 && next < targets.size()) {
            float[] d = Rot.applyT(deviceToWorld, targets.get(next).dir);
            float forward = -d[2];
            boolean visible = false;
            if (forward > 0.15f) {
                float sx = cx + f * d[0] / forward, sy = cy - f * d[1] / forward;
                visible = sx > 0 && sx < getWidth() && sy > 0 && sy < getHeight();
            }
            if (!visible) {
                float ax = d[0], ay = -d[1];
                float len = (float) Math.hypot(ax, ay);
                if (len < 1e-3f) {
                    ax = 1;
                    ay = 0;
                    len = 1;
                }
                ax /= len;
                ay /= len;
                drawArrow(c, cx + ax * 90 * dp, cy + ay * 90 * dp, ax, ay);
            }
        }
    }

    /**
     * Thanh rung: dài theo tốc độ xoay, thang đo tới 3 lần ngưỡng; vạch trắng là ngưỡng.
     * Xanh = đủ yên để chụp, vàng = gần đạt, đỏ = đang xoay.
     */
    private void drawStabilityBar(Canvas c, float cx, float y) {
        float w = 140 * dp, h = 8 * dp, left = cx - w / 2;
        fill.setColor(Color.argb(120, 0, 0, 0));
        c.drawRoundRect(new RectF(left, y, left + w, y + h), h / 2, h / 2, fill);
        float frac = Math.max(0.04f, Math.min(1f, speedRatio / 3f));
        int color = speedRatio < 1 ? Color.rgb(76, 175, 80)
                : speedRatio < 2 ? Color.rgb(255, 193, 7) : Color.rgb(244, 67, 54);
        fill.setColor(color);
        c.drawRoundRect(new RectF(left, y, left + w * frac, y + h), h / 2, h / 2, fill);
        fill.setColor(Color.WHITE);
        float tick = left + w / 3f;
        c.drawRect(tick - dp, y - 3 * dp, tick + dp, y + h + 3 * dp, fill);
    }

    private void drawArrow(Canvas c, float x, float y, float dx, float dy) {
        float s = 22 * dp;
        Path p = new Path();
        p.moveTo(x + dx * s, y + dy * s);
        p.lineTo(x - dy * s * 0.7f, y + dx * s * 0.7f);
        p.lineTo(x + dy * s * 0.7f, y - dx * s * 0.7f);
        p.close();
        fill.setColor(Color.rgb(255, 214, 0));
        c.drawPath(p, fill);
    }
}
