package com.sphere360.app.capture;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.view.View;

import java.util.ArrayList;
import java.util.List;

import com.sphere360.app.ui.Ui;

/**
 * Bản đồ nhỏ các tấm cần chụp, xếp theo hàng từ đỉnh trời xuống mặt đất: xanh = xong, vàng = đang tới. Vệt mảnh
 * là đường lia máy theo các khung cầu nối: liền khi hai khung đủ gần để bộ ghép dò qua, đứt (chấm cam) khi không.
 */
final class RowProgressView extends View {

    /** Thứ tự vẽ từ trên xuống: đỉnh, hàng trên, hàng ngang, hàng dưới, đáy. */
    private static final int[] ROW_ORDER = {3, 1, 0, 2, 4};

    private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final float dp;
    private List<CapturePlan.Target> targets;
    private int highlight = -1;

    /** Một điểm của vệt đường lia. */
    private static final class TrailPoint {
        final double yawDeg, pitchDeg;
        final boolean linked;  // nối được với điểm trước
        final boolean anchor;  // tấm chính: chỉ để nối vệt, không vẽ chấm

        TrailPoint(double yawDeg, double pitchDeg, boolean linked, boolean anchor) {
            this.yawDeg = yawDeg;
            this.pitchDeg = pitchDeg;
            this.linked = linked;
            this.anchor = anchor;
        }
    }

    private final List<TrailPoint> trail = new ArrayList<>();

    RowProgressView(Context ctx) {
        super(ctx);
        dp = ctx.getResources().getDisplayMetrics().density;
    }

    void setTargets(List<CapturePlan.Target> targets) {
        this.targets = targets;
        invalidate();
    }

    void addTrailPoint(double yawDeg, double pitchDeg, boolean linked, boolean anchor) {
        trail.add(new TrailPoint(yawDeg, pitchDeg, linked, anchor));
        invalidate();
    }

    void setHighlight(int index) {
        if (index != highlight) {
            highlight = index;
            invalidate();
        }
    }

    @Override
    protected void onMeasure(int wSpec, int hSpec) {
        setMeasuredDimension(MeasureSpec.getSize(wSpec), Math.round(ROW_ORDER.length * 12 * dp));
    }

    @Override
    protected void onDraw(Canvas c) {
        if (targets == null) return;
        float rowH = getHeight() / (float) ROW_ORDER.length;
        int widest = 1;
        List<List<Integer>> rows = new ArrayList<>();
        for (int r : ROW_ORDER) {
            List<Integer> idx = new ArrayList<>();
            for (int i = 0; i < targets.size(); i++) if (targets.get(i).row == r) idx.add(i);
            rows.add(idx);
            widest = Math.max(widest, idx.size());
        }
        float step = Math.min(18 * dp, getWidth() * 0.8f / widest);
        float radius = Math.min(4 * dp, step * 0.32f);
        drawTrail(c, rows, rowH, step);
        paint.setStyle(Paint.Style.FILL);
        for (int k = 0; k < rows.size(); k++) {
            List<Integer> idx = rows.get(k);
            float cy = rowH * (k + 0.5f);
            float x0 = getWidth() / 2f - step * (idx.size() - 1) / 2f;
            for (int j = 0; j < idx.size(); j++) {
                int i = idx.get(j);
                float cx = x0 + step * j;
                if (targets.get(i).done) {
                    paint.setColor(Ui.GOOD);
                    c.drawCircle(cx, cy, radius, paint);
                } else if (i == highlight) {
                    paint.setColor(Ui.ACCENT);
                    c.drawCircle(cx, cy, radius * 1.45f, paint);
                } else {
                    paint.setColor(0x59FFFFFF);
                    c.drawCircle(cx, cy, radius * 0.8f, paint);
                }
            }
        }
    }

    /** Vị trí x (theo hàng thứ k trên bản đồ) của hướng yaw: nội suy giữa các chấm của hàng theo vòng tròn. */
    private float rowX(List<Integer> idx, double yawDeg, float step) {
        float center = getWidth() / 2f;
        int n = idx.size();
        if (n < 2) return center;
        float x0 = center - step * (n - 1) / 2f;
        double yaw0 = targets.get(idx.get(0)).yawDeg;
        double u = ((yawDeg - yaw0) % 360 + 360) % 360 / (360.0 / n);  // chỉ số (lẻ) trong hàng, 0..n
        if (u > n - 0.5) u -= n;  // nửa sau khoảng cuối -> vẽ bên trái chấm đầu
        return x0 + step * (float) u;
    }

    private void drawTrail(Canvas c, List<List<Integer>> rows, float rowH, float step) {
        if (trail.isEmpty()) return;
        // Pitch trung bình của từng hàng trên bản đồ (từ trên xuống: giảm dần).
        int nr = rows.size();
        double[] pitch = new double[nr];
        for (int k = 0; k < nr; k++) {
            double sum = 0;
            for (int i : rows.get(k)) sum += targets.get(i).pitchDeg;
            pitch[k] = rows.get(k).isEmpty() ? 0 : sum / rows.get(k).size();
        }
        paint.setStrokeWidth(1.5f * dp);
        paint.setStrokeCap(Paint.Cap.ROUND);
        float px = 0, py = 0;
        boolean havePrev = false;
        for (TrailPoint t : trail) {
            // Hai hàng kẹp pitch của điểm, nội suy cả y lẫn x giữa hai hàng đó.
            int k = 0;
            while (k + 1 < nr && pitch[k + 1] >= t.pitchDeg) k++;
            int k2 = Math.min(nr - 1, k + 1);
            double w = k2 == k || pitch[k] == pitch[k2] ? 0
                    : Math.max(0, Math.min(1, (pitch[k] - t.pitchDeg) / (pitch[k] - pitch[k2])));
            float x = (float) (rowX(rows.get(k), t.yawDeg, step) * (1 - w) + rowX(rows.get(k2), t.yawDeg, step) * w);
            float y = (float) (rowH * (k + 0.5f + w * (k2 - k)));
            if (havePrev && t.linked && Math.abs(x - px) < 3 * step) {
                paint.setStyle(Paint.Style.STROKE);
                paint.setColor(0xB34CAF50);
                c.drawLine(px, py, x, y, paint);
            }
            if (!t.anchor) {
                paint.setStyle(Paint.Style.FILL);
                paint.setColor(t.linked ? 0x99FFFFFF : Ui.WARN);
                c.drawCircle(x, y, (t.linked ? 0.9f : 1.6f) * dp, paint);
            }
            px = x;
            py = y;
            havePrev = true;
        }
    }
}
