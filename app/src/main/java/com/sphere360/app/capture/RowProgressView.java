package com.sphere360.app.capture;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.view.View;

import java.util.ArrayList;
import java.util.List;

import com.sphere360.app.ui.Ui;

/** Bản đồ nhỏ các tấm cần chụp, xếp theo hàng từ đỉnh trời xuống mặt đất: xanh = xong, vàng = đang tới. */
final class RowProgressView extends View {

    /** Thứ tự vẽ từ trên xuống: đỉnh, hàng trên, hàng ngang, hàng dưới, đáy. */
    private static final int[] ROW_ORDER = {3, 1, 0, 2, 4};

    private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final float dp;
    private List<CapturePlan.Target> targets;
    private int highlight = -1;

    RowProgressView(Context ctx) {
        super(ctx);
        dp = ctx.getResources().getDisplayMetrics().density;
    }

    void setTargets(List<CapturePlan.Target> targets) {
        this.targets = targets;
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
}
