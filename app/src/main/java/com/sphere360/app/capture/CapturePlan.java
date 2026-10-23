package com.sphere360.app.capture;

import java.util.ArrayList;
import java.util.List;

/**
 * Danh sách góc cần chụp để phủ kín mặt cầu: hàng ngang, hàng trên, hàng dưới, đỉnh và đáy.
 * Hướng dùng hệ trục "thế giới" Z hướng lên, Y là hướng nhìn lúc bắt đầu chụp, yaw tăng khi xoay phải.
 */
final class CapturePlan {

    /** Tỉ lệ chồng lấn giữa hai tấm cạnh nhau (0.3 = 30%). */
    private static final double OVERLAP = 0.3;
    /**
     * Chế độ cầm tay: xoay người thì ống kính đi theo vòng tròn quanh người. Bước xoay nhỏ hơn làm quãng
     * dịch giữa hai tấm kề nhau ngắn lại (thị sai nhỏ hơn) và vùng chồng rộng hơn để chọn đường nối.
     */
    private static final double OVERLAP_HANDHELD = 0.48;

    static final class Target {
        final int row;           // 0 = ngang, 1 = trên, 2 = dưới, 3 = đỉnh, 4 = đáy
        final double yawDeg;
        final double pitchDeg;
        final float[] dir;       // vector đơn vị trong hệ thế giới
        boolean done;

        Target(int row, double yawDeg, double pitchDeg) {
            this.row = row;
            this.yawDeg = yawDeg;
            this.pitchDeg = pitchDeg;
            double y = Math.toRadians(yawDeg), p = Math.toRadians(pitchDeg);
            dir = new float[]{
                    (float) (Math.cos(p) * Math.sin(y)),
                    (float) (Math.cos(p) * Math.cos(y)),
                    (float) Math.sin(p)};
        }
    }

    static final String[] ROW_NAMES = {"Hàng ngang", "Hàng trên", "Hàng dưới", "Đỉnh trời", "Mặt đất"};

    static List<Target> create(double hFovDeg, double vFovDeg, boolean handheld) {
        List<Target> list = new ArrayList<>();
        double overlap = handheld ? OVERLAP_HANDHELD : OVERLAP;
        double rowPitch = Math.min(60, vFovDeg * 0.62);

        int n0 = count(360 / (hFovDeg * (1 - overlap)));
        addRow(list, 0, 0, n0, 0);

        // Với hàng nghiêng, chỗ hẹp nhất cần phủ là mép ngang với đỉnh của hàng giữa.
        double edge = vFovDeg / 2;
        double below = Math.toRadians(rowPitch - edge);
        double p = Math.toRadians(rowPitch);
        double forward = Math.cos(p) + Math.tan(below) * Math.sin(p);
        double yawHalf = Math.toDegrees(Math.atan(Math.tan(Math.toRadians(hFovDeg / 2)) / forward));
        int n1 = Math.max(4, count(360 / (2 * yawHalf * (1 - overlap))));
        // Lệch nửa bước so với hàng giữa để các tấm đan xen.
        addRow(list, 1, rowPitch, n1, 180.0 / n1);
        addRow(list, 2, -rowPitch, n1, 180.0 / n1);

        list.add(new Target(3, 0, 90));
        list.add(new Target(4, 0, -90));
        return list;
    }

    private static int count(double v) {
        return (int) Math.ceil(v - 1e-6);
    }

    private static void addRow(List<Target> list, int row, double pitch, int n, double offset) {
        for (int i = 0; i < n; i++) {
            double yaw = offset + 360.0 * i / n;
            if (yaw >= 180) yaw -= 360;
            list.add(new Target(row, yaw, pitch));
        }
    }
}
