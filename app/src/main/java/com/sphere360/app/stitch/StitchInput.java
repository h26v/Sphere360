package com.sphere360.app.stitch;

import java.io.File;

import com.sphere360.app.data.Session;

/** Một lượt chụp ở dạng mảng để truyền xuống bộ ghép C++. */
public final class StitchInput {

    public final String[] paths;
    public final float[] rotations;       // 9 số mỗi ảnh: camera -> toàn cảnh, hàng trước
    public final double focalPx;
    public final String[] bridgePaths;
    public final float[] bridgeRotations;
    public final int[] bridgeAfter;       // chỉ số tấm chụp ngay trước mỗi khung cầu nối
    public final double bridgeFocalPx;
    public final int bridgeRotation;

    private StitchInput(Session s, boolean withBridges) {
        int n = s.frames.size();
        paths = new String[n];
        rotations = new float[9 * n];
        for (int i = 0; i < n; i++) {
            Session.Frame f = s.frames.get(i);
            paths[i] = new File(s.dir, f.file).getAbsolutePath();
            System.arraycopy(f.rotation, 0, rotations, 9 * i, 9);
        }
        focalPx = s.focalPx;
        int m = withBridges && s.bridgeFocalPx > 0 ? s.bridges.size() : 0;
        bridgePaths = new String[m];
        bridgeRotations = new float[9 * m];
        bridgeAfter = new int[m];
        for (int i = 0; i < m; i++) {
            Session.Bridge b = s.bridges.get(i);
            bridgePaths[i] = new File(s.dir, b.file).getAbsolutePath();
            System.arraycopy(b.rotation, 0, bridgeRotations, 9 * i, 9);
            bridgeAfter[i] = b.after;
        }
        bridgeFocalPx = s.bridgeFocalPx;
        bridgeRotation = s.bridgeRotation;
    }

    public static StitchInput from(Session s) {
        return new StitchInput(s, true);
    }

    public int frameCount() {
        return paths.length;
    }
}
