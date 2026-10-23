package com.sphere360.app.capture;

import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.util.Size;
import android.util.SizeF;

import java.util.Arrays;

/** Thông số camera sau: kích thước JPEG lớn nhất, tiêu cự theo pixel, góc nhìn khi cầm dọc. */
final class CameraSpec {

    final String id;
    final Size jpegSize;      // theo hướng cảm biến (ngang)
    final Size previewSize;   // theo hướng cảm biến (ngang)
    final int sensorOrientation;
    final double focalPx;     // tiêu cự của ảnh JPEG đầy đủ, tính theo pixel
    final double hFovDeg;     // góc ngang khi cầm dọc (cạnh ngắn)
    final double vFovDeg;     // góc dọc khi cầm dọc (cạnh dài)
    final boolean manualFocus; // đặt được khoảng lấy nét bằng tay (để khóa nét)
    final boolean realtimeTimestamps; // mốc thời gian ảnh cùng đồng hồ với cảm biến chuyển động
    final boolean ois;                // có chống rung quang học
    final Size bridgeSize;            // luồng YUV cho khung cầu nối (ngang), null nếu không có
    final double bridgeFocalPx;       // tiêu cự theo pixel của khung cầu nối

    private CameraSpec(String id, Size jpegSize, Size previewSize, Size bridgeSize, int sensorOrientation,
                       double focalPx, boolean manualFocus, boolean realtimeTimestamps, boolean ois) {
        this.id = id;
        this.bridgeSize = bridgeSize;
        // Cùng tỉ lệ 4:3 lấy trọn cảm biến như ảnh JPEG nên tiêu cự tỉ lệ theo chiều rộng.
        this.bridgeFocalPx = bridgeSize == null ? 0 : focalPx * bridgeSize.getWidth() / jpegSize.getWidth();
        this.ois = ois;
        this.manualFocus = manualFocus;
        this.realtimeTimestamps = realtimeTimestamps;
        this.jpegSize = jpegSize;
        this.previewSize = previewSize;
        this.sensorOrientation = sensorOrientation;
        this.focalPx = focalPx;
        int shortSide = Math.min(jpegSize.getWidth(), jpegSize.getHeight());
        int longSide = Math.max(jpegSize.getWidth(), jpegSize.getHeight());
        this.hFovDeg = Math.toDegrees(2 * Math.atan(shortSide / (2 * focalPx)));
        this.vFovDeg = Math.toDegrees(2 * Math.atan(longSide / (2 * focalPx)));
    }

    /** Ảnh sau khi xoay đứng: rộng = cạnh ngắn. */
    int uprightWidth() {
        return Math.min(jpegSize.getWidth(), jpegSize.getHeight());
    }

    int uprightHeight() {
        return Math.max(jpegSize.getWidth(), jpegSize.getHeight());
    }

    static CameraSpec pickBackCamera(CameraManager cm) throws CameraAccessException {
        for (String id : cm.getCameraIdList()) {
            CameraCharacteristics c = cm.getCameraCharacteristics(id);
            Integer facing = c.get(CameraCharacteristics.LENS_FACING);
            if (facing == null || facing != CameraCharacteristics.LENS_FACING_BACK) continue;
            StreamConfigurationMap map = c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            if (map == null) continue;

            Size jpeg = largest4x3(map.getOutputSizes(ImageFormat.JPEG), Integer.MAX_VALUE);
            Size preview = largest4x3(map.getOutputSizes(SurfaceTexture.class), 1600 * 1200);
            if (jpeg == null || preview == null) continue;
            Size bridge = largest4x3(map.getOutputSizes(ImageFormat.YUV_420_888), 1440 * 1080);

            float[] focals = c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS);
            SizeF phys = c.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE);
            Size pixelArray = c.get(CameraCharacteristics.SENSOR_INFO_PIXEL_ARRAY_SIZE);
            Rect active = c.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE);
            Integer orientation = c.get(CameraCharacteristics.SENSOR_ORIENTATION);

            double focalPx;
            if (focals != null && focals.length > 0 && phys != null && pixelArray != null && active != null) {
                double fArray = focals[0] * pixelArray.getWidth() / phys.getWidth();
                focalPx = fArray * jpeg.getWidth() / active.width();
            } else {
                // Không có thông số: giả định góc ngang 65 độ.
                focalPx = jpeg.getWidth() / (2 * Math.tan(Math.toRadians(65) / 2));
            }
            // Ống kính cố định nét có khoảng lấy nét tối thiểu bằng 0, không cần khóa.
            int[] afModes = c.get(CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES);
            Float minFocus = c.get(CameraCharacteristics.LENS_INFO_MINIMUM_FOCUS_DISTANCE);
            boolean manualFocus = minFocus != null && minFocus > 0 && afModes != null
                    && Arrays.stream(afModes).anyMatch(m -> m == CameraCharacteristics.CONTROL_AF_MODE_OFF);
            Integer tsSource = c.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE);
            boolean realtime = tsSource != null
                    && tsSource == CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME;
            int[] oisModes = c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION);
            boolean ois = oisModes != null && Arrays.stream(oisModes)
                    .anyMatch(m -> m == CameraCharacteristics.LENS_OPTICAL_STABILIZATION_MODE_ON);
            return new CameraSpec(id, jpeg, preview, realtime ? bridge : null, orientation == null ? 90 : orientation,
                    focalPx, manualFocus, realtime, ois);
        }
        throw new IllegalStateException("Không tìm thấy camera sau");
    }

    private static Size largest4x3(Size[] sizes, long maxArea) {
        Size best = null;
        if (sizes == null) return null;
        for (Size s : sizes) {
            long area = (long) s.getWidth() * s.getHeight();
            if (area > maxArea) continue;
            if (Math.abs(s.getWidth() * 3 - s.getHeight() * 4) > s.getWidth() / 50) continue;
            if (best == null || area > (long) best.getWidth() * best.getHeight()) best = s;
        }
        return best;
    }
}
