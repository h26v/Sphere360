package com.sphere360.app.data;

/** Tên tệp và khoá SharedPreferences dùng chung giữa các màn hình. */
public final class Settings {

    public static final String PREFS = "settings";
    public static final String PREF_OUTPUT_WIDTH = "output_width";
    public static final int DEFAULT_OUTPUT_WIDTH = 8192;
    public static final String PREF_DEGHOST = "deghost";
    public static final String PREF_HANDHELD = "handheld";
    /** Chụp thêm tấm thẳng lên (đỉnh trời) / thẳng xuống (mặt đất). Tắt thì bộ ghép tô vùng đó từ xung quanh. */
    public static final String PREF_CAPTURE_ZENITH = "capture_zenith";
    public static final String PREF_CAPTURE_NADIR = "capture_nadir";
    /** Chụp bằng gimbal Osmo Mobile: gimbal tự quay tới từng góc. */
    public static final String PREF_GIMBAL = "capture_gimbal";

    private Settings() {}
}
