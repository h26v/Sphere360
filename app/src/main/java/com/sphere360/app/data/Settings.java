package com.sphere360.app.data;

/** Tên tệp và khoá SharedPreferences dùng chung giữa các màn hình. */
public final class Settings {

    public static final String PREFS = "settings";
    public static final String PREF_OUTPUT_WIDTH = "output_width";
    public static final int DEFAULT_OUTPUT_WIDTH = 8192;
    public static final String PREF_DEGHOST = "deghost";
    public static final String PREF_HANDHELD = "handheld";

    private Settings() {}
}
