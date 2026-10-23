package com.sphere360.app.viewer;

import android.graphics.Bitmap;
import android.opengl.GLES30;
import android.opengl.GLSurfaceView;
import android.opengl.GLUtils;
import android.util.Log;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/**
 * Vẽ ảnh cầu equirectangular: mỗi điểm ảnh trên màn hình tính hướng nhìn rồi tra màu trong ảnh,
 * nên không có lưới tam giác, không méo và không lộ đường nối.
 */
final class PanoRenderer implements GLSurfaceView.Renderer {

    interface Listener {
        void onMaxTextureSize(int size);
    }

    private static final String TAG = "PanoRenderer";
    private static final int GL_TEXTURE_MAX_ANISOTROPY_EXT = 0x84FE;

    private static final String VERTEX =
            "#version 300 es\n" +
            "in vec2 aPos;\n" +
            "out vec2 vNdc;\n" +
            "void main() { vNdc = aPos; gl_Position = vec4(aPos, 0.0, 1.0); }\n";

    // Hệ toàn cảnh giống lúc ghép: x phải, y xuống, z hướng nhìn ban đầu.
    private static final String FRAGMENT =
            "#version 300 es\n" +
            "precision highp float;\n" +
            "in vec2 vNdc;\n" +
            "uniform mat3 uRot;\n" +
            "uniform vec2 uTan;\n" +
            "uniform sampler2D uTex;\n" +
            "out vec4 outColor;\n" +
            "const float PI = 3.14159265358979;\n" +
            "void main() {\n" +
            "  vec3 d = normalize(uRot * vec3(vNdc.x * uTan.x, -vNdc.y * uTan.y, 1.0));\n" +
            "  vec2 uv = vec2(atan(d.x, d.z) / (2.0 * PI) + 0.5, 1.0 - acos(clamp(d.y, -1.0, 1.0)) / PI);\n" +
            // Chọn đạo hàm không bị nhảy ở kinh tuyến 180 độ để mipmap không vẽ ra một đường kẻ.
            "  vec2 uv2 = vec2(fract(uv.x + 0.5) - 0.5, uv.y);\n" +
            "  vec2 dx = dFdx(uv), dy = dFdy(uv);\n" +
            "  vec2 dx2 = dFdx(uv2), dy2 = dFdy(uv2);\n" +
            "  if (abs(dx2.x) + abs(dy2.x) < abs(dx.x) + abs(dy.x)) { dx = dx2; dy = dy2; }\n" +
            "  outColor = textureGrad(uTex, uv, dx, dy);\n" +
            "}\n";

    private final Listener listener;
    private final FloatBuffer quad;
    private int program;
    private int aPos, uRot, uTan;
    private int texture;
    private int width = 1, height = 1;

    private Bitmap pending;
    private final float[] rotation = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // theo hàng, camera -> toàn cảnh
    private float vFovDeg = 75;

    PanoRenderer(Listener listener) {
        this.listener = listener;
        float[] v = {-1, -1, 1, -1, -1, 1, 1, 1};
        quad = ByteBuffer.allocateDirect(v.length * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        quad.put(v).position(0);
    }

    /** Gọi từ luồng GL (queueEvent). */
    void setBitmap(Bitmap bmp) {
        pending = bmp;
    }

    synchronized void setView(float[] cameraToPano, float vFovDeg) {
        System.arraycopy(cameraToPano, 0, rotation, 0, 9);
        this.vFovDeg = vFovDeg;
    }

    float aspect() {
        return width / (float) height;
    }

    @Override
    public void onSurfaceCreated(GL10 unused, EGLConfig config) {
        program = link(VERTEX, FRAGMENT);
        aPos = GLES30.glGetAttribLocation(program, "aPos");
        uRot = GLES30.glGetUniformLocation(program, "uRot");
        uTan = GLES30.glGetUniformLocation(program, "uTan");
        int[] max = new int[1];
        GLES30.glGetIntegerv(GLES30.GL_MAX_TEXTURE_SIZE, max, 0);
        Log.i(TAG, "GL_MAX_TEXTURE_SIZE " + max[0]);
        texture = 0;
        listener.onMaxTextureSize(max[0]);
    }

    @Override
    public void onSurfaceChanged(GL10 unused, int w, int h) {
        width = Math.max(1, w);
        height = Math.max(1, h);
        GLES30.glViewport(0, 0, width, height);
    }

    @Override
    public void onDrawFrame(GL10 unused) {
        if (pending != null) upload();
        GLES30.glClearColor(0, 0, 0, 1);
        GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT);
        if (texture == 0) return;

        float[] r = new float[9];
        float fov;
        synchronized (this) {
            System.arraycopy(rotation, 0, r, 0, 9);
            fov = vFovDeg;
        }
        float tanV = (float) Math.tan(Math.toRadians(fov / 2));
        float tanH = tanV * width / height;

        GLES30.glUseProgram(program);
        // OpenGL đọc mat3 theo cột: truyền transpose = true cho ma trận lưu theo hàng.
        GLES30.glUniformMatrix3fv(uRot, 1, true, r, 0);
        GLES30.glUniform2f(uTan, tanH, tanV);
        GLES30.glActiveTexture(GLES30.GL_TEXTURE0);
        GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, texture);
        GLES30.glEnableVertexAttribArray(aPos);
        GLES30.glVertexAttribPointer(aPos, 2, GLES30.GL_FLOAT, false, 0, quad);
        GLES30.glDrawArrays(GLES30.GL_TRIANGLE_STRIP, 0, 4);
    }

    private void upload() {
        Bitmap bmp = pending;
        pending = null;
        if (texture != 0) GLES30.glDeleteTextures(1, new int[]{texture}, 0);
        int[] t = new int[1];
        GLES30.glGenTextures(1, t, 0);
        texture = t[0];
        GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, texture);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MIN_FILTER, GLES30.GL_LINEAR_MIPMAP_LINEAR);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MAG_FILTER, GLES30.GL_LINEAR);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_S, GLES30.GL_REPEAT);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_T, GLES30.GL_CLAMP_TO_EDGE);
        String ext = GLES30.glGetString(GLES30.GL_EXTENSIONS);
        if (ext != null && ext.contains("GL_EXT_texture_filter_anisotropic")) {
            GLES30.glTexParameterf(GLES30.GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 8f);
        }
        GLUtils.texImage2D(GLES30.GL_TEXTURE_2D, 0, bmp, 0);
        GLES30.glGenerateMipmap(GLES30.GL_TEXTURE_2D);
        bmp.recycle();
    }

    private static int link(String vs, String fs) {
        int v = compile(GLES30.GL_VERTEX_SHADER, vs);
        int f = compile(GLES30.GL_FRAGMENT_SHADER, fs);
        int p = GLES30.glCreateProgram();
        GLES30.glAttachShader(p, v);
        GLES30.glAttachShader(p, f);
        GLES30.glLinkProgram(p);
        int[] ok = new int[1];
        GLES30.glGetProgramiv(p, GLES30.GL_LINK_STATUS, ok, 0);
        if (ok[0] == 0) throw new RuntimeException("Link shader lỗi: " + GLES30.glGetProgramInfoLog(p));
        return p;
    }

    private static int compile(int type, String src) {
        int s = GLES30.glCreateShader(type);
        GLES30.glShaderSource(s, src);
        GLES30.glCompileShader(s);
        int[] ok = new int[1];
        GLES30.glGetShaderiv(s, GLES30.GL_COMPILE_STATUS, ok, 0);
        if (ok[0] == 0) throw new RuntimeException("Shader lỗi: " + GLES30.glGetShaderInfoLog(s));
        return s;
    }
}
