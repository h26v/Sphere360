package com.sphere360.app.gimbal;

import java.io.ByteArrayOutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Gói tin DUML, giao thức chung giữa các thiết bị DJI (gimbal Osmo Mobile, drone...). Khuôn gói:
 *
 * <pre>
 * 0x55 | độ dài (10 bit) + phiên bản (6 bit), LE | CRC8 của 3 byte đầu
 * bên gửi | bên nhận | số thứ tự (LE) | kiểu lệnh | bộ lệnh | mã lệnh | dữ liệu... | CRC16 của mọi byte trước (LE)
 * </pre>
 *
 * Bên gửi / nhận: loại thiết bị ở 5 bit thấp (2 = app, 4 = gimbal), số hiệu ở 3 bit cao. Kiểu lệnh: bit 7 là gói
 * trả lời, bit 5–6 là kiểu xin trả lời (0 không cần, 2 trả lời sau khi làm xong). Tham khảo: om-research (OM2–OM4)
 * và bộ giải mã DUML của dji-firmware-tools.
 */
public final class Duml {

    static final int APP = 0x02;
    static final int GIMBAL = 0x04;

    static final int SET_GIMBAL = 0x04;
    /** Quay theo góc (tương đối hoặc tuyệt đối) trong một khoảng thời gian. */
    static final int CMD_ANGLE = 0x14;
    /** Quay theo tốc độ. */
    static final int CMD_SPEED = 0x0C;

    public static final int MODE_RELATIVE = 0x04;
    public static final int MODE_ABSOLUTE = 0x05;
    public static final int MODE_SPEED = 0x80;

    public static final class Packet {
        public int sender, receiver, seq, cmdType, cmdSet, cmd;
        public byte[] payload;
        public byte[] raw;

        boolean isReply() {
            return (cmdType & 0x80) != 0;
        }

        @Override
        public String toString() {
            return String.format(Locale.US, "%02x->%02x seq=%d type=%02x %02x/%02x [%d] %s", sender, receiver, seq,
                    cmdType, cmdSet, cmd, payload.length, hex(payload));
        }
    }

    private Duml() {
    }

    static byte[] build(int sender, int receiver, int seq, int cmdType, int cmdSet, int cmd, byte[] payload) {
        int len = 13 + payload.length;
        byte[] p = new byte[len];
        p[0] = 0x55;
        p[1] = (byte) len;
        p[2] = (byte) (((len >> 8) & 0x03) | (1 << 2));  // phiên bản 1
        p[3] = (byte) crc8(p, 0, 3);
        p[4] = (byte) sender;
        p[5] = (byte) receiver;
        p[6] = (byte) seq;
        p[7] = (byte) (seq >> 8);
        p[8] = (byte) cmdType;
        p[9] = (byte) cmdSet;
        p[10] = (byte) cmd;
        System.arraycopy(payload, 0, p, 11, payload.length);
        int crc = crc16(p, 0, len - 2);
        p[len - 2] = (byte) crc;
        p[len - 1] = (byte) (crc >> 8);
        return p;
    }

    /** Dữ liệu lệnh quay gimbal: yaw, roll, pitch (0,1°, hoặc 0,1°/s khi quay theo tốc độ), chế độ, thời gian (0,1 s). */
    static byte[] rotation(double yawDeg, double pitchDeg, double rollDeg, int mode, double seconds) {
        byte[] b = new byte[8];
        put16(b, 0, (int) Math.round(yawDeg * 10));
        put16(b, 2, (int) Math.round(rollDeg * 10));
        put16(b, 4, (int) Math.round(pitchDeg * 10));
        b[6] = (byte) mode;
        b[7] = (byte) Math.max(0, Math.min(255, Math.round(seconds * 10)));
        return b;
    }

    private static void put16(byte[] b, int at, int v) {
        v = Math.max(-32768, Math.min(32767, v));
        b[at] = (byte) v;
        b[at + 1] = (byte) (v >> 8);
    }

    static int get16(byte[] b, int at) {
        return (short) ((b[at] & 0xFF) | (b[at + 1] << 8));
    }

    static int crc8(byte[] d, int off, int len) {
        int c = 0x77;
        for (int i = off; i < off + len; i++) {
            c ^= d[i] & 0xFF;
            for (int k = 0; k < 8; k++) c = (c & 1) != 0 ? (c >>> 1) ^ 0x8C : c >>> 1;
        }
        return c;
    }

    static int crc16(byte[] d, int off, int len) {
        int c = 0x3692;
        for (int i = off; i < off + len; i++) {
            c ^= d[i] & 0xFF;
            for (int k = 0; k < 8; k++) c = (c & 1) != 0 ? (c >>> 1) ^ 0x8408 : c >>> 1;
        }
        return c;
    }

    static String hex(byte[] b) {
        StringBuilder s = new StringBuilder();
        for (int i = 0; i < b.length; i++) {
            if (i > 0) s.append(' ');
            s.append(String.format(Locale.US, "%02x", b[i] & 0xFF));
        }
        return s.toString();
    }

    public static byte[] unhex(String s) {
        String t = s.replaceAll("[^0-9a-fA-F]", "");
        byte[] b = new byte[t.length() / 2];
        for (int i = 0; i < b.length; i++) b[i] = (byte) Integer.parseInt(t.substring(2 * i, 2 * i + 2), 16);
        return b;
    }

    /** Ghép các mẩu notify thành gói. Bỏ qua byte rác và gói sai CRC. */
    static final class Parser {
        private final ByteArrayOutputStream buf = new ByteArrayOutputStream();
        int badCrc;

        List<Packet> feed(byte[] data) {
            buf.write(data, 0, data.length);
            List<Packet> out = new ArrayList<>();
            byte[] b = buf.toByteArray();
            int pos = 0;
            while (true) {
                while (pos < b.length && b[pos] != 0x55) pos++;
                if (b.length - pos < 4) break;
                if (crc8(b, pos, 3) != (b[pos + 3] & 0xFF)) {
                    pos++;
                    continue;
                }
                int len = ((b[pos + 1] & 0xFF) | (b[pos + 2] << 8)) & 0x3FF;
                if (len < 13) {
                    pos++;
                    continue;
                }
                if (b.length - pos < len) break;
                int crc = (b[pos + len - 2] & 0xFF) | ((b[pos + len - 1] & 0xFF) << 8);
                if (crc16(b, pos, len - 2) != crc) {
                    badCrc++;
                    pos++;
                    continue;
                }
                Packet p = new Packet();
                p.raw = java.util.Arrays.copyOfRange(b, pos, pos + len);
                p.sender = b[pos + 4] & 0xFF;
                p.receiver = b[pos + 5] & 0xFF;
                p.seq = (b[pos + 6] & 0xFF) | ((b[pos + 7] & 0xFF) << 8);
                p.cmdType = b[pos + 8] & 0xFF;
                p.cmdSet = b[pos + 9] & 0xFF;
                p.cmd = b[pos + 10] & 0xFF;
                p.payload = java.util.Arrays.copyOfRange(b, pos + 11, pos + len - 2);
                out.add(p);
                pos += len;
            }
            buf.reset();
            buf.write(b, pos, b.length - pos);
            return out;
        }
    }
}
