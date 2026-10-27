package com.sphere360.app.gimbal;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanRecord;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.os.Build;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.ParcelUuid;

import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.UUID;

/**
 * Điều khiển gimbal Osmo Mobile qua Bluetooth LE bằng gói DUML (xem {@link Duml}): dịch vụ FFF0, ghi lệnh vào
 * FFF5, gimbal báo về qua notify FFF4. Giao thức theo om-research (đã chạy trên OM2–OM4); với OM7 chưa ai xác
 * nhận nên lớp này ghi log kỹ mọi thứ nhận được. Mọi thao tác Bluetooth chạy tuần tự trên một luồng riêng.
 * Cần quyền BLUETOOTH_SCAN và BLUETOOTH_CONNECT (Android 12+) trước khi gọi.
 */
@SuppressLint("MissingPermission")
public final class OsmoGimbal {

    public interface Listener {
        void onLog(String line);

        void onState(String state, boolean ready);

        /** Gói DUML gimbal gửi về (trên luồng Bluetooth). */
        void onPacket(Duml.Packet p);
    }

    static final UUID SERVICE = UUID.fromString("0000fff0-0000-1000-8000-00805f9b34fb");
    static final UUID NOTIFY = UUID.fromString("0000fff4-0000-1000-8000-00805f9b34fb");
    static final UUID WRITE = UUID.fromString("0000fff5-0000-1000-8000-00805f9b34fb");
    static final UUID CCCD = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");

    private static final long SCAN_MS = 15000;

    private final Context ctx;
    private final Listener listener;
    private final HandlerThread thread = new HandlerThread("gimbal");
    private final Handler h;

    private BluetoothLeScanner scanner;
    private ScanCallback scanCb;
    private BluetoothGatt gatt;
    private BluetoothGattCharacteristic writeChar;
    private final ArrayDeque<byte[]> chunks = new ArrayDeque<>();
    private boolean writing;
    private int mtu = 23;
    private int seq = 1;
    private boolean ready;
    private final Duml.Parser parser = new Duml.Parser();
    private final Map<String, Integer> seen = new HashMap<>();
    private long lastStateLog;
    /** Ghi cả lệnh tốc độ (vòng điều khiển gửi 10 lần mỗi giây nên mặc định không ghi). */
    public volatile boolean verbose;

    public OsmoGimbal(Context ctx, Listener listener) {
        this.ctx = ctx.getApplicationContext();
        this.listener = listener;
        thread.start();
        h = new Handler(thread.getLooper());
    }

    public boolean isReady() {
        return ready;
    }

    /** Tìm gimbal (hoặc đúng địa chỉ address nếu có) rồi kết nối. */
    public void connect(String address) {
        h.post(() -> doConnect(address));
    }

    public void close() {
        h.post(() -> {
            stopScan();
            if (gatt != null) {
                gatt.disconnect();
                gatt.close();
                gatt = null;
            }
            ready = false;
            listener.onState("Đã ngắt", false);
        });
        thread.quitSafely();
    }

    /** Quay theo góc trong seconds giây. mode: Duml.MODE_RELATIVE hoặc Duml.MODE_ABSOLUTE. */
    public void rotate(double yawDeg, double pitchDeg, double rollDeg, int mode, double seconds, int cmdType) {
        send(cmdType, Duml.SET_GIMBAL, Duml.CMD_ANGLE, Duml.rotation(yawDeg, pitchDeg, rollDeg, mode, seconds));
    }

    /** Quay theo tốc độ (°/s) trong seconds giây. */
    public void speed(double yawDps, double pitchDps, double rollDps, double seconds, int cmdType) {
        send(cmdType, Duml.SET_GIMBAL, Duml.CMD_SPEED, Duml.rotation(yawDps, pitchDps, rollDps, Duml.MODE_SPEED, seconds));
    }

    /** Gửi một gói DUML bất kỳ từ app tới gimbal. */
    public void send(int cmdType, int cmdSet, int cmd, byte[] payload) {
        send(Duml.GIMBAL, cmdType, cmdSet, cmd, payload);
    }

    /** Như trên, gửi tới bên nhận receiver (loại thiết bị | số hiệu << 5). */
    public void send(int receiver, int cmdType, int cmdSet, int cmd, byte[] payload) {
        h.post(() -> {
            if (!ready) {
                log("Chưa kết nối, bỏ qua lệnh");
                return;
            }
            byte[] p = Duml.build(Duml.APP, receiver, seq++ & 0xFFFF, cmdType, cmdSet, cmd, payload);
            if (cmd != Duml.CMD_SPEED || verbose)
                log("→ " + String.format(Locale.US, "%02x %02x/%02x ", receiver, cmdSet, cmd) + Duml.hex(p));
            int size = Math.max(20, mtu - 3);
            for (int i = 0; i < p.length; i += size)
                chunks.add(java.util.Arrays.copyOfRange(p, i, Math.min(p.length, i + size)));
            pump();
        });
    }

    private void log(String s) {
        listener.onLog(s);
    }

    private void doConnect(String address) {
        BluetoothManager bm = (BluetoothManager) ctx.getSystemService(Context.BLUETOOTH_SERVICE);
        BluetoothAdapter adapter = bm != null ? bm.getAdapter() : null;
        if (adapter == null || !adapter.isEnabled()) {
            listener.onState("Bluetooth đang tắt", false);
            return;
        }
        if (gatt != null) {
            gatt.close();
            gatt = null;
        }
        ready = false;
        writeChar = null;
        if (address != null && !address.isEmpty()) {
            openGatt(adapter.getRemoteDevice(address));
            return;
        }
        scanner = adapter.getBluetoothLeScanner();
        if (scanner == null) {
            listener.onState("Không quét được Bluetooth", false);
            return;
        }
        listener.onState("Đang tìm gimbal...", false);
        Set<String> logged = new HashSet<>();
        scanCb = new ScanCallback() {
            @Override
            public void onScanResult(int type, ScanResult r) {
                h.post(() -> onFound(r, logged));
            }

            @Override
            public void onScanFailed(int code) {
                h.post(() -> listener.onState("Quét lỗi " + code, false));
            }
        };
        scanner.startScan(null, new ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build(), scanCb);
        h.postDelayed(() -> {
            if (scanCb != null) {
                stopScan();
                listener.onState("Không thấy gimbal (bật gimbal, tắt DJI Mimo rồi thử lại)", false);
            }
        }, SCAN_MS);
    }

    private void onFound(ScanResult r, Set<String> logged) {
        if (scanCb == null) return;
        BluetoothDevice d = r.getDevice();
        ScanRecord rec = r.getScanRecord();
        String name = rec != null && rec.getDeviceName() != null ? rec.getDeviceName() : d.getName();
        boolean fff0 = false;
        if (rec != null && rec.getServiceUuids() != null)
            for (ParcelUuid u : rec.getServiceUuids()) fff0 |= SERVICE.equals(u.getUuid());
        boolean dji = rec != null && rec.getManufacturerSpecificData(0x08AA) != null;  // mã hãng của DJI
        boolean named = name != null && name.matches("(?i).*(osmo|\\bom ?\\d).*");
        if (logged.add(d.getAddress())) {
            StringBuilder extra = new StringBuilder();
            if (rec != null && rec.getServiceUuids() != null)
                for (ParcelUuid u : rec.getServiceUuids()) extra.append(' ').append(shortUuid(u.getUuid()));
            if (rec != null && rec.getManufacturerSpecificData() != null)
                for (int i = 0; i < rec.getManufacturerSpecificData().size(); i++)
                    extra.append(String.format(Locale.US, " hãng %04x:%s", rec.getManufacturerSpecificData().keyAt(i),
                            Duml.hex(rec.getManufacturerSpecificData().valueAt(i))));
            log(String.format(Locale.US, "thấy %s \"%s\" rssi %d%s", d.getAddress(), name, r.getRssi(), extra));
        }
        if (fff0 || dji || named) {
            stopScan();
            openGatt(d);
        }
    }

    private void stopScan() {
        if (scanner != null && scanCb != null) {
            try {
                scanner.stopScan(scanCb);
            } catch (Exception ignored) {
            }
        }
        scanCb = null;
    }

    private void openGatt(BluetoothDevice d) {
        listener.onState("Đang kết nối " + d.getAddress() + "...", false);
        gatt = d.connectGatt(ctx, false, cb, BluetoothDevice.TRANSPORT_LE);
    }

    private void pump() {
        if (writing || chunks.isEmpty() || gatt == null || writeChar == null) return;
        byte[] c = chunks.poll();
        int type = (writeChar.getProperties() & BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE) != 0
                ? BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE : BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT;
        boolean ok;
        if (Build.VERSION.SDK_INT >= 33) {
            ok = gatt.writeCharacteristic(writeChar, c, type) == BluetoothGatt.GATT_SUCCESS;
        } else {
            writeChar.setWriteType(type);
            writeChar.setValue(c);
            ok = gatt.writeCharacteristic(writeChar);
        }
        if (!ok) {
            log("Ghi lỗi, bỏ " + (chunks.size() + 1) + " mẩu");
            chunks.clear();
            return;
        }
        writing = true;
    }

    private final BluetoothGattCallback cb = new BluetoothGattCallback() {
        @Override
        public void onConnectionStateChange(BluetoothGatt g, int status, int state) {
            h.post(() -> {
                if (state == BluetoothProfile.STATE_CONNECTED) {
                    log("Đã nối (status " + status + "), xin MTU 247");
                    if (!g.requestMtu(247)) g.discoverServices();
                } else if (state == BluetoothProfile.STATE_DISCONNECTED) {
                    ready = false;
                    writing = false;
                    chunks.clear();
                    listener.onState("Mất kết nối (status " + status + ")", false);
                }
            });
        }

        @Override
        public void onMtuChanged(BluetoothGatt g, int newMtu, int status) {
            h.post(() -> {
                if (status == BluetoothGatt.GATT_SUCCESS) mtu = newMtu;
                log("MTU " + mtu);
                g.discoverServices();
            });
        }

        @Override
        public void onServicesDiscovered(BluetoothGatt g, int status) {
            h.post(() -> {
                BluetoothGattCharacteristic notify = null;
                for (BluetoothGattService s : g.getServices()) {
                    log("dịch vụ " + shortUuid(s.getUuid()));
                    for (BluetoothGattCharacteristic c : s.getCharacteristics())
                        log("   " + shortUuid(c.getUuid()) + " " + props(c.getProperties()));
                }
                BluetoothGattService s = g.getService(SERVICE);
                if (s != null) {
                    writeChar = s.getCharacteristic(WRITE);
                    notify = s.getCharacteristic(NOTIFY);
                }
                if (writeChar == null || notify == null) {
                    listener.onState("Không có dịch vụ FFF0/FFF4/FFF5: gimbal này dùng giao thức khác", false);
                    return;
                }
                g.setCharacteristicNotification(notify, true);
                BluetoothGattDescriptor d = notify.getDescriptor(CCCD);
                if (d == null) {
                    onReady();
                    return;
                }
                byte[] v = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE;
                if (Build.VERSION.SDK_INT >= 33) {
                    g.writeDescriptor(d, v);
                } else {
                    d.setValue(v);
                    g.writeDescriptor(d);
                }
            });
        }

        @Override
        public void onDescriptorWrite(BluetoothGatt g, BluetoothGattDescriptor d, int status) {
            h.post(() -> {
                log("Bật notify FFF4: status " + status);
                onReady();
            });
        }

        @Override
        public void onCharacteristicWrite(BluetoothGatt g, BluetoothGattCharacteristic c, int status) {
            h.post(() -> {
                if (status != BluetoothGatt.GATT_SUCCESS) log("Ghi mẩu lỗi status " + status);
                writing = false;
                pump();
            });
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic c, byte[] value) {
            byte[] copy = value.clone();
            h.post(() -> onNotify(copy));
        }

        @Override
        @SuppressWarnings("deprecation")
        public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic c) {
            byte[] v = c.getValue();
            if (v != null) {
                byte[] copy = v.clone();
                h.post(() -> onNotify(copy));
            }
        }
    };

    private void onReady() {
        ready = true;
        listener.onState("Đã sẵn sàng", true);
        pump();
    }

    private void onNotify(byte[] data) {
        for (Duml.Packet p : parser.feed(data)) {
            String key = String.format(Locale.US, "%02x/%02x%s", p.cmdSet, p.cmd, p.isReply() ? "r" : "");
            Integer n = seen.get(key);
            seen.put(key, n == null ? 1 : n + 1);
            // Gói đẩy định kỳ chỉ log vài cái đầu; trả lời lệnh thì log hết.
            if (n == null || n < 3 || p.isReply()) log("← " + p);
            if (p.cmdSet == 0x04 && p.cmd == 0x05 && p.payload.length >= 40
                    && android.os.SystemClock.elapsedRealtime() - lastStateLog > 400) {
                lastStateLog = android.os.SystemClock.elapsedRealtime();
                java.nio.ByteBuffer bb = java.nio.ByteBuffer.wrap(p.payload).order(java.nio.ByteOrder.LITTLE_ENDIAN);
                log(String.format(Locale.US, "trạng thái i16 %d %d %d %d | q %.4f %.4f %.4f %.4f", bb.getShort(0),
                        bb.getShort(2), bb.getShort(4), bb.getShort(8), bb.getFloat(24), bb.getFloat(28),
                        bb.getFloat(32), bb.getFloat(36)) + " | i16[10] " + bb.getShort(10));
            }
            listener.onPacket(p);
        }
    }

    /** Số gói đã nhận theo từng loại (bộ lệnh/mã lệnh, "r" là gói trả lời). */
    public String stats() {
        StringBuilder s = new StringBuilder();
        for (Map.Entry<String, Integer> e : seen.entrySet())
            s.append(e.getKey()).append('×').append(e.getValue()).append("  ");
        if (parser.badCrc > 0) s.append("sai CRC×").append(parser.badCrc);
        return s.toString().trim();
    }

    private static String shortUuid(UUID u) {
        String s = u.toString();
        return s.endsWith("-0000-1000-8000-00805f9b34fb") && s.startsWith("0000") ? s.substring(4, 8) : s;
    }

    private static String props(int p) {
        StringBuilder s = new StringBuilder();
        if ((p & BluetoothGattCharacteristic.PROPERTY_READ) != 0) s.append("read ");
        if ((p & BluetoothGattCharacteristic.PROPERTY_WRITE) != 0) s.append("write ");
        if ((p & BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE) != 0) s.append("writeNR ");
        if ((p & BluetoothGattCharacteristic.PROPERTY_NOTIFY) != 0) s.append("notify ");
        if ((p & BluetoothGattCharacteristic.PROPERTY_INDICATE) != 0) s.append("indicate ");
        return s.toString().trim();
    }

    /** Thử đọc góc từ gói đẩy trạng thái gimbal (04/05, đoán theo drone DJI): {yaw, pitch, roll} độ. Null nếu không phải. */
    public static float[] attitude(Duml.Packet p) {
        if (p.cmdSet != Duml.SET_GIMBAL || p.cmd != 0x05 || p.payload.length < 6) return null;
        return new float[]{Duml.get16(p.payload, 4) / 10f, Duml.get16(p.payload, 0) / 10f, Duml.get16(p.payload, 2) / 10f};
    }
}
