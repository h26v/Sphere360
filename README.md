# Sphere360

Ứng dụng Android chụp **ảnh cầu 360° đủ cả trời và đất** ở độ phân giải gốc của camera, ghép ngay trên máy
và xem lại bằng trình xem 360° trong app. Ảnh xuất là equirectangular 2:1 có metadata GPano, nên Google
Photos, Facebook… nhận ra là ảnh 360°.

## Tính năng

- **Chụp có hướng dẫn**: chấm vàng chỉ hướng tấm tiếp theo, bản đồ các tấm theo 5 hàng (đỉnh, trên, ngang,
  dưới, đáy). App tự chụp khi máy ngắm đúng, đủ thẳng và đứng yên.
- **Chống rung**: ngưỡng rung tính theo tốc trập thật, bật OIS; sau mỗi tấm đo độ nhòe lúc phơi sáng bằng
  con quay, nhòe quá thì bỏ tấm và chụp lại.
- **Khoá sáng, cân bằng trắng và lấy nét** theo tấm đầu tiên để các tấm cùng tông và cùng tiêu cự.
- **Hai cách chụp**: kê máy cố định (27 tấm) hoặc cầm tay xoay người (34 tấm, chồng nhiều hơn).
- **Tuỳ chọn chụp đỉnh/đáy**: tắt được tấm thẳng lên hoặc thẳng xuống (vướng chân máy, chân người, hay lệch nhất
  khi cầm tay). Bộ ghép chiếu vùng quanh cực ra mặt phẳng tiếp xúc rồi tô mượt từ xung quanh (push-pull).
- **Khung cầu nối**: trong lúc lia máy giữa hai tấm, app giữ thêm ảnh xám nhỏ (luồng YUV ~1440×1080) mỗi 2°
  kèm hướng máy, để bộ ghép đo được độ lệch lớn của vật gần giữa hai tấm.
- **Ghép trên máy** bằng bộ ghép C++/OpenCV riêng: căn góc theo điểm đặc trưng có neo trọng lực, uốn ảnh theo
  optical flow ở vùng chồng để giảm bóng mờ do thị sai, đường nối graph cut né chỗ thị sai lớn, trộn nhiều dải tần.
- **Kích thước xuất** 8K, 12K, 16K hoặc tối đa theo camera (1 pixel gốc = 1 pixel ảnh cầu).
- **Trình xem 360°**: kéo, chụm để phóng to, hoặc xoay theo con quay. Mở được ảnh JPEG từ app khác.

## Cấu trúc mã

```
app/src/main/java/com/sphere360/app/
  home/      MainActivity        trang chính, thư viện các lượt chụp, cài đặt
  capture/   CaptureActivity     màn hình chụp và tự chụp
             CameraSpec          chọn camera sau, cỡ JPEG, tiêu cự theo pixel
             CapturePlan         danh sách hướng cần chụp để phủ kín mặt cầu
             GuideOverlayView    chấm mục tiêu, vòng ngắm, thanh rung
             RowProgressView     bản đồ các tấm theo hàng
             MotionLog           lịch sử con quay và hướng máy để đo nhòe lúc phơi sáng
             BridgeRecorder      giữ khung cầu nối từ luồng YUV khi đang lia máy
  stitch/    StitchActivity      chạy bộ ghép, hiện tiến độ, lưu vào Thư viện ảnh
             NativeStitcher      cầu nối JNI tới bộ ghép C++
             StitchInput         một lượt chụp (tấm + khung cầu nối) ở dạng mảng cho JNI
             GPanoWriter         chèn metadata 360° (XMP GPano) vào JPEG
  viewer/    ViewerActivity      trình xem 360° (OpenGL ES 3)
             PanoRenderer
  data/      Session             một lượt chụp: ảnh gốc + session.json
             Settings            khoá SharedPreferences
  ui/        Ui                  màu và thành phần giao diện dùng chung
  util/      Rot                 phép tính ma trận xoay 3x3
  probe/     ArProbeActivity     thử nghiệm ARCore (vị trí 6 bậc tự do, Depth API)
             GimbalProbeActivity thử gimbal Osmo Mobile; GimbalMapper dò tầm với các khớp ở một tư thế
  debug/     DebugActivity       lệnh debug qua adb (xem dưới)
  gimbal/    OsmoGimbal          điều khiển gimbal Osmo Mobile qua Bluetooth LE (dịch vụ FFF0, gói DUML)
             Duml                dựng và đọc gói DUML (CRC8, CRC16)
             GimbalAim           vòng kín: đưa máy tới hướng cho trước bằng lệnh tốc độ và con quay của máy

app/src/main/cpp/                bộ ghép ảnh cầu (C++17, OpenCV)
  stitcher      toàn bộ quy trình ghép (không phụ thuộc JNI)
  alignment     điểm khớp từng cặp (RANSAC phép xoay) và tinh chỉnh góc mọi tấm (Levenberg–Marquardt)
  warp          nắn ảnh lên khung equirect, trường uốn
  flow          optical flow (DIS) ở vùng chồng -> trường uốn giảm bóng mờ
  epipolar      mô hình epipolar khi tâm ống kính dịch đi (chụp cầm tay)
  bridge        dò flow giữa hai tấm liền nhau qua chuỗi khung cầu nối
  seams         graph cut biết thị sai, giao vùng đỉnh/đáy cho tấm chụp thẳng lên/xuống
  quality       đo độ lệch còn lại sau uốn (thước đo khi chỉnh thuật toán, bật bằng kDebugResidual)
  geometry      phép xoay, Kabsch, ma trận nội tại
  params.h      các tham số của bộ ghép (params.cpp: đổi theo tên lúc chạy, cho lệnh debug)
  log           log ra logcat và ra file
  jni_bridge    hàm JNI cho NativeStitcher.stitch()

tools/          script Python kiểm tra ảnh cầu trên máy tính (dựng các góc nhìn phối cảnh, đo tiêu cự)
```

### Quy trình ghép

1. Tìm điểm SIFT ở ảnh ~1 MP, chỉ so khớp các cặp nằm cạnh nhau theo cảm biến.
2. Mỗi cặp: RANSAC 2 điểm tìm phép xoay tương đối, bỏ cặp lệch xa cảm biến.
3. Trung bình phép xoay có chống nhiễu (Cauchy): mỗi cặp ảnh và mỗi chuỗi khung cầu nối cho một phép đo góc
   tương đối, con quay neo góc giữa hai tấm chụp liền nhau; neo chặt theo trọng lực, lỏng theo hướng ngang.
4. Optical flow ở từng vùng chồng. Hai tấm chụp liền nhau có khung cầu nối thì flow được dò qua chuỗi khung
   (mỗi bước lệch vài độ) rồi tinh chỉnh thẳng giữa hai tấm. Cặp nào khớp mô hình epipolar (máy dịch đi khi
   chụp) thì flow bị ép dọc đường epipolar.
5. Trường uốn chung: mỗi ảnh có một lưới độ dịch (ô 1°); mọi điểm flow của mọi cặp cùng ràng buộc
   d_q − d_p = F, cộng độ mượt và lực kéo nhẹ về 0, giải một lần bằng conjugate gradient. Chỗ ba bốn ảnh
   chồng nhau vẫn khớp được với nhau (cách cũ: mỗi cặp đẩy hai ảnh đi nửa đường rồi lấy trung bình). Lượt
   thứ hai uốn ảnh theo trường vừa giải, đo flow còn lại và giải phần bù.
6. Cân bằng sáng theo từng ảnh và kênh màu. Đường nối graph cut có thêm chi phí theo độ lệch còn lại sau khi
   uốn (đo lại flow trên ảnh đã uốn), nên đi qua chỗ đã khớp, né chỗ còn gãy. Giao đỉnh/đáy cho tấm chụp thẳng
   lên/xuống, nối vòng ở kinh tuyến 180°.
7. Dựng ở độ phân giải đầy đủ, trộn nhiều dải tần (≤ 8K) hoặc trộn mép (lớn hơn).

## Build

Cần:

- JDK 17 trở lên
- Android SDK 36, NDK `28.2.13676358`, CMake `4.1.2`
- [OpenCV Android SDK](https://opencv.org/releases/) 4.x (đã dùng 4.14.0), bản có `sdk/native/staticlibs`

Tạo `local.properties` ở thư mục gốc (không đưa lên git):

```properties
sdk.dir=C:/duong/dan/android-sdk
opencv.sdk=C:/duong/dan/OpenCV-android-sdk
```

Có thể đặt biến môi trường `OPENCV_ANDROID_SDK` thay cho `opencv.sdk`. Trên Windows dùng dấu `/` trong
đường dẫn.

```bash
./gradlew assembleRelease
adb install -r app/build/outputs/apk/release/app-release.apk
```

Bản release được ký bằng khoá debug để cài thẳng lên máy. Chỉ build cho `arm64-v8a`; minSdk 29.

### Build tự động (GitHub Actions)

`.github/workflows/build.yml` build APK release sau mỗi lần push (mọi nhánh, tag `v*`) và cho pull request:
cài NDK/CMake, tải OpenCV Android SDK (có cache), rồi `./gradlew assembleRelease`. APK nằm ở mục **Artifacts**
của lần chạy, giữ 30 ngày. Máy CI tự tạo khoá debug riêng nên APK không cài đè được lên app đã cài từ máy khác;
muốn cùng chữ ký thì lưu `~/.android/debug.keystore` (base64) vào secret `DEBUG_KEYSTORE_BASE64`.

## Lệnh debug qua adb

`debug.DebugActivity` chỉ adb gọi được (đòi quyền `DUMP`). Ghép một lượt mà không đụng tới `pano.jpg`,
`session.json` hay Thư viện ảnh, và đổi được tham số trong `params.h` theo tên:

```bash
adb shell am start -n com.sphere360.app/.debug.DebugActivity --es cmd stitch --es session 20261023_171607 --es out t1.jpg --ei width 4096 --es params "'kSeamParallaxWeight=4,kDebugSeams=1'"
```

Kết quả nằm trong thư mục lượt chụp: `t1.jpg`, `t1.log` (log bộ ghép), và khi `kDebugSeams=1` thêm
`t1_seams.jpg` (đường nối và bản đồ thị sai) cùng `t1_chain_<p>_<q>.jpg` (điểm dò qua chuỗi khung cầu nối: xanh
còn, đỏ mất), cuối cùng `t1.done` (JSON: ok, error, ms). `kDebugResidual=1` ghi thêm vào log độ lệch còn lại
sau uốn (cả vùng chồng và riêng trên đường nối) và các cặp lệch nhiều nhất; kèm `kDebugSeams=1` thì có thêm
`t1_res_<p>_<q>.jpg` (hai ảnh đã uốn và bản đồ lệch). `kDebugDump=1` (kèm `kDebugSeams=1`) ghi phép xoay cuối
cùng `t1_cams.txt` và trường uốn `t1_fields.bin` (~200 MB) để phân tích trên máy tính. Thêm `kUseBridges=0` để ghép không dùng khung cầu nối. `--es cmd params` ghi danh sách
tham số ra `files/params.txt`. Script `tools/dbg.sh` gói các lệnh này (cài APK, ghép, kéo kết quả về).

Thử gimbal (`probe.GimbalProbeActivity`, cũng chỉ adb mở được; log thẻ `GimbalProbe` và
`files/gimbal/probe_*.txt`). OM7 bỏ qua lệnh quay theo góc (`04/14`) mà OM2–OM4 dùng nhưng nhận lệnh tốc độ
(`04/0C`), nên mọi lệnh quay đi qua vòng kín `GimbalAim`. `map` dò tầm với ở tư thế đang đặt, ghi
`files/gimbal/map_*.csv` cho `tools/gimbal_cover.py`:

```bash
adb shell am start -n com.sphere360.app/.probe.GimbalProbeActivity --es do connect
adb shell am start -n com.sphere360.app/.probe.GimbalProbeActivity --es do goto --ef dyaw 40 --ef dpitch 0
adb shell am start -n com.sphere360.app/.probe.GimbalProbeActivity --es do map
```

## Mẹo chụp

- Ống kính càng đứng yên một chỗ thì càng ít lệch: tốt nhất là xoay máy quanh một chỗ kê (hộp, gậy, chân máy).
- Tắt quạt, chờ không có người đi qua: vật chuyển động sẽ bị nhòe hoặc cắt đôi.
- Tấm đầu tiên hướng vào chỗ cách 2–3 m vì app khoá nét, sáng và màu theo tấm này.

## Giới hạn đã biết

- Vật rất gần máy (dưới ~1 m) khi chụp cầm tay vẫn có thể gãy hoặc nhân đôi ở chỗ nối: optical flow không
  bắt được độ lệch trên ~6–10°. Khung cầu nối chỉ có giữa hai tấm chụp liền nhau (cùng hàng), không có cho
  đường nối giữa hai hàng.
- Đáy ảnh thấy chân người chụp hoặc chỗ kê (tắt chụp mặt đất thì vùng này được tô mờ thay vào).
- Trình xem trong app giới hạn ảnh ở 8192 px chiều ngang.
- Việc ghép chạy trong màn hình ghép, chưa chạy nền.

## Công cụ kiểm tra (`tools/`)

Cần Python với `opencv-python` và `numpy`.

```bash
python tools/grid_views.py pano.jpg out      # out_eq.jpg, out_grid.jpg (12 góc nhìn), out_poles.jpg
python tools/detail_views.py pano.jpg out.jpg 0,30,80 90,0,70   # các góc yaw,pitch,fov tuỳ chọn
python tools/focal_probe.py <thư mục lượt chụp>                 # đo tiêu cự thật từ các cặp ảnh kề
python tools/gimbal_cover.py map_20261027_101412.csv            # độ phủ mặt cầu của một tư thế gimbal
python tools/gimbal_sweet.py                                    # mô hình: độ phủ theo độ nghiêng tay cầm
```
