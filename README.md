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
- **Ghép trên máy** bằng bộ ghép C++/OpenCV riêng: căn góc theo điểm đặc trưng có neo trọng lực, uốn ảnh theo
  optical flow ở vùng chồng để giảm bóng mờ do thị sai, đường nối graph cut, trộn nhiều dải tần.
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
  stitch/    StitchActivity      chạy bộ ghép, hiện tiến độ, lưu vào Thư viện ảnh
             NativeStitcher      cầu nối JNI tới bộ ghép C++
             GPanoWriter         chèn metadata 360° (XMP GPano) vào JPEG
  viewer/    ViewerActivity      trình xem 360° (OpenGL ES 3)
             PanoRenderer
  data/      Session             một lượt chụp: ảnh gốc + session.json
             Settings            khoá SharedPreferences
  ui/        Ui                  màu và thành phần giao diện dùng chung
  util/      Rot                 phép tính ma trận xoay 3x3
  probe/     ArProbeActivity     thử nghiệm ARCore (vị trí 6 bậc tự do, Depth API)

app/src/main/cpp/                bộ ghép ảnh cầu (C++17, OpenCV)
  stitcher      toàn bộ quy trình ghép (không phụ thuộc JNI)
  alignment     điểm khớp từng cặp (RANSAC phép xoay) và tinh chỉnh góc mọi tấm (Levenberg–Marquardt)
  warp          nắn ảnh lên khung equirect, trường uốn
  flow          optical flow (DIS) ở vùng chồng -> trường uốn giảm bóng mờ
  epipolar      mô hình epipolar khi tâm ống kính dịch đi (chụp cầm tay)
  seams         giao vùng đỉnh/đáy cho tấm chụp thẳng lên/xuống
  geometry      phép xoay, Kabsch, ma trận nội tại
  params.h      các tham số của bộ ghép
  jni_bridge    hàm JNI cho NativeStitcher.stitch()

tools/          script Python kiểm tra ảnh cầu trên máy tính (dựng các góc nhìn phối cảnh, đo tiêu cự)
```

### Quy trình ghép

1. Tìm điểm SIFT ở ảnh ~1 MP, chỉ so khớp các cặp nằm cạnh nhau theo cảm biến.
2. Mỗi cặp: RANSAC 2 điểm tìm phép xoay tương đối, bỏ cặp lệch xa cảm biến.
3. Tinh chỉnh góc mọi tấm cùng lúc (Huber), neo chặt theo trọng lực, neo lỏng theo hướng ngang.
4. Optical flow ở từng vùng chồng; mỗi ảnh uốn nửa đường để nội dung gặp nhau. Cặp nào khớp mô hình
   epipolar (máy dịch đi khi chụp) thì flow bị ép dọc đường epipolar.
5. Cân bằng sáng theo từng ảnh và kênh màu, đường nối graph cut, giao đỉnh/đáy cho tấm chụp thẳng lên/xuống,
   nối vòng ở kinh tuyến 180°.
6. Dựng ở độ phân giải đầy đủ, trộn nhiều dải tần (≤ 8K) hoặc trộn mép (lớn hơn).

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

## Mẹo chụp

- Ống kính càng đứng yên một chỗ thì càng ít lệch: tốt nhất là xoay máy quanh một chỗ kê (hộp, gậy, chân máy).
- Tắt quạt, chờ không có người đi qua: vật chuyển động sẽ bị nhòe hoặc cắt đôi.
- Tấm đầu tiên hướng vào chỗ cách 2–3 m vì app khoá nét, sáng và màu theo tấm này.

## Giới hạn đã biết

- Vật rất gần máy (dưới ~1 m) khi chụp cầm tay vẫn có thể gãy hoặc nhân đôi ở chỗ nối: optical flow không
  bắt được độ lệch trên ~6–10°.
- Đáy ảnh thấy chân người chụp hoặc chỗ kê.
- Trình xem trong app giới hạn ảnh ở 8192 px chiều ngang.
- Việc ghép chạy trong màn hình ghép, chưa chạy nền.

## Công cụ kiểm tra (`tools/`)

Cần Python với `opencv-python` và `numpy`.

```bash
python tools/grid_views.py pano.jpg out      # out_eq.jpg, out_grid.jpg (12 góc nhìn), out_poles.jpg
python tools/detail_views.py pano.jpg out.jpg 0,30,80 90,0,70   # các góc yaw,pitch,fov tuỳ chọn
python tools/focal_probe.py <thư mục lượt chụp>                 # đo tiêu cự thật từ các cặp ảnh kề
```
