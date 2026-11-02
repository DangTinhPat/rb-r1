# check_imu: kiểm tra hướng trục và bộ lọc IMU

Firmware đọc **dữ liệu thô** của MPU6500 qua SPI rồi gửi lên máy tính **không dây** (Wi-Fi, TCP) hoặc qua cáp USB;
GUI pygame vẽ một chiếc máy bay xoay theo dữ liệu thật để bạn đối chiếu với thực tế. Không điều khiển gì.

```text
check_imu/
├── firmware/            PlatformIO: đọc MPU6500, đọc đều 500 Hz trong task riêng, phát $IMU,... qua TCP 8023 và UART0, có OTA
└── gui/imu_plane.py     GUI pygame: máy bay, bộ lọc, bản đồ trục
```

## Đấu nối IMU (giống PCB robot)

| Tín hiệu | GPIO |
| --- | --- |
| CS | 10 |
| MOSI | 11 |
| SCLK | 12 |
| MISO | 13 |
| INT | 14 (chưa dùng) |

Cấu hình thanh ghi giống firmware chính: gia tốc ±2 g, con quay ±500 °/s, lọc số bật, lấy mẫu nội 500 Hz.

## Chạy không dây (khuyến nghị)

Board nối mạng Wi-Fi `LUMI`, phát dữ liệu qua TCP cổng 8023 (một máy tính mỗi lúc) và nhận OTA. Chỉ cần cấp nguồn
cho PCB, không cần cáp dữ liệu.

```bash
cd examples/check_imu/firmware

# Thông tin mạng đã có sẵn trong repo riêng tư; muốn đổi mạng thì sửa hoặc tạo lại hai file này từ file mẫu
cp include/wifi_secrets.example.h include/wifi_secrets.h
cp secrets.example.ini secrets.local.ini

# Nạp lần đầu bằng cáp USB
pio run -e check_imu -t upload

# Từ lần sau nạp qua Wi-Fi, dùng IP của board (xem trong log serial hoặc router)
pio run -e check_imu_air -t upload --upload-port 10.10.61.85

# GUI nối qua mạng
pip install -r ../gui/requirements.txt
python3 ../gui/imu_plane.py --port socket://10.10.61.85:8023
```

- Hostname của board là `rb1-imu`. Tên `rb1-imu.local` chỉ dùng được nếu máy tính hỗ trợ mDNS; nếu không, dùng IP.
- **Đọc cảm biến tách khỏi việc gửi mạng.** Một task riêng ở lõi 1 đọc MPU6500 đúng chu kỳ 2 ms (500 Hz, bằng tần số
  mẫu nội của chip), ghi mốc thời gian micro giây ngay sau khi đọc, rồi đưa vào hàng đợi 1024 mẫu; `loop()` lấy từ hàng
  đợi để gửi. Wi-Fi nghẽn chỉ làm gói đến muộn, không làm mất mẫu và không làm lệch mốc thời gian.
- Đo trên mạng thực (40 giây): nhận đúng **500.0 Hz**, khoảng cách mẫu lớn nhất 2.6 ms, không có đoạn đứng nào quá 4 ms.
  Bản cũ đọc và gửi chung một vòng nên mỗi lần Wi-Fi nghẽn là mất một đoạn mẫu: 46 lần đứng hình mỗi phút, tổng 10%
  thời gian không có mẫu, nên khi board chuyển động góc xoay bị đánh giá thấp và yaw trôi.
- Firmware cũng tắt chế độ tiết kiệm năng lượng Wi-Fi. Nếu mạng không kịp, firmware báo `# bo mau: ...` số mẫu bị bỏ.
- Chỉ một máy tính nối được cùng lúc; máy thứ hai nhận `# busy`. Đóng GUI trước khi chạy công cụ khác.

## Chạy bằng cáp USB

```bash
cd examples/check_imu/firmware
pio run -e check_imu -t upload
python3 ../gui/imu_plane.py                  # tự tìm /dev/ttyUSB* hoặc /dev/ttyACM*
python3 ../gui/imu_plane.py --port /dev/ttyUSB0
python3 ../gui/imu_plane.py --demo           # dữ liệu giả lập, không cần board
```

Mở cổng serial sẽ reset ESP32, nên đóng Serial Monitor trước khi chạy GUI.

## Cách đọc máy bay

Hệ trục robot: **+X phía trước (mũi)**, **+Y bên trái**, **+Z lên**. Cánh trái tô **đỏ**, cánh phải **xanh**, mũi **cam**.

- Robot cúi mũi xuống thì mũi máy bay phải cúi xuống (pitch dương).
- Robot hạ hông phải xuống thì cánh trái (đỏ) phải nhấc lên (roll dương).
- Xoay robot sang trái quanh trục đứng thì máy bay quay sang trái (yaw dương). Yaw chỉ lấy từ con quay nên sẽ trôi
  dần, vì IMU không có từ kế.

## Bộ lọc (bật tắt bằng phím hoặc chuột)

| Phím | Chức năng |
| --- | --- |
| `F` | Đổi chế độ: **FUSION** (gia tốc + con quay, bộ lọc bù kiểu Mahony), **ACCEL** (chỉ gia tốc: nhiễu, không có yaw), **GYRO** (chỉ con quay: mượt nhưng trôi) |
| `1` | Lọc thông thấp gia tốc (5 Hz) |
| `2` | Lọc thông thấp con quay (20 Hz) |
| `3` | Trừ bias con quay (bias đo bằng `C` hoặc tự học) |
| `4` | Tự học bias khi robot đứng yên |
| `5` | Khóa yaw khi đứng yên (coi vận tốc góc bằng 0) |
| `6` | Thành phần tích phân kiểu Mahony: bù bias con quay trục X, Y khi đang chuyển động |
| `C` | Đo bias con quay trong 2 giây (giữ robot đứng yên) và đặt yaw về 0 |
| `+` / `-` | Tăng/giảm hệ số hiệu chỉnh Kp của FUSION |
| `R` | Đặt lại tư thế |
| `Space` | Tạm dừng |
| `M` | Đổi bản đồ trục chip → thân |

Mặc định `3`, `4`, `5`, `6` bật và `1`, `2` tắt. Tắt từng công tắc để so sánh tác dụng.

## Chống trôi

IMU 6 trục không có từ kế, nên **yaw chỉ có thể tính bằng cách cộng dồn con quay trục Z**. Mọi độ lệch nhỏ của con quay
đều bị cộng dồn thành góc trôi, không thể loại bỏ hoàn toàn bằng bộ lọc. Đo trên board thật (nằm yên 60 giây): con quay Z
lệch 0.80 °/s, tức yaw trôi khoảng 48° mỗi phút; con quay Y còn lệch lớn hơn (khoảng 12 °/s).

Cách xử lý, theo các phương pháp được dùng trong thực tế:

- **Tích phân Mahony (phím 6):** thành phần tích phân của sai số nghiêng bù bias không đổi của con quay. Trọng lực chỉ
  cho thấy bias vuông góc với nó, nên chỉ sửa được X và Y; bias quanh trục yaw không ước lượng được bằng cách này.
  Phần tích phân được **chiếu bỏ thành phần song song với trọng lực** trước khi dùng. Nếu không, khi board nghiêng một
  phần của nó rò sang trục yaw thành vận tốc xoay giả (đo trong mô phỏng: yaw sai 14.4° sau 60 giây chuyển động mạnh,
  sau khi chiếu bỏ chỉ còn 0.5°).
- **Nhận diện đứng yên và tự học bias (phím 4, 5):** theo cách hiệu chuẩn con quay lúc chạy của Android (GyroCal)
  và ZUPT/ZARU trong dẫn đường quán tính. Cửa sổ 1.5 giây, "yên" khi độ lệch chuẩn của con quay và gia tốc đều nhỏ.
  Lần đầu phải yên liên tục 3 giây mới nhận bias (kể cả thời gian đầy cửa sổ, tổng khoảng 4.5 giây); khi đó yaw được đặt về 0.
  Sau đó chỉ coi là đứng yên khi con quay nằm trong khoảng 2 °/s quanh bias, nên xoay đều không bị nhầm thành độ lệch.
  Khi đứng yên, vận tốc góc được coi bằng 0 để yaw không trôi. Bias được cập nhật chậm (hằng số thời gian 5 giây).

Kết quả thử trên dữ liệu thật và dữ liệu mô phỏng có bias và nhiễu giống thật:

| Tình huống | Không xử lý | Với `3`, `4`, `5`, `6` |
| --- | --- | --- |
| Board yên 60 giây (thật) | yaw +60.1° | yaw +0.02° |
| Yên, xoay 90° trong 3 giây, yên | sai +24.7° | sai −0.13° |
| Xoay 5 °/s trong 10 giây | sai +24.5° | sai −0.13° |
| Xoay đều 20 °/s trong 20 giây | sai nhiều | đúng |
| Nghiêng liên tục, con quay lệch 12 °/s | pitch sai 6.3° | pitch sai 0.8° |

Khi board chuyển động, độ chính xác phụ thuộc vào việc không bỏ sót mẫu. Mô phỏng 60 giây chuyển động tay mạnh trên
cả ba trục (sau 12 giây yên, có bias và nhiễu như đo thật, đáp án lấy từ quỹ đạo thật; 5 lần thử):

| Cấu hình | Sai số yaw cuối (trung bình) |
| --- | --- |
| Firmware cũ: 200 Hz, đứng hình như đo thật, tích phân Mahony chưa chiếu | 57.5° |
| Firmware mới: 500 Hz đều, tích phân Mahony đã chiếu | **0.5°** |

Sai số hướng "lên" (nghiêng) trong cùng mô phỏng chỉ 0.18° (RMS). Các con số mô phỏng không bao gồm gia tốc thẳng của
tay (gia tốc kế chỉ thấy trọng lực), nên trên thực tế nghiêng sẽ kém hơn khi lắc mạnh.

Giới hạn còn lại: không có từ kế nên yaw vẫn **trôi khi robot chuyển động lâu** (bias thay đổi theo nhiệt độ, nhiễu cộng dồn),
và xoay đều chậm dưới khoảng 2 °/s bị coi là đứng yên. Muốn yaw tuyệt đối thì cần từ kế hoặc tham chiếu ngoài.
Với robot này, một tham chiếu ngoài khả thi là chênh lệch tốc độ hai bánh (odometry), vì khi robot yên thì tốc độ yaw thật bằng 0.

Tài liệu tham khảo:
[Mahony orientation filter](https://ahrs.readthedocs.io/en/latest/filters/mahony.html),
[Madgwick, báo cáo nội bộ](https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/madgwick_internal_report.pdf),
[Android GyroCal (gyro_cal.c)](https://chromium.googlesource.com/chromiumos/platform/ec/+/master/common/gyro_cal.c),
[Fifteen Years of Progress at Zero Velocity (ZUPT)](https://arxiv.org/pdf/2008.09208).

## Bản đồ trục

IMU nằm ở mặt dưới PCB, **úp xuống**: chip X hướng về phía trước, chip Y hướng sang phải, chip Z hướng xuống gầm.
Bản đồ mặc định (`Úp xuống`) đổi sang trục thân: thân = (X chip, −Y chip, −Z chip). Phím `M` đổi sang các bản đồ khác
để thử khi thực tế không khớp.

Kiểm từng trục:

1. **Nằm yên:** dòng "Kiểm tra khi nằm yên" phải báo ✔ (Z thân ≈ +9.81). Z chip đọc −9.81.
2. **Cúi mũi xuống:** máy bay cúi mũi xuống; X chip đọc âm.
3. **Cúi mũi xuống:** ω Y chip đọc âm.
4. **Hạ hông phải xuống:** cánh đỏ nhấc lên; Y chip đọc âm.

Nếu máy bay chuyển động ngược hướng với robot, đổi bản đồ bằng `M` cho tới khi khớp, rồi cập nhật `kImuAxisMap` trong
`firmware/control/include/rb1/control/board/BoardConfig.hpp` và `model/imu.yaml`.

## Định dạng dữ liệu

```text
$IMU,<ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>      ms: mili giây từ lúc bật (có phần lẻ micro giây); a*: m/s²; g*: rad/s (hệ trục chip); 500 mẫu mỗi giây
# ...                                          thông báo, GUI bỏ qua
```
