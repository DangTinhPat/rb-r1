# ESP motor bench cho GUI trên LF

Firmware bench độc lập chuyển từ `lumi-quantum/examples/check_motor`.
Phía LF nằm trong `highlv/examples/check_motor` nhưng hai phần không import,
include hoặc build phụ thuộc nhau. Không sửa production HCR1/IMU/policy.

- UART0 **921600 8N1, TX43/RX44** nhận lệnh LF; log ESP console bị tắt để tránh
  xen log vào UART. Telemetry, ACK và lỗi bench trả trên UART.
- Hai worker riêng cho bus phải UART1/GPIO17 và trái UART2/GPIO15.
- Giữ driver MIT/BL4012, jog, MOVE tương đối, wheel rpm, gains và telemetry từ Lumi.
- Khớp được cấu hình thanh ghi response-level 0x10 = 2 lúc OFF, bắt buộc đọc
  lại đúng giá trị. Bánh có thể không trả write ACK: xác nhận bằng đọc lại thanh
  ghi torque/goal, không coi timeout đơn thuần là thành công.
- Wi-Fi TCP8023 có xác thực chỉ đọc log; HTTP8081 giữ status và OTA của example
  gốc. Không nhận lệnh điều khiển motor qua Wi-Fi.
- Boot OFF/latch; bắt buộc sáu motor online và đọc lại OFF trước RESET/HOME.
  HOME chụp tư thế OFF hiện tại, không chạy cơ cấu hay thay zero driver.
- Lease UART 20 ms, lỗi giữ đến RESET, lỗi bus khi ON tắt cả hai bên. Đọc chi tiết
  timeout và giới hạn xác nhận ở README GUI.

## Build

Không Wi-Fi/OTA:

```sh
pio run -d lowlv/examples/check_motor/firmware -e check-motor
```

Có Wi-Fi/OTA: chép
`firmware/src/libs/bench_net/bench_secrets.example.h` thành `bench_secrets.h`
cùng thư mục, điền mạng 2.4 GHz và access key riêng, rồi:

```sh
pio run -d lowlv/examples/check_motor/firmware -e check-motor-ota
```

`bench_secrets.h`, `.local/`, `.pio/` và sdkconfig sinh tự động đều gitignored.
Máy phát triển hiện đã có header cục bộ lấy từ cấu hình check_hcr_link của RB-R1;
không sao chép credential Lumi-Quantum. Không commit file credential.

Output: `firmware/.pio/build/check-motor-ota/firmware.bin`. Build không tự nạp.
Chuyển lần đầu từ firmware Arduino `check_hcr_link` dùng Arduino OTA của firmware
đang chạy (hoặc USB). Chỉ nạp **application firmware.bin**, không nạp merged image,
bootloader hay partition table qua OTA. Partition map ví dụ này khớp
Arduino `default_16MB.csv` (app0 0x10000, app1 0x650000, mỗi slot 0x640000).

Sau khi bản bench OTA chạy, dùng công cụ HTTP kế thừa:

```sh
python3 lowlv/examples/check_motor/tools/ota_upload.py --status
python3 lowlv/examples/check_motor/tools/ota_upload.py
```

Công cụ đọc `.local/ota.json` trong folder example: `host`, `password` (= access
key), `mac`. OTA bench xác nhận cả sáu motor OFF bằng register readback trước
khi ghi flash, rồi khóa bus đến reboot. Nếu không có sáu driver kết nối, bước
xác nhận này không pass; dùng USB khi cần chuyển firmware trong trường hợp đó.
Nạp/chạy motor thật cần yêu cầu hiện tại và xác nhận motor OFF; lượt tạo example
này chỉ build/test, không tự flash hay bật motor.

## Giao thức bench UART (không phải HCR1)

Dòng LF kết thúc LF (`\n`), tối đa 95 ký tự:

```text
HELLO                         STOP, reset sequence, trả #HELLO MOTOR_BENCH_1
STOP                          tắt/latch vô điều kiện
P <seq>                       gia hạn lease; chỉ seq tăng nghiêm ngặt
C <seq> RESET                  yêu cầu OFF readback + feedback mới ở cả sáu motor
C <seq> MOTOR_HOME             chụp tư thế OFF hiện tại ở hai worker
C <seq> ON <mask>              chọn motor bằng bit 0..5; phải HOME đủ sáu
C <seq> OFF <mask>
C <seq> W <m> <rpm>            chỉ m=0,3; |rpm| <= 950
C <seq> MOVE <m> <delta_deg> <rpm>  khớp; |delta| <= 360; 0 < rpm <= 30
C <seq> JOG <m> <-1|0|1> <rpm>     khớp; refresh khi giữ nút
C <seq> G <m> <kp> <kd>        gains khớp, clamp theo driver
C <seq> GG
```

P và C dùng chung sequence uint32 từ 1. Mỗi phiên LF bắt đầu HELLO, tắt/latch
trước khi reset sequence. Không replay lệnh từ phiên trước. Không nhận NaN/Inf,
index phân số, giá trị ngoài phạm vi hoặc dư token.

```text
#A <seq> QUEUED                chỉ nhận/xếp lệnh, chưa xác nhận áp dụng vật lý
#A <seq> REJECT <reason>
#S <fault:0|1> <homed_mask> <off_confirmed_mask>
#T <cycleR_us> <cycleL_us> <errorsR> <errorsL> <6 motor records>
#G <m> <kp> <kd> <kp_max> <kd_max>
```

Record motor: `online,on_requested,moving,status_hex,pos_deg,vel_rpm,target,miss`.
Telemetry 50 ms. Xem bảng mapping ở README GUI. Status/encoder là phản hồi driver;
`on_requested` không thay torque-enable readback. OFF không ACK/readback được
vẫn hiển thị chưa xác nhận và tiếp tục thử OFF khi fault.

## Host tests

```sh
python3 lowlv/examples/check_motor/tests/check_motor_bus.py
python3 lowlv/examples/check_motor/tests/check_control.py
```

Test bus giữ golden packet/checksum của Lumi. Test control biên dịch parser,
lease và worker thật với bus/ESP-IDF giả, chạy ASan/UBSan; kiểm tra expiry, replay,
mất write ACK và xác nhận readback, response-level 1→2 lúc OFF, HOME, lỗi driver và không gửi lại MIT quá hạn.
