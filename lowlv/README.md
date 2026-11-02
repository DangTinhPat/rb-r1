# gatewayLF — firmware ESP32-S3 tầng dưới

Firmware chạy trên ESP32-S3 (N16R8), đóng vai **cầu nối giữa Luckfox (LF) và phần cứng robot**:

```
Luckfox (LF)  ──UART0, giao thức HCR1, 921600 baud──►  ESP32-S3  ──►  6 động cơ BL4012 (2 bus UART)
(chạy model,                                             │
 logic đứng dậy)                                         └──◄── IMU MPU6500 (SPI)
```

- LF gửi **4 góc khớp mục tiêu + 2 tốc độ bánh**. Không có lệnh mô-men.
- ESP đọc IMU, tính hướng trọng lực, gửi telemetry cho LF, và chỉ ra lệnh cho motor khi mọi điều kiện an toàn đều thỏa (hạn lệnh 20 ms, driver sẵn sàng, đã HOME…).
- Mất liên lạc với LF, lệnh quá hạn hoặc lỗi driver thì ESP tự tắt motor.
- UART0: chân TX GPIO43, RX GPIO44, nối với UART2 của LF (GPIO1_B2/B3). Console dùng USB native, không dùng UART0.
- Bus motor trái: UART2 / GPIO15. Bus motor phải: UART1 / GPIO17.
- **Sau mỗi lần khởi động phải chạy `MOTOR_HOME` trước khi cho phép ARM.**

## Cấu trúc thư mục

```
gatewayLF/
├── platformio.ini        Cấu hình build (env: esp32-s3-hybrid-mcu)
├── src/                  Mã nguồn .cpp của firmware
│   ├── main.cpp          Điểm vào setup()/loop(): UART0, task service 2 kHz, console, Wi-Fi, OTA
│   ├── core/             Lõi gateway: bắt tay với LF, kiểm tra hạn lệnh, ACK, latch lỗi
│   ├── motor/            Điều khiển 6 động cơ BL4012
│   └── imu/              Driver cảm biến MPU6500
├── include/gateway/      Header, chia theo cùng nhóm với src/
│   ├── protocol/         Giao thức HCR1 (frame, CRC, parser)
│   ├── core/             GatewayCore.hpp
│   ├── motor/            Giao diện và lớp điều khiển motor
│   ├── imu/              Driver IMU, ánh xạ trục, ước lượng trọng lực
│   └── secrets/          File mẫu *.example.hpp cho Wi-Fi và mật khẩu OTA
├── third_party/bl4012e/  Module bus joint_bus.c lấy từ dự án BL4012e/3in1
├── scripts/              Script PlatformIO chạy lúc build
├── tools/                Công cụ chạy trên laptop hoặc LF (không nằm trong firmware)
├── tests/                Test chạy trên máy host, không cần phần cứng
└── docs/                 Tài liệu chi tiết
```

### `src/` và `include/gateway/`

Hai thư mục này có cùng bốn nhóm con. Mỗi nhóm có header trong `include/gateway/<nhóm>/` và mã nguồn trong `src/<nhóm>/`.

| Nhóm | File | Vai trò |
|---|---|---|
| `protocol/` | `hcr_protocol.h` | Mã hóa/giải mã frame HCR1, CRC, parser byte stream. Giống bản phía LF. |
| `core/` | `GatewayCore` | Máy trạng thái chính: bắt tay, epoch/sequence, telemetry 200 Hz, nhận TARGET, kiểm tra hạn 20 ms, latch lỗi, ARM/DISARM. Không biết gì về phần cứng cụ thể. |
| `motor/` | `MotorBackend.hpp` | Giao diện trừu tượng giữa `GatewayCore` và motor (nộp lệnh, hỏi kết quả, tắt motor). |
| | `BlMotor` | Logic motor BL4012: đóng gói lệnh, hai worker (trái/phải), mapping dấu và zero, unwrap encoder, retry có giới hạn, ACK khi cả hai bus đã áp dụng lệnh. |
| | `EspBlMotor` | Phần chạm phần cứng ESP32: UART một dây, task worker. Gọi `joint_bus.c`. |
| `imu/` | `Mpu6500Spi` | Đọc MPU6500 qua SPI. |
| | `ImuMount.hpp` | Ánh xạ trục chip sang hệ body (+X trước, +Y trái, +Z lên). |
| | `Attitude.hpp` | Bù bias gyro và tính vector trọng lực (cả roll và pitch). |
| `secrets/` | `*.example.hpp` | Mẫu cấu hình Wi-Fi và hash mật khẩu OTA. |

`main.cpp` ghép các phần trên lại: nhận byte từ UART0, đẩy vào hàng đợi, và task service gọi `GatewayCore` mỗi 500 µs.

### Các thư mục khác

- **`third_party/bl4012e/`**: module bus `joint_bus.c` do bạn viết và kiểm chứng trên motor ở dự án BL4012e/3in1.
  - `upstream/` là bản gốc, kèm SHA256.
  - `src/` và `include/` là bản firmware thực sự biên dịch, có sửa thêm (deadline, hủy giao dịch, phân loại lỗi).
  - `PROVENANCE.md` ghi rõ khác gì so với bản gốc.
- **`scripts/`**: `bl4012e_bus.py` đưa module bus vào build. `ota_manifest.py` sinh file `.ota.json` (SHA256, hash khóa OTA) bên cạnh `firmware.bin`.
- **`tools/`**:
  - `ota_upload.py`: nạp firmware qua Wi-Fi sau khi xác nhận motor đã OFF.
  - `check_hcr_uart.py`: kiểm tra UART giữa LF và ESP ở chế độ DISABLED.
  - `hcr_uart_peer.py`, `hcr_protocol.py`: giả lập phía LF để test.
  - `decode_motor_wave.py`: giải mã sóng bus motor.
- **`tests/`**: gồm các test và mock bus, build có ASan/UBSan.
  - `check_gateway.py`: test `GatewayCore`, parser, console.
  - `check_bl_motor.py`: test logic motor với bus giả.
  - `check_inherited_bus.py`: so khớp module bus với bản gốc.
  - `test_*.py`: test công cụ (OTA, giải mã sóng).
- **`docs/`**:
  - `MCU.md`: mô tả tổng quan MCU và các lệnh console.
  - `HCR1_PROTOCOL.md`: định dạng frame.
  - `MOTOR_INTEGRATION.md`: ghép driver motor.
  - `OTA.md`: quy trình nạp qua mạng.
  - `FLOW_AUDIT.md`: đối chiếu luồng LF–ESP–motor.
  - `*_VALIDATION.md`: báo cáo kiểm chứng. Các tài liệu này phân biệt kiểm tra build/mock với phép đo thật trên robot.

## Build và nạp

Cài PlatformIO Core, sau đó:

```sh
pio run -e esp32-s3-hybrid-mcu
```

Bản build mặc định **không có Wi-Fi hay OTA**. Muốn bật:

1. Wi-Fi: chép `include/gateway/secrets/HcrWifiSecrets.example.hpp` thành `HcrWifiSecrets.hpp` cùng thư mục, điền mạng 2.4 GHz.
2. OTA: tạo `include/gateway/secrets/OtaSecrets.hpp` chứa MD5 của mật khẩu OTA. Chi tiết trong [docs/OTA.md](docs/OTA.md).

Hai file này đã nằm trong `.gitignore`, **không được commit**.

Chọn cổng nạp đúng board trước khi flash.

## Kiểm tra trên máy host

Không cần phần cứng:

```sh
python3 tests/check_gateway.py
python3 tests/check_bl_motor.py
python3 tests/check_inherited_bus.py
python3 -m pytest -q tests
```

## Đọc thêm

[Mô tả MCU](docs/MCU.md) · [Giao thức HCR1](docs/HCR1_PROTOCOL.md) · [Ghép driver motor](docs/MOTOR_INTEGRATION.md) · [OTA](docs/OTA.md) · [Kiểm chứng bản kế thừa](docs/MISS_RECOVERY_VALIDATION.md)
