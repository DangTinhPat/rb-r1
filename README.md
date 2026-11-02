# RB-R1

Robot hai bánh có chân. Repository này chứa **hai chương trình chạy trên hai
bộ xử lý khác nhau**. Chúng không dùng chung code, không dùng chung build, không
chạy chung máy; chúng chỉ nói chuyện với nhau qua một sợi UART bằng giao thức
HCR1.

```text
┌───────────────────────── highlv/ ─────────────────────────┐        ┌──────────────────────── lowlv/ ────────────────────────┐
│ Tầng trên · SBC Luckfox RV1106 (Linux, ARMv7, NPU)        │        │ Tầng dưới · ESP32-S3 N16R8 (firmware, PlatformIO)       │
│                                                           │        │                                                         │
│ Python runtime + helper C++ (recovery) + policy RKNN/GRU  │  HCR1  │ GatewayCore + driver 6 motor BL4012 + IMU MPU6500       │
│ nghĩ: đứng dậy, giữ Support, chạy model, chọn target      │◄──────►│ làm: đọc cảm biến, áp target, kiểm hạn lệnh, tắt motor │
│                                                           │  UART  │                                                         │
│ /dev/ttyS2 (UART2, GPIO1_B2/B3)                           │ 921600 │ UART0 (TX GPIO43, RX GPIO44)                            │
└───────────────────────────────────────────────────────────┘  8N1   └────────────────────────────────────────┬────────────────┘
                                                                                      UART1/GPIO17 (bus phải) ─┤─ UART2/GPIO15 (bus trái)
                                                                                      IMU MPU6500 (SPI) ───────┘  → 6 động cơ BL4012
```

## Hai thế giới

| | `highlv/` | `lowlv/` |
|---|---|---|
| Chạy ở đâu | Luckfox RV1106, Linux | ESP32-S3, bare firmware (Arduino/PlatformIO) |
| Ngôn ngữ | Python ≥3.9 (stdlib), C++17, C | C++, C |
| Vai trò | Quyết định: recovery, policy, chọn TARGET | Thi hành: đọc IMU/encoder, áp reference, bảo vệ |
| Thời gian | Telemetry xử lý 200 Hz, policy 100 Hz, deadline mềm trên Linux | Task service 2 kHz, TTL lệnh 20 ms, watchdog motor độc lập |
| Phần cứng chạm tới | UART tới ESP, NPU, (không chạm motor) | IMU, encoder, 2 bus motor, Wi-Fi/OTA |
| Build | CMake + `scripts/build.sh host\|arm` | PlatformIO, env `esp32-s3-hybrid-mcu` |
| Test | `scripts/test.sh`, `scripts/smoke_test.py` | `tests/check_*.py`, `tests/test_*.py` (chạy trên host) |
| Tài liệu | [highlv/README.md](highlv/README.md) | [lowlv/README.md](lowlv/README.md) |

`highlv` **không biết** motor nối thế nào, bus nào, ID nào. `lowlv` **không biết**
robot đang đứng dậy hay cân bằng, model nào đang chạy, hay policy là gì. Nó chỉ
biết: có TARGET hợp lệ, còn mới, đúng epoch, cả hai bus áp dụng được hay không.

## Ranh giới và quy tắc

1. **Không có code dùng chung.** `highlv` không `include`, import hay link bất cứ
   thứ gì trong `lowlv`, và ngược lại. Mỗi bên tự có codec HCR1 của riêng mình
   (`highlv/protocol/`, `highlv/src/rb_runtime/protocol.py` và
   `lowlv/include/gateway/protocol/`, `lowlv/tools/hcr_protocol.py`). Hai bản
   trùng nhau là chủ đích, không phải sơ suất cần gộp.
2. **Mỗi bên tự giả lập bên kia để test.** `highlv/tests/fixtures/` thay thế ESP
   bằng fixture tối giản; `lowlv/tools/hcr_uart_peer.py` thay thế Luckfox.
   Các fixture này chỉ để test codec/luồng, không phải sản phẩm của bên kia.
3. **Chỗ duy nhất hai bên gặp nhau là hợp đồng HCR1:** frame, payload, flag, profile,
   mã lỗi, thời hạn. Đổi hợp đồng thì sửa cả hai bên trong cùng một thay đổi và
   cập nhật cả hai tài liệu giao thức.
4. **Mỗi bên sống độc lập khi triển khai.** `highlv` chép lên Luckfox; `lowlv` nạp
   vào ESP32-S3 bằng USB hoặc OTA. Cập nhật bên này không cần động bên kia, miễn
   hợp đồng HCR1 không đổi.

## Hợp đồng HCR1 (tóm tắt)

Chi tiết đầy đủ: [highlv/docs/protocol.md](highlv/docs/protocol.md) và
[lowlv/docs/HCR1_PROTOCOL.md](lowlv/docs/HCR1_PROTOCOL.md).

- UART TTL 3,3 V, full duplex, 921600 baud, 8N1; frame little-endian, magic `A5 5A`,
  CRC32. Nối chéo TX/RX, chung GND.
- ESP → Luckfox: `HELLO_ACK`, `TELEMETRY` (gyro, trọng lực, accel, q, qd, target đã
  áp dụng, cờ hợp lệ, `applied_seq`).
- Luckfox → ESP: `HELLO`, `TARGET`, `STATUS`.
- `TARGET` = 4 góc tuyệt đối `[hipL, hipR, crankL, crankR]` (rad) + 2 tốc độ bánh
  `[omegaL, omegaR]` (rad/s) + 4 vận tốc feedforward. **Không có torque.**
- ESP chỉ nhận TARGET đúng CRC, đúng epoch, sequence mới, số hữu hạn, nguồn còn
  mới ≤ 30 ms, và coi nó hết hạn sau 20 ms. Mất liên lạc, quá hạn hoặc lỗi driver
  thì tắt motor và giữ lỗi.
- `applied_seq` chỉ tăng khi **cả hai bus motor** xác nhận đã áp dụng reference.

Lệnh vận hành (ARM, DISARM, RESET, ESTOP, VELOCITY) không đi qua HCR1; chúng là
lệnh socket cục bộ trên Luckfox.

## Thứ tự làm việc an toàn

1. Bật nguồn ESP; chạy `MOTOR_HOME` (thủ công) trước khi cho phép ARM.
2. Khởi động runtime `highlv`; hai bên bắt tay HELLO/HELLO_ACK và vào DISARMED.
3. ARM → HOME → STANDUP → WARMUP → MODEL (chi tiết ở README của `highlv`).

Không vận hành motor, flash firmware, OTA hay triển khai lên Luckfox nếu chưa có
yêu cầu rõ ràng. Quy tắc bảo trì nằm ở [AGENTS.md](AGENTS.md).

## Trạng thái kiểm chứng

| Hạng mục | Tình trạng |
|---|---|
| `highlv` build host, 27 unit test, smoke test | PASS sau khi chuyển vào `highlv/` (2026-11-02) |
| `highlv` cross-build ARM, RKNN/NPU trên Luckfox | Chưa kiểm lại sau khi chuyển thư mục |
| `lowlv` | Chỉ mới được copy vào repo; chưa build PlatformIO hay chạy test trong thư mục này |
| UART vật lý LF–ESP, driver motor, đứng dậy/cân bằng trên robot | Chưa kiểm lại trong đợt gộp repo |
| Báo cáo trước đó | Có lỗi echo/reply bus motor lúc Folding, chưa bàn giao sang model (xem `highlv/docs/history/`) |

## Nguồn gốc

- `highlv/`: runtime tầng trên, trước đây là repository `reconect`; core recovery phát
  triển từ Lumi-Quantum (xem `highlv/docs/source_provenance.md`).
- `lowlv/`: firmware tầng dưới, trước đây là repository `gatewayLF`, snapshot tại
  commit `afdff75` (không kèm lịch sử commit).
