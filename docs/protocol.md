# HCR1 — Giao thức Luckfox ↔ MCU

HCR1 truyền telemetry từ MCU lên runtime và reference từ runtime xuống MCU.
Luckfox chạy recovery/policy; MCU đọc cảm biến, homing và thực thi reference.
Codec Python: `src/rb_runtime/protocol.py`. Codec C đối chiếu:
`protocol/include/hcr_protocol.h`. Firmware MCU là project riêng.

## 1. Transport và frame

UART TTL 3,3 V, full duplex, **921600 baud, 8N1**, không flow control.
Kết nối đã ghi nhận: ESP32 TX43 → LF UART2 RX GPIO1_B3;
ESP32 RX44 ← LF UART2 TX GPIO1_B2; GND chung.
Đây là đường LF–MCU; bus motor thuộc firmware MCU.

Little endian, không padding. Float IEEE754 binary32.

| Offset | Số byte | Trường |
|---:|---:|---|
| 0 | 2 | Magic `A5 5A` |
| 2 | 1 | Version = 1 |
| 3 | 1 | Message type |
| 4 | 2 | Payload length, tối đa 256 |
| 6 | 2 | Reserved = 0 |
| 8 | 4 | Sequence uint32, tăng theo từng chiều |
| 12 | 8 | Timestamp monotonic microsecond |
| 20 | N | Payload |
| 20+N | 4 | CRC32 của header và payload |

CRC32 IEEE tương đương `zlib.crc32`, reflected polynomial `0xEDB88320`,
init/final XOR `0xffffffff`. Sequence wrap được xét theo nửa khoảng uint32.
Frame replay/cũ/sai epoch không làm mới watchdog. Timestamp TELEMETRY là lúc lấy sample.

| Type | Tên | Chiều |
|---:|---|---|
| `0x01` | HELLO | LF → MCU; MCU có thể yêu cầu phiên mới |
| `0x02` | HELLO_ACK | MCU → LF |
| `0x03` | TELEMETRY | MCU → LF |
| `0x81` | TARGET | LF → MCU |
| `0x82` | STATUS | LF → MCU |

ARM/DISARM/RESET/ESTOP/VELOCITY là lệnh qua command socket local của runtime,
không có message CONTROL trong HCR1 hiện tại.

## 2. HELLO và phiên điều khiển

Payload 12 byte, `<IHHI>`: `epoch, telemetry_hz=200, policy_hz=100, schema=0x31524348`.
LF sinh epoch uint32 khác 0 cho mỗi phiên mới. MCU trả HELLO_ACK với cùng payload.
Handshake chỉ mở link và đưa LF về DISARMED; không cho phép motor tự hoạt động.

MCU reboot có thể phát HELLO `(0,200,100,schema)` kể cả khi sequence trở về 0.
LF phát disabled output, tạo epoch mới và bắt tay lại. RESET/reconnect không tự ARM.

## 3. TELEMETRY

Payload 128 byte, `<III29f>`: `epoch:u32, flags:u32, applied_seq:u32, values[29]`.
Tổng frame 152 byte.

| Float index | Giá trị | Đơn vị / frame |
|---:|---|---|
| 0–2 | gyro `[wx,wy,wz]` | rad/s, body frame |
| 3–5 | projected gravity `[gx,gy,gz]` | vector đơn vị; thẳng đứng `[0,0,-1]` |
| 6–8 | accel specific force | m/s²; tĩnh thẳng đứng `[0,0,+9.81]` |
| 9–14 | q `[hipL,hipR,kneeL,kneeR,wheelL,wheelR]` | rad, đã đổi chiều/homing, bánh unwrap |
| 15–20 | qd cùng thứ tự q | rad/s |
| 21–22 | generalized hip torque `[L,R]` | Nm, feedback tùy chọn |
| 23–28 | applied target | cùng thứ tự và đơn vị TARGET |

Body frame: +X tiến, +Y trái, +Z lên. q chưa trừ default của model.
Encoder crank được đổi thành `q_knee = phi_measured − q_hip_measured`,
`qd_knee = phi_dot_measured − qd_hip_measured` trên MCU.

| Mask | Ý nghĩa |
|---:|---|
| 1 | IMU/attitude hợp lệ và mới |
| 2 | Tất cả encoder hợp lệ và mới |
| 4 | Đã hiệu chuẩn/homing |
| 8 | E-stop |
| 16 | Có hip torque feedback đáng tin |
| 32 | Có applied target hợp lệ |
| 64 | Driver fault |

Ba bit đầu bắt buộc cho telemetry điều khiển. Bit 32 bắt buộc từ warm-up.
Thiếu torque thì đặt hai giá trị bằng 0 và bỏ bit 16; không tạo dữ liệu giả.
Generalized hip torque = torque hip motor + torque crank motor theo tọa độ model.
Torque feedback không thuộc actor observation 32D.

`applied_seq` chỉ tăng sau khi MCU xác nhận reference đã thực sự áp dụng.
`applied_target` là reference cuối sau filter/quantization/limiter nếu có;
không phải encoder đo, giá trị vừa enqueue hoặc target của một bus chưa hoàn tất.
Backend hai chân phải xác nhận cả hai bus trước khi ACK batch. Không cần chờ
trục vật lý đi tới vị trí đích.

## 4. TARGET

Payload 52 byte, `<IIHH10f>`, tổng frame 76 byte.

| Offset payload | Trường |
|---:|---|
| 0 | epoch:u32 |
| 4 | source_seq:u32 — sequence TELEMETRY dùng để tính lệnh |
| 8 | profile:u16 |
| 10 | enables:u16 — bit 0 chân, bit 1 bánh |
| 12 | target[6]: hipL, hipR, crankL, crankR, omegaL, omegaR |
| 36 | feedforward[4]: hipL, hipR, kneeL, kneeR rates |

Target[0:4] là góc tuyệt đối rad sau homing. Target[4:6] là tốc độ bánh rad/s.
Không có torque command. Với driver điều khiển crank, gửi `phi_ref` sau sign/zero;
không gửi `phi_ref − hip_ref` thay cho crank target. Hiệu đó chỉ là knee hinge reference.
Feedforward là hip/knee hinge velocity; crank FF = hip FF + knee FF.

| Profile | Ý nghĩa |
|---:|---|
| 0 DISABLED | Bỏ enable; payload 0 không yêu cầu chạy motor về góc 0 |
| 1 HOME_PD | Giữ home |
| 2 RECOVERY | Reference đứng dậy/Support và velocity FF |
| 3 POLICY_PD | Reference giải mã từ policy; FF = 0 |

MCU chỉ nhận frame CRC đúng, đúng epoch, sequence mới, float finite, profile
hợp lệ và source sample đã phát còn mới <=30 ms. TTL TARGET **20 ms** kể từ nhận.
Frame replay/cũ hoặc STATUS không gia hạn TTL. Watchdog motor phải chạy độc lập
với task UART và LF. Lỗi/timeout không tự re-enable khi có gói mới.

Recovery helper đã có wheel delay 50 ms. Filter reference recovery tau=0,075 s
thuộc tầng áp dụng phía dưới; applied_target phải phản ánh reference sau filter.
Đáp ứng/gain driver thật cần kiểm riêng, không suy ra từ payload HCR1.

## 5. STATUS và deadline

Payload `<IIII>`: `epoch, mode, fault_code, rejected_frames`.
Mode: 0 WAIT_LINK, 1 DISARMED, 2 HOME, 3 STANDUP, 4 WARMUP, 5 MODEL, 6 FAULT.

| Fault | Ý nghĩa |
|---:|---|
| 1 | Telemetry timeout |
| 2 | Telemetry không hợp lệ |
| 3 | Driver fault / E-stop |
| 4 | Clock/sample gap hoặc queued sample quá cũ |
| 5 | Thiếu applied_target khi warm-up/policy |
| 6 | Recovery helper lỗi |
| 7 | Policy lỗi |
| 8 | Inference vượt hard deadline hoặc trễ liên tiếp |
| 9 | Ngã >70° trong MODEL |
| 10 | Transport lỗi hoặc service dừng |
| 11 | Telemetry handler >15 ms khi đang điều khiển |
| 12 | Applied ACK quá hạn |

LF timeout telemetry/ACK 30 ms. Policy chạy 100 Hz theo lịch tuyệt đối.
Inference >10 ms: bỏ prediction, giữ reference trước, hidden vẫn cập nhật.
Inference >15 ms hoặc 3 lần trễ liên tiếp: FAULT. Không handoff bằng prediction muộn.
Thông tin chi tiết, latency và counter được ghi thêm vào status JSON của LF.

## 6. Codec đối chiếu và firmware thật

`protocol/include/hcr_protocol.h` phục vụ interop C/Python.
`tests/fixtures/reference_mcu_link.h` chỉ là fixture tối giản để test codec/TTL;
không chứa đầy đủ state machine, sensor freshness, source history và motor worker
của firmware MCU. Không dùng fixture thay firmware thật.

Mapping bus được ghi nhận ở firmware riêng: trái UART2/GPIO15, phải UART1/GPIO17;
wheel ID1, hip ID2, crank ID3. Homing thủ công và xác nhận applied/OFF hai bus
thuộc trách nhiệm firmware MCU. Project runtime không cấu hình bus motor.
