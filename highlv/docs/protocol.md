# HCR1 — Giao thức Luckfox ↔ MCU

HCR1 truyền telemetry từ MCU lên runtime và reference từ runtime xuống MCU.
Luckfox chạy kế hoạch gập và policy; MCU đọc cảm biến, homing và thực thi reference.
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
| `0x05` | ENCODER_HOME_ACK | MCU → LF |
| `0x06` | EXECUTION_DIAG (tùy chọn) | MCU → LF |
| `0x07` | CAL_IMU_ACK | MCU → LF |
| `0x81` | TARGET | LF → MCU |
| `0x82` | STATUS | LF → MCU |
| `0x83` | ENCODER_HOME | LF → MCU |
| `0x84` | CAL_IMU | LF → MCU |

Operator gửi ARM/DISARM/RESET/ESTOP/VELOCITY/CAL_IMU qua command socket local;
socket này không phải frame HCR1. Runtime chuyển riêng RESET_ENCODERS và CAL_IMU
thành request HCR1 chuyên biệt. HCR1 không có message CONTROL tổng quát.

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

Adapter HAL Lumi hiện dùng encoder ID3 đo knee trực tiếp (`coupling=0`): MCU đổi
`driver_ref=phi_ref-hip_ref`, `driver_ff=knee_ff`, và phản hồi knee trực tiếp sau
sign/zero. `applied_target[2:4]` vẫn là crank ảo `hip_quantized+knee_quantized`.
Nếu driver đo crank (`coupling=1`) thì dùng phi/phi_dot trực tiếp như mô tả trên.
Đây là chi tiết HAL, không đổi layout hoặc ý nghĩa HCR1.

| Profile | Ý nghĩa |
|---:|---|
| 0 DISABLED | Bỏ enable; payload 0 không yêu cầu chạy motor về góc 0 |
| 1 HOME_PD | Giữ home |
| 2 RECOVERY | Reference gập (fold) và velocity FF |
| 3 POLICY_PD | Reference giải mã từ policy; FF = 0 |

MCU chỉ nhận frame CRC đúng, đúng epoch, sequence mới, float finite, profile
hợp lệ và source sample đã phát còn mới <=30 ms. TTL TARGET **20 ms** kể từ nhận.
Frame replay/cũ hoặc STATUS không gia hạn TTL. Watchdog motor phải chạy độc lập
với task UART và LF. Lỗi/timeout không tự re-enable khi có gói mới.

Filter reference tau=0,075s thuộc tầng áp dụng phía dưới;
applied_target phải phản ánh reference sau filter.
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
| 6 | Kế hoạch gập lỗi (pitch quá giới hạn hoặc đầu vào không hữu hạn) |
| 7 | Policy lỗi |
| 8 | Inference vượt hard deadline hoặc trễ liên tiếp |
| 9 | Ngã >70° trong MODEL |
| 10 | Transport lỗi hoặc service dừng |
| 11 | Telemetry handler >15 ms khi đang điều khiển |
| 12 | Applied ACK quá hạn |

LF vẫn giữ telemetry timeout 30 ms. Cửa sổ ACK thích ứng từ 30 ms tới mức trần
60 ms trong `config/runtime.json`: runtime học latency ACK đầu-cuối từ các
`applied_seq` tiến triển gần nhất (p90 của tối đa 32 mẫu, cộng biên 8 ms bù jitter;
chưa có mẫu thì dùng mức sàn). Cửa sổ không nới thêm theo pha. Timeout ghi
sequence, tuổi ACK và cửa sổ đã chọn khi hết hạn. Cơ chế không đổi TTL TARGET
20 ms, source age hay yêu cầu ACK từ cả hai bus. Trạng thái LF công bố cửa sổ,
tuổi ACK và số mẫu latency trong `ack_diagnostics`.
Policy chạy 100 Hz theo lịch tuyệt đối.
Inference >10 ms: bỏ prediction, giữ reference trước, hidden vẫn cập nhật.
Inference >15 ms hoặc 3 lần trễ liên tiếp: FAULT. Không handoff bằng prediction muộn.
Thông tin chi tiết, latency và counter được ghi thêm vào status JSON của LF.

### Chất lượng đường truyền

Status JSON tách hai nguyên nhân dễ lẫn nhau:

- `link_quality`: suy ra từ dấu thời gian 5 ms của MCU, không phụ thuộc OS. `gaps` và
  `lost_frames` đếm telemetry MCU đã đóng dấu nhưng LF không nhận; `max_burst` là số frame
  mất liên tiếp lớn nhất; `max_gap_us` là khoảng lớn nhất giữa hai mẫu. Mất từ 3 frame liên
  tiếp trở lên dễ chạm TTL 20 ms của MCU. Lệch đồng hồ MCU nhỏ hơn nửa chu kỳ không được
  tính là mất frame.
- `loop_stats`: do chính LF đo. `max_iteration_gap_ms`/`stalls_over_10ms` thấy LF bị đứng
  (bị preempt, RT throttling); `max_frames_per_read` thấy hàng đợi tồn; `handler_ms_histogram`
  (biên `handler_ms_edges`) và `max_read_to_write_ms` là thời gian từ lúc đọc UART đến lúc
  TARGET được ghi, `late_writes_over_5ms` đếm các lần trễ hơn một chu kỳ;
  `max_status_write_ms` và `status_write_failures` cho phần ghi file trạng thái.

Đọc cùng nhau: `lost_frames` tăng mà `stalls_over_10ms` đứng yên là lỗi đường dây/MCU;
cả hai cùng tăng là LF quá tải. TARGET được ghi ngay sau mỗi telemetry, không gom theo lần đọc.


### Chụp HOME khi Disable: ENCODER_HOME (0x83), ENCODER_HOME_ACK (0x05)

Nút web **Reset encoder · Chụp HOME** / lệnh local `RESET_ENCODERS` gửi đúng
một request LF → ESP. Đây là chụp mốc phần mềm như `MOTOR_HOME`, không reset
bộ đếm phần cứng hoặc di chuyển motor. HOME model: hipL/R=2.35706715 rad,
kneeL/R=0; mốc bánh tại vị trí hiện tại đặt về 0. Mốc chỉ giữ trong RAM MCU.

- Request 8 byte `<II>`: `epoch:u32, source_seq:u32` (TELEMETRY đã phát ≤30 ms).
- Reply ESP → LF 12 byte `<III>`: `epoch:u32, request_seq:u32, result:u32`.
  `request_seq` là sequence frame request, không phải sequence TARGET.
- Result: `1 HOME_APPLIED` đã chụp; `2 HOME_NOT_OFF` chưa Disable/verified OFF;
  `3 HOME_FEEDBACK` source cũ/không tồn tại; `4 HOME_REFUSED` lỗi hoặc backend
  từ chối (feedback cũ, encoder còn quay, chưa cấu hình…).

Cả hai chiều dùng CRC/epoch/sequence chung của HCR1. MCU chỉ chụp khi không
armed/enabled, LF không ở HOME/STANDUP/WARMUP/MODEL, cả sáu driver đã xác nhận
OFF, encoder mới và mọi tốc độ tuyệt đối ≤0.15 rad/s. IMU chưa calibrated
không ngăn chụp encoder. E-stop/lỗi driver cục bộ, timeout hoặc lỗi transport
đang latch vẫn ngăn chụp. LF FAULT khác mã 2 bị từ chối; maintenance bỏ request
không trả reply. RemoteFault do LF thiếu calibration không bị xóa
bởi thao tác capture; chỉ được xử lý qua phiên mới sau xác nhận.

Reply này không phải applied TARGET ACK, không đặt TARGET_VALID và không gia
hạn TTL 20 ms. Capture không bật motor, không xóa fault latch và không đổi IMU.
Sau reply thành công, LF gửi disabled, tạo epoch mới rồi bắt tay về DISARMED;
không tự ARM. LF chỉ nhận reply khớp epoch/request đang chờ. Sau 1 s chưa có
reply, UI báo chưa xác nhận; không tự gửi lại khi timeout hoặc reconnect.
Firmware cũ không hỗ trợ request sẽ hết thời gian chờ; phải cập nhật cả hai tầng.

### Calib IMU khi Disable: CAL_IMU (0x84), CAL_IMU_ACK (0x07)

Nút web **Calib IMU · giữ yên** gửi một request LF → ESP với payload `<II>`:
`epoch:u32, source_seq:u32` của TELEMETRY mới (≤30 ms). ESP chỉ nhận khi không
armed/enabled, cả hai bus đã xác nhận OFF, telemetry nguồn mới và không có lỗi
driver/E-stop. ACK `<III>` gồm epoch, request sequence và kết quả: `1` đã nhận;
`2` motor chưa OFF; `3` telemetry cũ; `4` trạng thái không cho phép; `5` đang
calib. ACK chỉ xác nhận bắt đầu. ESP thu tối thiểu 500 mẫu trong ít nhất 1 s;
LF báo hoàn tất khi IMU_VALID xuất hiện, hoặc lỗi nếu chưa hợp lệ sau 5 s.
Lệnh không di chuyển chân, không xóa lỗi driver/E-stop và không tự ARM.

## 6. Codec đối chiếu và firmware thật

`protocol/include/hcr_protocol.h` phục vụ interop C/Python.
`tests/fixtures/reference_mcu_link.h` chỉ là fixture tối giản để test codec/TTL;
không chứa đầy đủ state machine, sensor freshness, source history và motor worker
của firmware MCU. Không dùng fixture thay firmware thật.

Mapping bus được ghi nhận ở firmware riêng: trái UART2/GPIO15, phải UART1/GPIO17;
wheel ID1, hip ID2, gối ID3 (HAL đổi từ crank HCR). Homing thủ công và xác nhận applied/OFF hai bus
thuộc trách nhiệm firmware MCU. Project runtime không cấu hình bus motor.


## Chẩn đoán thực thi tùy chọn: EXECUTION_DIAG (0x06)

MCU → LF, tối đa 10 Hz, payload **96 byte `<24I>`**, mọi trường uint32 LE.
Frame 120 byte dùng chung sequence MCU→LF, epoch và CRC hiện tại; không đổi
TARGET, TELEMETRY, TTL 20 ms hoặc source age 30 ms. Firmware/runtime cũ có thể
bỏ qua type này. Không dùng sequence chẩn đoán làm source TARGET/HOME.

| Word index (offset = index × 4) | Trường |
|---|---|
| 0 | `epoch` |
| 1 | `version = 1` |
| 2 | `local_fault` |
| 3 | `motor_error` |
| 4 | `link_rx_age_us` |
| 5 | `target_rx_age_us` |
| 6 | `pending_age_us` |
| 7 | `pending_seq` |
| 8 | `superseded` |
| 9, 10 | `cycle_l_us, cycle_r_us` |
| 11, 12 | `io_l_us, io_r_us` |
| 13, 14 | `cycle_l_max_us, cycle_r_max_us` |
| 15, 16 | `io_l_max_us, io_r_max_us` |
| 17, 18 | `error_l, error_r` |
| 19, 20 | `stage_l, stage_r` |
| 21 | `flags` |
| 22 | `rx_queue_max_us` |
| 23 | `applied_latency_max_us` |

`link_rx_age_us` tính từ frame LF hợp lệ được service chấp nhận gần nhất;
`target_rx_age_us` từ lúc UART nhận TARGET hợp lệ gần nhất, kể cả DISABLED.
STATUS không làm mới tuổi TARGET. Tuổi chưa biết hoặc tràn uint32 là
`0xffffffff`; tuổi queue bao gồm cả frame bị loại vì chờ quá lâu.

`flags`: bit0/bit1 = worker trái/phải đã claim batch; bit2 = đang có batch chờ;
bit8/bit10 = lease TARGET đã hết hạn lúc worker L/R latch lỗi;
bit9/bit11 = batch đang chạy đã quá deadline gốc lúc worker L/R latch lỗi;
bit31 = backend hỗ trợ số đo. Các bit còn lại bằng 0. `pending_seq` có thể bằng 0 hợp lệ;
chỉ đọc tuổi/seq khi bit2 đặt. Không có backend thì nhóm số đo motor bằng 0,
bit31 clear; không diễn giải thành bus hoạt động tốt.

Chu kỳ và I/O là microsecond. Chu kỳ gồm startup/poll/batch và cả OFF khi lỗi;
I/O là một giao dịch bus. Max chu kỳ/I/O tính từ lượt active gần nhất, được giữ
sau OFF. `superseded` và max completion latency tích lũy từ boot/reset backend.
Completion latency tính từ tạo batch tới service công nhận cả hai bus hoàn tất,
không phải thời gian trục cơ khí đi tới đích. RX queue max đặt lại khi đổi epoch.

`local_fault`: 0 none, 1 TARGET timeout, 2 sensors, 3 driver/backend, 4 E-stop,
5 transport, 6 LF remote fault. `motor_error`, `error_l/r`: 0 none, 1 bus,
2 driver status, 3 configuration, 4 range, 5 expired, 6 feedback, 7 cancelled,
8 startup. `stage_l/r` giữ BusError tại thời điểm worker latch lỗi: 0 none,
1 unavailable, 2 packet, 3 TX, 4 echo timeout, 5 echo mismatch, 6 reply timeout,
7 invalid reply, 8 readback mismatch, 9 deadline, 10 cancelled, 11 quiet,
12 write echo only. Stage=0 không tự chứng minh bus tốt; đối chiếu error/local fault.

MCU bỏ qua lượt chẩn đoán nếu queue TX thiếu chỗ, không gia hạn deadline và
không tạo fault chỉ vì bỏ một frame chẩn đoán chưa truyền. UART chỉ đăng ký
TELEMETRY đã truyền vào source history. LF lưu chẩn đoán riêng; không cập nhật
telemetry/ACK watchdog, timestamp sample, encoder source hoặc sinh TARGET.
Payload sai kích thước/version/epoch hoặc replay bị bỏ, không làm FAULT điều khiển.
Web đánh dấu dữ liệu cũ khi tuổi nhận + tuổi status + độ trễ vận chuyển vượt
1,5 s, hoặc runtime offline. Mốc đồng hồ chẩn đoán lấy riêng từ HELLO_ACK/
chẩn đoán và TELEMETRY nếu có; không sửa đồng hồ điều khiển. Chẩn đoán vẫn
hiện khi IMU/HOME chưa hợp lệ; không dùng số đo cũ để kết luận lỗi.
Đây là số đo hỗ trợ tìm nguyên nhân, không thay cho fault latch hoặc applied ACK.
