# HCR1 — giao diện LF ↔ ESP32-S3 N16R8

LF chạy **kế hoạch gập và policy**. ESP32 lấy cảm biến,
ước lượng attitude, điều khiển servo/driver, và watchdog. ESP32 không cần GRU,
ONNX, MuJoCo hoặc thuật toán quyết định chuyển controller.

**Lệnh điều khiển chính từ LF xuống ESP32 là 4 góc target + 2 vận tốc bánh.**
TARGET không chứa torque target. Torque trong TELEMETRY là feedback tùy chọn
theo chiều ngược lại. Các công thức torque của servo mô phỏng ở cuối mục 5
không đổi ý nghĩa của gói TARGET. Bản đối chiếu luồng: [FLOW_AUDIT.md](FLOW_AUDIT.md).

UART TTL 3.3 V, nối chéo TX/RX và GND. Cấu hình chuẩn: **921600 baud, 8N1,
full duplex, không flow control**. Cấu hình MCU hiện chọn TX0/GPIO43 → LF
GPIO1_B3/UART2 RX, RX0/GPIO44 ← LF GPIO1_B2/UART2 TX, cùng GND.
UART LF–ESP32 này tách biệt với hai bus motor một dây half duplex.
Phải kiểm tra process đang giữ `/dev/ttyS2` trước khi chọn serial transport;
cấu hình hiện dùng serial `/dev/ttyS2`. Ngày 2026-10-31 đã kiểm UART hai chiều:
2.998 TELEMETRY/15 s (199,86 Hz), CRC=0, TARGET DISABLED được ACK.
IMU/encoder/HOME hợp lệ; chưa kiểm chuyển động hoặc TARGET bật motor.

## 1. Frame

Little endian, không padding. Float là IEEE754 binary32. CRC32 IEEE như
`zlib.crc32(header + payload)`; polynomial reflected `0xEDB88320`, init/final xor `0xffffffff`.

| Offset | Bytes | Nội dung |
|---:|---:|---|
| 0 | 2 | Magic `A5 5A` |
| 2 | 1 | Version `1` |
| 3 | 1 | Message type |
| 4 | 2 | Payload length, tối đa 256 |
| 6 | 2 | Reserved = 0 |
| 8 | 4 | Sequence tăng liên tục theo từng chiều, wrap uint32 |
| 12 | 8 | Timestamp microsecond, monotonic; TELEMETRY dùng thời điểm lấy mẫu |
| 20 | N | Payload |
| 20+N | 4 | CRC32 của 20+N byte trước |

`epoch` do LF sinh sau mỗi kết nối/reset, khác sequence. Frame từ epoch cũ bị bỏ.
LF kiểm tra timestamp, CRC và sequence; gói trùng/cũ không làm mới watchdog.
Clock của LF và ESP32 không cần đồng bộ. ESP32 dùng `esp_timer_get_time()` (64 bit).

Các type lệnh vận hành bổ sung: `0x83 ENCODER_HOME` / `0x05 ENCODER_HOME_ACK`;
`0x84 CAL_IMU` / `0x07 CAL_IMU_ACK`. Cả hai request mang epoch cùng sequence
TELEMETRY nguồn; reply mang epoch, sequence request và mã kết quả.

## 2. Bắt tay và vận hành

1. LF gửi `HELLO (1)` mỗi giây khi chờ liên kết. Payload `<IHHI>`:
   `epoch:u32, telemetry_hz:u16=200, policy_hz:u16=100, schema:u32=0x31524348`.
2. ESP32 tắt enable khi gặp epoch mới và trả `HELLO_ACK (2)` với payload y hệt.
3. ESP32 gửi TELEMETRY ở **200 Hz / 5 ms**, kể cả khi disarmed. Sensor phải được
   hiệu chuẩn và q ở tư thế home trước ARM. Thời gian HOME bắt đầu ở sample đầu sau ARM.
4. Người vận hành gửi lệnh cục bộ vào runtime trên LF qua Unix socket
   `/tmp/rb-hybrid-command.sock` (mode `0600`): `ARM`, `DISARM`, `RESET`,
   `ESTOP`, `RESET_ENCODERS`, `CAL_IMU` hoặc `VELOCITY vx wz`. Socket này không
   phải HCR1; runtime đổi riêng RESET_ENCODERS và CAL_IMU thành request HCR1
   chuyên biệt. Không tự ARM khi boot/reconnect.
5. LF gửi TARGET ở 200 Hz: logic được cập nhật 200 Hz; action policy đổi 100 Hz,
   giữ nguyên giữa hai packet. Servo fixture trong HIL tích phân 2000 Hz;
   task service MCU được yêu cầu đánh thức 2000 Hz. Hai con số này không chứng minh
   bus/driver hoặc vòng điều khiển motor vật lý đạt 2000 Hz.
6. Khi LF báo lỗi hoặc hết TTL, ESP32 bỏ enable theo cơ chế an toàn phần cứng.
   RESET tạo epoch mới, bắt tay lại, ARM lại. Lỗi được latch, DISARM không xóa lỗi.
   E-stop/lỗi driver cục bộ trên ESP cần CLEAR_FAULT sau verified OFF; epoch
   mới không tự xóa hai lỗi này.

LF giữ home 3 s; chạy kế hoạch gập từ 3 s; warm-up model từ 4,775 s; bàn giao từ 5,275 s
(`warmup_start_s`, `handoff_earliest_s` trong `highlv/config/runtime.json`) khi đã gập xong (`Folded`)
và đã có ít nhất 50 lượt warm-up. GRU không reset khi bàn giao.
Không thêm blend giữa output logic/policy vào controller đã đóng gói.

ESP32 vừa reboot có thể gửi HELLO với `epoch=0, hz=200/100, schema=HCR1` để yêu cầu
phiên mới, không cần biết sequence/epoch cũ. LF lập tức disable và cấp epoch mới.
Codec `hcr_request_session()` hỗ trợ trường hợp này. Watchdog phía MCU latch lỗi:
gói target đến lại sau khi timeout không tự bật motor; cần RESET/epoch mới rồi ARM.

## 3. TELEMETRY (3): ESP32 → LF

Payload 128 byte: `<III29f>`.

| Offset payload | Kiểu | Trường |
|---:|---|---|
| 0 | u32 | epoch |
| 4 | u32 | validity/fault flags |
| 8 | u32 | sequence của TARGET **đã thực sự được servo áp dụng** |
| 12 | 29 × float32 | dữ liệu theo bảng dưới |

| Float index | Dữ liệu | Đơn vị |
|---|---|---|
| 0–2 | gyro body `[wx,wy,wz]` | rad/s |
| 3–5 | projected gravity body `[gx,gy,gz]` | vector đơn vị, đứng thẳng `[0,0,-1]` |
| 6–8 | accelerometer specific force body `[ax,ay,az]` | m/s²; đứng thẳng tĩnh z≈+9.81 |
| 9–14 | q `[LH,RH,LK,RK,LW,RW]` | rad; góc bánh unwrap |
| 15–20 | qd cùng thứ tự | rad/s |
| 21–22 | generalized hip torque `[L,R]` | Nm, `tau_hip_motor + tau_crank_motor` |
| 23–28 | mục tiêu servo đang áp dụng `[hipL,hipR,phiL,phiR,wheelL,wheelR]` | rad cho 4 đầu; rad/s cho 2 cuối |

Các góc q là tọa độ MJCF đã homing/đổi chiều, **chưa trừ q_default**.
Nếu encoder đo motor crank: `q_knee = phi - q_hip`, `qd_knee = phidot - qd_hip`.
LF tự tạo observation 32D, trừ default `[-.7,-.7,.9,.9]`, normalize đúng backend,
và giữ GRU 512D. Không normalize thêm trên ESP32.

`applied_target` phải phản ánh mục tiêu cuối cùng đang áp dụng, gồm wheel reference
sau bộ lọc của recovery. LF dùng nó để tạo `last_action` trong warm-up. Khi policy
đã sở hữu điều khiển, `last_action` là action policy bước trước.

Flags:

| Bit/mask | Ý nghĩa |
|---|---|
| 0 / 1 | IMU/attitude hợp lệ, mới |
| 1 / 2 | toàn bộ encoder hợp lệ, mới |
| 2 / 4 | hiệu chuẩn/homing hoàn tất |
| 3 / 8 | emergency stop |
| 4 / 16 | hip torque có dữ liệu đáng tin cậy |
| 5 / 32 | applied_target có dữ liệu hợp lệ |
| 6 / 64 | lỗi driver |

Ba bit đầu bắt buộc. Bit 5 bắt buộc từ khi warm-up. Nếu chưa có current/torque,
đặt hai torque bằng 0 và bỏ bit 4; LF bỏ qua torque.
Không khai báo torque hợp lệ bằng dữ liệu giả.

## 4. TARGET (0x81): LF → ESP32

Payload 52 byte `<IIHH10f>`:

| Offset | Trường |
|---:|---|
| 0 | epoch:u32 |
| 4 | sequence TELEMETRY đã dùng để tính lệnh:u32 |
| 8 | profile:u16 |
| 10 | enables:u16: bit 0 legs, bit 1 wheels |
| 12 | target[6]: hipL, hipR, **crankL, crankR**, wheel_speedL, wheel_speedR |
| 36 | feedforward[4]: tốc độ hipL,hipR,kneeL,kneeR, rad/s |

`target[0:4]` là góc tuyệt đối sau homing trong tọa độ model, đơn vị rad;
`target[4:6]` là vận tốc góc bánh, rad/s. Đây không phải góc delta hay torque.
Hai góc crank là `phi_ref`, trong khi TELEMETRY.q[2:4] là knee hinge:
`q_knee = phi_measured - q_hip_measured`. Với driver thực đo/điều khiển crank,
gửi `phi_ref` sau đổi chiều/mốc góc; không tự thay bằng `phi_ref-hip_ref`.
Hiệu `phi_ref-hip_ref` chỉ là knee reference trong tọa độ khớp tương đối.

Feedforward là vận tốc của **hip và knee hinge**, không phải torque và không
phải trực tiếp crank velocity. Nếu một lệnh motor cần crank velocity thì
`phi_dot_ff = hip_dot_ff + knee_dot_ff` sau ánh xạ đúng tọa độ.
Nhánh HOME/POLICY hiện gửi bốn feedforward bằng 0; nhánh RECOVERY cung cấp chúng.

Adapter HAL Lumi hiện dùng encoder ID3 đo knee trực tiếp (`coupling=0`): MCU đổi
`driver_ref=phi_ref-hip_ref`, `driver_ff=knee_ff`, và phản hồi knee trực tiếp sau
sign/zero. `applied_target[2:4]` vẫn là crank ảo `hip_quantized+knee_quantized`.
Nếu driver đo crank (`coupling=1`) thì dùng phi/phi_dot trực tiếp như mô tả trên.
Đây là chi tiết HAL, không đổi layout hoặc ý nghĩa HCR1.

Metadata profile/enables/sequence/TTL đi kèm không làm thay đổi sáu target chính.

| Profile | Nghĩa |
|---:|---|
| 0 DISABLED | bỏ enable; các số 0 **không phải yêu cầu chạy motor về góc 0** |
| 1 HOME_PD | giữ tư thế home bằng profile PD của model |
| 2 RECOVERY | target logic C++ và velocity feedforward |
| 3 POLICY_PD | target được giải mã từ model |

Chỉ cập nhật lệnh khi CRC đúng, đúng epoch, seq mới, float finite, profile hợp lệ.
TTL cố định **20 ms** từ lúc nhận gói hợp lệ; watchdog chạy trên task motor của ESP32,
không phụ thuộc task UART hoặc LF. Gói TARGET tham chiếu sample cũ/không tồn tại phải bị bỏ.
LF cũng kiểm tra ACK của TARGET; quá 30 ms không có ACK mới thì FAULT.

**Đáp ứng servo tham chiếu đã dùng trong mô phỏng/HIL:** các gain và torque cap
dưới đây mô tả cách fixture tạo lực cho MuJoCo từ góc/vận tốc target. Chúng không
phải các trường torque được LF truyền trên UART, cũng không chứng minh driver
thật bắt buộc phải dùng torque mode. Tầng dưới có thể dùng MIT/position/velocity
interface phù hợp, nhưng phải đối chiếu gain, giới hạn, trễ và đáp ứng thực tế;
chưa có bằng chứng các default của BL4012e tương đương fixture này.

Không đổi gain/profile hoặc thêm limiter âm thầm rồi coi là cùng controller:

- HOME/POLICY: hip PD Kp=30,Kd=1, cap ±1.5 Nm; crank PD Kp=30,Kd=1, cap ±5 Nm;
  wheel velocity gain .004 Nm/(rad/s), cap ±.13 Nm. Feedback crank là `hip+knee`.
  Không tự tanh/clamp action policy trong LF; MCU phải báo lỗi nếu target không thể áp dụng.
- RECOVERY: controller gốc dùng servo theo tọa độ hip/knee. Knee reference lấy
  `phi_ref-hip_ref`. Outer position gain 15 tạo velocity reference cộng feedforward,
  giới hạn ±5 rad/s. Inner PI velocity Kp=.8, Ki=5, anti-windup khi saturation;
  cap generalized hip ±6.5 Nm, knee ±5 Nm; lag applied torque .005 s trong simulator.
  Chuyển sang motor: `crank=clip(tau_knee,±5)`, `hip_motor=clip(tau_hip-crank,±1.5)`.
- Simulator lịch sử có wheel delay10tick ×5ms. Runtime LF hiện đã bỏ FIFO này
  khi đồng bộ trajectory Lumi; MCU không thêm delay50ms. Recovery reference
  ở MCU vẫn lọc first-order tau=.075s và phản hồi trong TELEMETRY.applied_target.
- Mô hình SimBl4012Wheel còn có settle/torque lag, ma sát và đường torque-speed.
  Đây là mô hình actuator được dùng khi kiểm tra, không phải yêu cầu cộng ma sát giả
  vào motor thật. Đối chiếu chế độ điều khiển/gain sẵn có trong driver trước khi ghép servo.

`tests/mcu_servo_reference.cpp` tái hiện servo/plant phục vụ HIL. Firmware ESP32 thật
phải nối các target/profile này với HAL motor và kiểm tra đáp ứng thực tế.
BL4012e hiện ghi wheel speed ở `0x20` bằng RPM; quy đổi từ TARGET là
`rpm = omega_rad_s * 60/(2*pi)`, sau đó áp chiều motor. Lệnh thanh ghi `0x18`
Torque Enable là bật/tắt tạo lực, không phải giá trị torque mong muốn.

Triển khai MCU BL4012e hiện có: `BlMotorBackend`/`EspBlMotor`, UART2/GPIO15
trái và UART1/GPIO17 phải. Backend xử lý Pending/Applied/Fault nội bộ, không đổi
layout HCR1. ACK đợi batch của cả hai bus; applied_target là reference sau
quantization và lọc recovery. Chi tiết gains/zero/giới hạn và phần chưa được
kiểm trên motor thật ở [MOTOR_INTEGRATION.md](MOTOR_INTEGRATION.md).

## 5. STATUS (0x82)

Payload `<IIII>`: `epoch, mode, fault_code, rejected_frames`.
Mode: 0 WAIT_LINK, 1 DISARMED, 2 HOME, 3 STANDUP, 4 WARMUP, 5 MODEL, 6 FAULT.
Fault: 1 telemetry timeout, 2 invalid telemetry, 3 driver/E-stop, 4 clock/sample gap,
5 missing applied_target, 6 logic fault, 7 policy fault, 8 inference quá hard deadline,
9 fall >70°, 10 transport/service stop, 11 telemetry processing >15 ms, 12 ACK timeout.

Policy giữ lịch 100 Hz. Một kết quả inference đến sau 10 ms bị bỏ, giữ target/action
đang áp dụng; hidden vẫn ghi nhận observation vừa xử lý. Không bàn giao bằng prediction
đến muộn. Số lần trễ được ghi vào status JSON. Nếu inference >15 ms hoặc trễ 3 lần
liên tiếp, LF chuyển FAULT. Đây là xử lý jitter của Linux/NPU dùng chung; không hạ
tần số policy xuống 50 Hz. TTL 20 ms độc lập trên MCU vẫn có hiệu lực nếu LF bị treo.

LF logs thêm lý do, phase (Idle/Folding/Folded), số inference và latency trong
`/tmp/rb-hybrid-status.json`. STATUS không được dùng để kéo dài TTL motor.


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

LF gửi payload `<II>` gồm epoch và source telemetry mới (≤30 ms). ESP chỉ nhận
khi không armed/enabled, motor backend xác nhận OFF, telemetry nguồn còn mới và
không có lỗi driver/E-stop. Reply `<III>` gồm epoch, request sequence và kết quả:
`1` đã nhận, `2` motor chưa OFF, `3` source cũ, `4` trạng thái từ chối, `5` đang
bận. ACK xác nhận bắt đầu thu mẫu; ESP yêu cầu ít nhất 500 mẫu và 1 s ổn định.
LF báo hoàn tất khi IMU_VALID xuất hiện, hoặc lỗi sau 5 s. Lệnh không di chuyển
motor, không xóa lỗi driver/E-stop và không ARM.

## 6. Mã tích hợp ESP32

- `mcu/hcr_protocol.h`: encode/decode frame, CRC, payload, parser byte stream.
- `mcu/hcr_link.h`: handshake, target validation, epoch/sequence và TTL.
- Một task UART sở hữu parser/link; một task motor đọc target snapshot atomically.
  Chỉ ACK sau khi motor task thực sự áp dụng lệnh. Cờ E-stop ưu tiên cao nhất.
- Sensor task lấy gyro/encoder đồng bộ, attitude estimator tạo gravity; gửi telemetry
  theo timer 5 ms. Không gửi `last_action`, GRU hoặc simulation state/ground-truth.

Thông lượng: telemetry 152 byte × 200 Hz ≈30.4 kB/s; target 76 byte ×200 Hz ≈15.2 kB/s.
921600 baud 8N1 cung cấp ≈92.16 kB/s theo mỗi chiều, còn chỗ cho STATUS.


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
