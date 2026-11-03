# Bản chất luồng điều khiển tầng dưới — từ TARGET đến driver động cơ

Tài liệu này giải thích **cách `lowlv` biến một frame TARGET của HCR1 thành byte
trên bus động cơ**, và cơ chế nào đảm bảo motor tắt khi mọi thứ trục trặc. Đây
là tài liệu đọc-hiểu, không thay thế các tài liệu đặc tả:

- Frame và payload: [HCR1_PROTOCOL.md](HCR1_PROTOCOL.md)
- Ánh xạ motor, lịch sử sửa lỗi bus: [MOTOR_INTEGRATION.md](MOTOR_INTEGRATION.md)
- Tổng quan MCU, console, lịch UART: [MCU.md](MCU.md)
- Đối chiếu luồng LF–ESP–motor: [FLOW_AUDIT.md](FLOW_AUDIT.md)

## Phạm vi và độ tin cậy của tài liệu

Nội dung được đọc từ mã nguồn trong working tree ngày 2026-11-03 (gồm các thay
đổi chưa commit lúc đó). Các file đã đọc:

`src/main.cpp`, `src/core/GatewayCore.cpp`, `include/gateway/core/GatewayCore.hpp`,
`include/gateway/core/TaskSchedule.hpp`, `include/gateway/motor/MotorBackend.hpp`,
`include/gateway/motor/BlMotor.hpp`, `src/motor/BlMotor.cpp`,
`src/motor/EspBlMotor.cpp`, `include/gateway/motor/EspBlMotor.hpp`,
`include/gateway/motor/hal/MotorFrameCodec.hpp`,
`include/gateway/motor/hal/MotorMapping.hpp`, `third_party/bl4012e/src/joint_bus.c`.

Phân biệt ba loại phát biểu:

| Loại | Ý nghĩa | Ví dụ |
|---|---|---|
| **Từ mã nguồn** | Đọc trực tiếp từ code | công thức đổi xung, thứ tự gửi bus |
| **Từ chú thích** | Lấy từ comment trong code, chưa đo | driver tính `torque = Kp·(pos−đo)+Kd·(rpm−đo)+tau`; driver khớp trả lời sau ~500 µs |
| **Chưa kiểm** | Không được đọc hoặc không có bằng chứng | mạch bên trong driver BL4012, thời gian thực đo trên robot |

Tài liệu này **không** là kết quả build, test hay đo phần cứng. Mã chưa được
build PlatformIO hay chạy test cho tài liệu này.

---

## 1. Bản chất trong một đoạn

Tầng dưới không "nhớ" lệnh. Mỗi TARGET hợp lệ chỉ **thuê** motor thêm 20 ms. Hết
hạn mà không có TARGET mới thì motor bị tắt. Mọi thứ còn lại làm ba việc:

1. **Chặn** mọi TARGET không đủ điều kiện (epoch, sequence, độ mới, cảm biến, HOME).
2. **Chuyển** góc/tốc độ thành lệnh bus mà **không bao giờ vượt hạn** (hạn kiểm
   ở nhiều lớp, kể cả khi task chính bị đứng).
3. **Xác nhận** (`applied_seq`) chỉ khi **cả hai bus** đã được driver trả lời hợp lệ.

ESP không điều khiển torque. Nó nạp **điểm đặt và độ cứng** (`pos`, `rpm`, `kp`,
`kd`) cho vòng PD nằm trong driver. Driver tự đóng vòng.

---

## 2. Bức tranh tổng thể

```text
 LF ── UART0 (TX43/RX44, 921600) ──► hcr-uart ──► rx_queue ──► hcr-service ──► GatewayCore
                                                                                   │
                                                                         MotorBackend (BlMotorBackend)
                                                                           │ request_[trái/phải].batch
                                                          ┌────────────────┴────────────────┐
                                                    motor-left worker                 motor-right worker
                                                    UART2 / GPIO15                    UART1 / GPIO17
                                                    wheel ID1, hip ID2, knee ID3      wheel ID1, hip ID2, knee ID3
                                                          │                                  │
                                                    joint_bus.c (1 dây, 1 Mbaud)       joint_bus.c
```

`GatewayCore` không biết gì về phần cứng cụ thể. Nó chỉ nói chuyện với giao diện
`MotorBackend`. Worker là phần duy nhất ghi bus.

### 2.1 Bốn luồng chạy trên core 1

Định nghĩa trong `TaskSchedule.hpp`, với
`static_assert(service > uart > motor)`.

| Luồng | Ưu tiên | Việc |
|---|---:|---|
| `hcr-service` | 20 | `esp_timer` đánh thức mỗi 500 µs (2 kHz). Chạy toàn bộ `GatewayCore`, đọc IMU, xử lý hàng đợi. |
| `hcr-uart` | 19 | Sở hữu UART0 và parser. Đọc byte → `rx_queue`; lấy `tx_queue` → ghi ra dây. |
| `motor-left`, `motor-right` | 18 | Mỗi luồng lặp `worker.step(); vTaskDelay(1)` cho một bus. |
| `loop()` (Arduino) | thấp | Console, Wi-Fi, OTA, in `STATUS`. Ngoài lịch điều khiển. |

Thứ tự ưu tiên có chủ ý: watchdog phải chen được mọi thứ; UART phải chen được
motor để telemetry/TARGET không bị chậm. Các luồng nối với nhau bằng hàng đợi cố
định (`rx_queue` 32, `tx_queue` 16, `command_queue` 8) và một snapshot nhỏ dưới
khoá `MotorCritical` — không có khoá dài, không cấp phát heap trong core.

---

## 3. Một tick của service (500 µs)

Thứ tự trong `service()` (`main.cpp`):

```text
 1. transport_fault / estop_requested (từ UART task, console) → core.stop / core.estop
 2. core.tick(now)                         ← LUÔN chạy trước mọi I/O khác
 3. đọc IMU (SPI) → cập nhật hướng trọng lực → core.setImu
 4. lấy tối đa 8 sự kiện từ rx_queue:
        event.sent  → core.telemetrySent(...)       (telemetry đã THỰC SỰ ra dây)
        ngược lại   → core.receive(frame, now, received_us) → xếp gói trả lời vào tx_queue
 5. yêu cầu calib IMU (HCR) và lệnh console (ESTOP, CLEAR_FAULT, MOTOR_HOME, CAL_IMU...)
 6. mỗi 500 ms: HELLO nếu chưa có phiên | mỗi 5 ms: TELEMETRY | mỗi 100 ms: EXECUTION_DIAG
```

Bước 2 nằm trước bước 4 nên một TARGET cũ nằm trong hàng đợi không thể kéo dài
hạn 20 ms. Hạn tính từ `received_us` do UART task đóng dấu lúc parse xong
(`GatewayCore::receive`: `deadline_ = received_us + HCR_TARGET_TTL_US`), không tính
từ lúc service xử lý.

### 3.1 `GatewayCore::tick`

```text
 enabled_ && now ≥ deadline_                → stop(TargetTimeout)
 connected_ && 500 ms không nghe LF         → stop(Transport); reset_pending_ = true (xin epoch mới)
 feedback_ = motor_.feedback(now)
 feedback_.estop                            → stop(Estop)
 feedback_.driver_fault                     → stop(Driver)
 armed_ && (!motor_.ready() || !freshSensors) → ghi sensor_fault_; stop(Sensors)
 !enabled_  → chỉ chốt ACK của lệnh DISABLED nếu motor_.disabled(); return
 enabled_   → motor_.apply(target_, seq, deadline_, now, applied)
                Pending → return
                Fault, hoặc applied sai epoch/seq → stop(Driver)
                Applied → applied_ = applied.target; applied_seq_ = applied.seq
```

---

## 4. Cổng nhận TARGET (`GatewayCore::receive`)

Một TARGET chỉ được nhận khi qua **tất cả** các cổng sau; thiếu một cổng thì
đếm `rejected_` và bỏ:

| # | Điều kiện |
|---|---|
| 1 | Không ở chế độ bảo trì OTA. |
| 2 | Gói không quá cũ: `received_us ≤ now` và `now − received_us < 20 ms`. |
| 3 | Đã có phiên (`connected_`), không `reset_pending_`, `epoch` đúng. |
| 4 | `seq` mới hơn `rx_seq_` (so theo nửa khoảng uint32) và `stamp_us` không lùi. |
| 5 | CRC/độ dài/số hữu hạn hợp lệ (`hcr_target`); `profile == DISABLED` thì `flags == 0`. |
| 6 | Với profile khác DISABLED: không có lỗi cục bộ (`fault_ == None`), `motor_.ready()`, `freshSensors`, `sourceValid(source_seq)`, và STATUS gần nhất của LF có mode trong 2..5. |
| 7 | Lần đầu (chưa armed): motor đang OFF **và** bốn khớp hip/knee trong ±0,15 rad quanh HOME; khi đó `armed_ = true`. |

Điều kiện `sourceValid` là một ý tưởng đáng chú ý. ESP chỉ ghi `seq → stamp` vào
`history_` (32 phần tử) **sau khi telemetry đã ra khỏi UART** (`telemetrySent`).
Vì vậy TARGET chỉ được nhận nếu nó được LF tính từ một telemetry mà ESP chắc chắn
đã gửi và còn mới (≤30 ms). Điều này chặn việc LF tính lệnh từ dữ liệu cũ.

Sau khi qua cổng:

- `DISABLED`: `motor_.disable()`, `enabled_ = applied_valid_ = armed_ = false`,
  `disable_ack_pending_ = true`; `applied_seq_` chỉ được đặt khi
  `motor_.disabled()` xác nhận OFF thật.
- Khác: lưu `target_`, `target_seq_`, `enabled_ = true`, `deadline_ = received_us + 20 ms`.

Các nhánh khác của `receive`: HELLO (mở phiên/epoch), STATUS của LF
(`mode == 6` hoặc có fault → `stop(RemoteFault)`; `mode ≤ 1` → `stop()`),
`ENCODER_HOME` (chụp HOME, không bao giờ làm mới hạn hay ARM), `CAL_IMU`
(không bao giờ bật motor hay làm mới hạn).

---

## 5. Đường đi của góc target xuống driver

Điểm xuất phát: `target_` trong `GatewayCore` chứa
`[hipL, hipR, crankL, crankR, ωL, ωR]`: bốn góc tuyệt đối (rad, hệ model, sau
HOME) và hai tốc độ bánh (rad/s). Chưa có byte nào xuống motor.
Ở tick sau, `GatewayCore::tick` gọi `BlMotorBackend::apply`.

### 5.1 Chặng 1 — tách thành từng động cơ

`makeBatch` chạy cho từng bên `s` (0 = trái, 1 = phải). Mỗi bus có ba motor:
wheel ID 1, hip ID 2, knee ID 3 (`MotorMapping.hpp`).

```text
 hip  = target[s]                 góc hông tuyệt đối
 knee = target[2+s] − hip         TARGET mang "crank" (= hip + knee) nên trừ hip để ra góc gối
 ω    = target[4+s]               tốc độ bánh (rad/s), không phải góc
```

Việc trừ hip có hiệu lực vì driver gối **đo trực tiếp khớp gối**
(`kKneeCrankCoupling = 0`, đo trên robot 2026-11-01). Nếu driver đo crank thì
mới gửi `phi` thẳng. Hằng số này quyết định `target = knee + coupling·hip`.

Kiểm tra miền (khi bit chân `flags & 1` bật):

| Khớp | Miền hợp lệ (rad) |
|---|---|
| hip | −1,56817833 … 2,35706715 |
| knee | 0 … 1,94778745 |

Ngoài miền → `makeBatch` trả `false` → backend latch `MotorError::Range` →
`ApplyResult::Fault` → `stop(Driver)`. **Không có clamp ngầm.**

Bit bánh (`flags & 2`) tắt thì `ω = 0`. Bit chân tắt thì khớp bị nhả (§5.6).

### 5.2 Chặng 2 — đổi radian sang xung của driver

Với khớp `j` (0 = hip, 1 = knee) của bên `s`:

```text
 pos_counts = sign[s][j] · (target − zero_[s][j]) / count_rad          count_rad = 2π / 16384 ≈ 3,835e-4 rad
```

| Thành phần | Ý nghĩa |
|---|---|
| `sign` | Dấu lắp đặt. Hip: trái +1, phải −1. Knee: trái +1, phải −1 (`kLegBindings`). |
| `zero_` | Hằng bù, chụp **một lần** lúc `MOTOR_HOME`: `zero = q_home − sign·raw_encoder`. Nối "góc model" với "số đếm encoder" mà không cần biết gốc encoder. |
| kết quả | **int16**, ±32767 xung ≈ ±12,57 rad (~2 vòng). Ngoài dải → `quantize()` từ chối. |

Ví dụ (hông trái; encoder lúc chụp HOME đọc 0,5 rad; HOME = 2,35706715 rad):

```text
 zero_ = 2,35706715 − 1·0,5 = 1,85706715
 target hip = 2,0 rad
 pos_counts = (2,0 − 1,85706715) / 3,834952e-4 = 372,7 → 373 xung
 giá trị thực sự áp = 1·373·3,834952e-4 + 1,85706715 = 2,000111 rad
```

Con số **đã lượng tử hóa** (2,000111) mới được ghi vào `applied_target` báo lại
cho LF, không phải 2,0. Cùng cơ chế đó cho `applied_target[2+s]` (crank ảo =
`hip_lượng_tử + knee_lượng_tử`).

### 5.3 Chặng 3 — đóng gói "khối MIT" 10 byte

Cái gửi xuống không phải "chạy tới góc rồi dừng", mà là bộ tham số của vòng PD
trong driver:

| Trường | Đơn vị trên dây | Giá trị | Nguồn |
|---|---|---|---|
| `kp` | 0,001 N·m/rad (u16) | `gain.kp × 1000`. Hip 10 → 10000; knee 20 → 20000 | `BlMotorConfig::profile` / `knee_profile`; phải ≤ `kp_max` do driver báo (thanh ghi 0x32) |
| `kd` | 0,001 N·m·s/rad (u16) | `gain.kd × 1000`. Hip 0,036 → 36; knee 0,05 → 50 | tương tự |
| `pos` | xung (i16) | 373 (ví dụ trên) | chặng 2 |
| `rpm` | vòng/phút trục ra (i16) | `sign · ff / (2π/60)`; chỉ khác 0 ở profile RECOVERY | `feedforward[]` của TARGET |
| `tau` | 0,001 N·m (i16) | **luôn 0** | cố định trong `makeBatch` |

Luật do chú thích trong `MotorFrameCodec.hpp` mô tả (loại **từ chú thích**):
`torque = Kp·(pos − đo) + Kd·(rpm − tốc độ đo) + tau`. Với kp = 10 N·m/rad, lệch
0,1 rad sinh khoảng 1 N·m. Driver tự đóng vòng này.

Gains hiện tại: cả ba profile (HOME, RECOVERY, POLICY) dùng cùng bộ — hip
`10 / 0,036`, knee `20 / 0,05` — theo `BlMotorConfig` (chú thích "Lumi-Quantum
BoardConfig").

`rpm` (feedforward) chỉ có ở RECOVERY, được lấy từ `feedforward[]` của TARGET:
hip dùng `ff[s]`, knee dùng `ff[2+s] + coupling·ff[s]`. HOME và POLICY có FF = 0.

Khối MIT được bọc thành khung kiểu Dynamixel, ghi vào thanh ghi `0x1A`. Frame
ví dụ trên (ID 2, kp 10000, kd 36, pos 373, rpm 0, tau 0) là đúng 17 byte
(đã tính checksum bằng Python):

```text
 FF FF  02   0D   03    1A   10 27  24 00  75 01  00 00  00 00   02
 sync   ID   LEN  WRITE reg  kp     kd     pos    rpm    tau     CHK
              =13       0x1A (10000) (36)   (373)  (0)    (0)    ~Σ(ID..tau)
```

`LEN = số tham số + 2`; `CHK = ~(ID + LEN + INST + các tham số)`.

### 5.4 Chặng 4 — bàn giao cho worker

`makeBatch` ghi vào `request_[s].batch`: khối MIT 10 byte cho từng khớp, tốc độ
bánh, hạn (`deadline_us` = lease hiện tại), `token` mới, `generation`. Nó cũng
lưu `pending_` (giá trị `actual` đã lượng tử hóa). `apply()` trả `Pending` rồi
thôi — **core không bao giờ ghi bus**.

Mỗi `BlMotorWorker::step()` (mỗi ~1 ms):

1. Sao chép `request_[side]` dưới khoá.
2. Nếu `batch.token ≠ state_.completed` và `state_.prepared`: gọi `claimBatch`.
   - `Cancelled`: đã đổi `generation`.
   - `Superseded`: batch này không còn là batch đang chờ.
   - `Expired`: `now ≥ lease_us` hoặc `now ≥ batch.deadline_us`.
   - `Claimed`: ghi `batch_started_[side] = token`.
3. Thứ tự I/O của batch:

   ```text
   [bánh trước]   nếu legs_engaged_ (batch MIT trước đã hoàn tất), cả hai bit chân+bánh bật và bánh đã "settled"
   hip  (ID 2): ghi MIT 0x1A     ← lệnh MIT đầy đủ tự bật torque của khớp
   knee (ID 3): ghi MIT 0x1A
   [bánh sau]     ghi tốc độ 0x20 (nếu chưa gửi ở trên)
   đọc bánh       thanh ghi 0x24 (làm mới encoder bánh)
   ```

   Trước mỗi lệnh gọi `allowed(r, true)`; sai là ngắt và (nếu quá hạn) latch
   `MotorError::Expired`.
4. Hoàn tất: `state_.completed = token`, `legs_engaged_ = (flags & 1)`.

Batch đầu của phiên giữ thứ tự chân-trước-bánh; batch sau khi chân đã được giữ
thì gửi bánh trước, vì phản hồi bánh là phần chậm nhất của chu kỳ.

Khởi động bánh không nằm trong batch TARGET: khi `want_wheel && !on_[2]`, worker
ghi tốc độ 0, bật torque bánh (ghi `0x18 = 1` kèm đọc lại), đợi
`wheel_settle_us` (25 ms), rồi **đọc lại thanh ghi `0x20` để chứng minh bằng 0**
(`verifyWheelZero`) trước khi coi `prepared`.

### 5.5 Chặng 5 — một giao dịch trên dây (`joint_bus.c`)

Bus là **một dây half-duplex, 1 Mbaud**. TX và RX của UART dùng chung một chân GPIO
(`uart_set_pin(uart, pin, pin)`); chân ở chế độ `INPUT_OUTPUT` với pull-up. Mỗi
giao dịch (`exchange` trong `joint_bus.c`):

```text
 1. nếu bus đang "drain" (sau lỗi trước): chờ tới khi đường yên lặng ≥1 tick (tối đa 10 tick)
 2. chờ đủ TURNAROUND_US = 100 µs kể từ lần bus rảnh (để driver trước kịp nhả đường)
 3. permitted(): còn hạn và còn được phép? không thì bỏ
 4. uart_flush_input
 5. send_frame (ngắt tắt, khoá chung s_wire_lock, ≤ TX_HOLD_MAX_US = 1000 µs, thực tế ≤ ~480 µs):
        nối TX vào chân → ghi cả khung vào FIFO → quay tới khi TX idle → NHẢ chân ngay
 6. đọc lại CHÍNH khung vừa gửi ("echo") và memcmp
        thiếu → EchoTimeout; khác → EchoMismatch; cả hai bật cờ drain
 7. chờ phản hồi: cửa sổ khớp 3 ms (RX_TIMEOUT_JOINT), bánh 9 ms (RX_TIMEOUT_WHEEL)
        thiếu đúng 1 byte sync đầu (FF) vẫn được phục hồi và ghi "sync_restored"
 8. kiểm phản hồi: sync FF FF, ID, LEN, checksum, và ba byte PRE_* phải khớp yêu cầu
```

Phản hồi của một lệnh ghi MIT tới khớp (khung "joint reply") dài
`0 + 7 + 4 = 11` byte:
`FF FF ID LEN STATUS POS_L POS_H PRE_LEN PRE_INST PRE_P1 CHK`. Với bánh,
`echo_at = 5` nên phản hồi ngắn hơn, không có vị trí.

Thời gian (loại **từ mã/chú thích**, không phải đo): 17 byte × 10 bit ÷ 1 Mbaud
= **170 µs** gửi; 11 byte phản hồi = **110 µs** nhận. Chú thích trong `joint_bus.c`
nói driver khớp trả lời sau ~500 µs; BL4012 bánh thường ~1,3 ms nhưng khi bị giữ
hoặc có tải thì chậm hơn nhiều.

**Hai lưu ý cơ học.**
- Bước 5 chạy với ngắt tắt trên core này và khoá `s_wire_lock` là **biến tĩnh
  dùng chung hai bus**. Hai bus trái và phải **không bao giờ phát cùng lúc**.
  Phần chờ phản hồi của hai bus thì chồng lên nhau được.
- Hạn của bus (`bus_.deadline_us`) = `min(now + 20 ms, permit.deadline_us)`. Ngoài
  ra `permit.valid` hỏi lại backend giữa giao dịch "lệnh này còn hiệu lực
  không". Nhờ đó một bus đang chờ phản hồi vẫn bị ngắt khi lệnh bị hủy hoặc quá hạn.

Sau khi `joint_bus.c` trả về, `EspMotorBus::request` **kiểm lại bằng codec Lumi**
(`motorReply`/`parseJointReply`). Hai lớp kiểm độc lập.

### 5.6 Khi bit chân tắt hoặc bánh tắt

- Bit chân tắt (`flags & 1 == 0`): khớp không nhận MIT. Nếu đang bật thì
  `torque(j, false)` (OFF có đọc lại), rồi gửi PING để vẫn có encoder tươi.
  `actual.target` giữ giá trị đã hoàn tất trước đó (`completed_`) hoặc, nếu chưa
  có, giá trị đo hiện tại.
- Bit bánh tắt: `torque(2, false)` nếu bánh đang bật.

### 5.7 Bánh xe: tốc độ, không phải góc

`ω` (rad/s) được đổi thành rpm int16 rồi ghi vào thanh ghi `0x20` của ID 1:

```text
 rpm = sign[s][2] · ω / (2π/60)
```

- `|rpm|` phải ≤ `wheel_max_rpm` (950). Vượt → `makeBatch` trả `false` → `Range`.
- Profile RECOVERY: `ω` đi qua bộ lọc bậc nhất `τ = recovery_wheel_tau_s = 0,075 s`
  trước khi lượng tử hóa. `dt` lấy từ lần áp dụng trước (mặc định 5 ms).
- Driver bánh có thể không trả lời lệnh ghi (response level < 2). Khi đó ESP chỉ
  thấy echo, trả `BusError::WriteEchoOnly` (không bao giờ bịa ACK), rồi worker
  **đọc lại thanh ghi `0x20`** để chứng minh giá trị đã vào (`wheelSpeed`).
- Nếu cùng rpm đã được xác nhận trong vòng 100 ms của cùng phiên bật, lệnh ghi
  được bỏ qua (`skipped_writes`); encoder bánh vẫn được đọc ở mỗi batch.

---

## 6. Phản hồi quay ngược lên và nghĩa của "đã áp dụng"

### 6.1 Mỗi lệnh ghi cũng là một lần đọc encoder

Phản hồi của lệnh MIT chứa **vị trí hiện tại** của khớp. `jointSample` lấy giá
trị này:

```text
 raw_q = position × count_rad                     (rad, thô)
 raw_qd = Δposition × count_rad / Δt              (rad/s, sai phân theo tem thời gian)
 |Δposition| > 16384 xung giữa hai mẫu  → FeedbackError::JointJump → fail
```

Bánh đọc riêng bằng thanh ghi `0x24` (vị trí 10 bit, tốc độ, tải). Vị trí bánh
được unwrap theo từng mẫu; khoảng cách hai mẫu quá `feedback_age_us` hoặc nhảy
đúng 512 xung (mơ hồ) → lỗi `WheelGap`/`WheelAmbiguous`.

### 6.2 STATUS khác 0 là driver tự cắt

`reply.status ≠ 0` (quá nhiệt `0x04`, quá tải `0x20`...) → `fail(MotorError::Status)`
→ `trip` → cả hệ thống tắt và khoá lỗi.

### 6.3 ACK = driver đã nhận và trả lời hợp lệ, không phải khớp đã tới góc

Backend chỉ coi batch hoàn tất khi **cả hai bên** có `state_.completed == token`:

```text
 apply():  if (in_flight_ && state_[0].completed == token_ && state_[1].completed == token_)
              completed_ = pending_;  in_flight_ = false;
              (đo applied_latency_max_us = now − pending_at_)
           ...
           if (have_complete_)  out = completed_; return Applied;
```

`GatewayCore` đặt `applied_seq_ = applied.seq` và `applied_ = applied.target`;
telemetry đưa chúng lên LF qua `applied_seq` và `applied_target`. Giao thức
nói rõ: không cần chờ trục tới đích (xem [HCR1_PROTOCOL.md](HCR1_PROTOCOL.md)).

**`applied_seq` có thể tụt sau TARGET mới nhất.** Một TARGET mới thay thế batch
cũ **chỉ khi chưa bus nào claim** nó (`batch_started_[0]` và `[1]` đều bằng 0).
Nếu đã claim thì batch cũ được chạy nốt với tham chiếu và hạn ban đầu. Đây là
hành vi chủ ý; nó được đếm trong `superseded` và hiển thị trong
`EXECUTION_DIAG`.

### 6.4 Feedback → telemetry

`BlMotorBackend::feedback(now)`:

```text
 encoders_valid = cả 6 mẫu tươi hơn feedback_age_us (≤ 20 ms), hữu hạn, không lỗi
 homed          = homed_ && encoders_valid
 q[hip]   = sign·raw_q[0] + zero_[0]
 q[knee]  = (sign·raw_q[1] + zero_[1]) − coupling · q[hip]
 q[wheel] = sign·raw_q[2] + zero_[2]
 qd[...]  tương tự
 torque_valid luôn false (không có cảm biến dòng)
```

`GatewayCore::flags` tạo các cờ từ đó: `IMU_OK` (IMU ≤10 ms), `ENCODER_OK`
(≤20 ms), `CALIBRATED` (`enc && homed && IMU_OK`), `TARGET_VALID`
(`applied_valid_ && enabled_`), `ESTOP`, `DRIVER_FAULT`. Nhóm nào không hợp lệ
thì telemetry để giá trị tạm hữu hạn (gravity = `[0,0,-1]`) và xoá cờ; không bao
giờ gửi dữ liệu giả với cờ bật.

---

## 7. Các lớp chặn thời gian độc lập

| Lớp | Ở đâu | Chặn gì |
|---:|---|---|
| 1 | `GatewayCore::tick`: `now ≥ deadline_` | Không có TARGET mới thì `stop(TargetTimeout)` |
| 2 | `BlMotorBackend::apply`: `now ≥ deadline` | `MotorError::Expired`, huỷ batch |
| 3 | `BlMotorWorker::allowed()` trước mỗi I/O; `permit.deadline_us` trong từng giao dịch bus | Ngắt cả giữa một giao dịch bus. **Chạy được kể cả khi service task bị đứng** |
| 4 | Worker khi `!active`: lặp ép OFF, đọc lại thanh ghi `0x18` | Chỉ coi là tắt khi đọc ra torque-enable = 0 |

Lớp 3 và 4 là chỗ chống việc service task treo. Chú ý ngữ nghĩa:
`disable()` chỉ **yêu cầu** huỷ; chỉ `disabled()` mới **xác nhận** motor đã OFF
thật (cả hai bên `state_.off`, không còn request active, cùng `generation`).
Mã nguồn cố ý không coi "đã gửi lệnh OFF" là "đã tắt".

OFF được thử theo thứ tự bánh → hip → knee (`off()`), và **mọi** motor đều được
thử kể cả khi cái đầu vắng mặt. Worker còn tự nghi ngờ trạng thái OFF đã cache:
cứ 100 ms (khi không bảo trì) nó đặt lại `on_ = {true,true,true}` để phát hiện
driver reset hoặc bị bật từ ngoài.

---

## 8. Khoá lỗi và đường hồi phục

```text
 GatewayCore::stop(reason)  : motor_.disable(); enabled_ = armed_ = applied_valid_ = false;
                              ghi fault_ nếu chưa có (Estop luôn ghi đè)
 BlMotorBackend::trip(e)    : error_ = e (nếu chưa có); homed_ = false; huỷ batch
                              → ready() = false → mọi TARGET sau đó bị từ chối
```

| Lỗi (`LocalFault`) | Xoá bằng |
|---|---|
| `Transport`, `TargetTimeout`, `Sensors`, `RemoteFault` | `newSession` (kết nối lại) |
| `Driver`, `Estop` | **Chỉ** `CLEAR_FAULT` (`clearLocalFault`) khi đã xác nhận motor OFF và không còn E-stop |

Hệ quả:

- Sau lỗi bus/driver, `homed_ = false`; phải `MOTOR_HOME` lại trước khi ARM.
  Thứ tự "HOME rồi mới ARM" luôn bắt buộc.
- `resetFault()` không xoá một lần HOME tốt nếu backend không có lỗi.
- Chế độ bảo trì OTA (`maintenance_`) là chốt vĩnh viễn tới khi reboot; mọi ARM
  và TARGET đều bị chặn.

---

## 9. Hai điều đáng chú ý rút ra từ mã

1. **ESP không kiểm tính liên tục của quỹ đạo.**
   `GatewayCore::receive` chỉ kiểm lúc ARM rằng *feedback* gần HOME (±0,15 rad).
   `makeBatch` chỉ kiểm miền khớp và tốc độ bánh. Một TARGET nhảy từ 0,5 sang
   2,0 rad trong miền hợp lệ vẫn được gửi với kp = 10. An toàn về quỹ đạo nằm ở
   LF (nó giữ HOME 3 s đầu, rồi gập bằng quỹ đạo). Đây là hệ quả của nguyên
   tắc "không clamp ngầm", nhưng cũng có nghĩa ESP không tự chặn được một LF gửi
   bước nhảy lớn trong miền. Mọi thay đổi nhằm "thêm giới hạn bước nhảy" cần
   đánh giá lại hợp đồng HCR1 và phải sửa cả hai tầng.

2. **Thời gian một batch là tổng vài giao dịch, và bánh chi phối nhiều nhất.**
   Mỗi bus gồm: hip, knee, ghi tốc độ bánh, đọc bánh (và có thể đọc lại thanh
   ghi bánh). Cửa sổ chờ của bánh dài tới 9 ms. Đây là thứ quyết định
   `applied_latency` mà cơ chế ACK thích ứng của LF (cửa sổ 30–60 ms) đang đo,
   và là vùng đã gây lỗi echo/reply lúc Folding được ghi trong báo cáo cũ
   (xem [MOTOR_INTEGRATION.md](MOTOR_INTEGRATION.md)). Thời gian này mới được suy
   từ code và chú thích; chưa có số đo mới trong tài liệu này.

---

## 10. Hằng số thường gặp

| Hằng số | Giá trị | Nơi định nghĩa |
|---|---|---|
| TTL TARGET | 20 ms | `HCR_TARGET_TTL_US` |
| Tuổi nguồn tối đa của TARGET (`source_seq`) | 30 ms | `GatewayCore::sourceValid` |
| Mất LF → tắt motor | 500 ms | `GatewayCore::tick` |
| Tuổi encoder / IMU để coi hợp lệ | ≤20 ms / ≤10 ms | `GatewayCore::flags`, `feedback_age_us` |
| Chu kỳ service | 500 µs | `main.cpp` |
| Chu kỳ telemetry / HELLO / diag | 5 ms / 500 ms / 100 ms | `main.cpp` |
| Gói UART chờ quá 10 ms trong hàng đợi TX | bỏ và `transport_fault` | `pumpUart` |
| Xung mỗi vòng khớp / dải int16 | 16384 / ±32767 (≈ ±12,57 rad) | `MotorFrameCodec.hpp` |
| `wheel_max_rpm` | 950 | `BlMotorConfig` |
| `wheel_settle_us` | 25000 | `BlMotorConfig` |
| `TURNAROUND_US` / `RX_TIMEOUT_JOINT` / `RX_TIMEOUT_WHEEL` | 100 µs / 3 ms / 9 ms | `joint_bus.c` |
| Baud bus motor | 1 000 000 | `EspMotorBus::begin` |
| HOME hip | 2,35706715 rad | `main.cpp`, `GatewayCore.cpp`, `BlMotorConfig::hip_max` |

HOME (2,35706715 rad) xuất hiện cứng ở nhiều chỗ (`GatewayCore.cpp` hai lần,
`main.cpp` lệnh `MOTOR_HOME`, và phía `highlv`). Đổi tư thế HOME phải sửa tất cả.

---

## 11. Cách tự kiểm lại các phát biểu trong tài liệu

Các lệnh dưới đây chạy trên máy host, không cần phần cứng (xem
[../README.md](../README.md)). Chưa chạy chúng khi viết tài liệu này.

```sh
python3 tests/check_gateway.py        # GatewayCore, parser, console
python3 tests/check_bl_motor.py       # BlMotorBackend + worker với bus giả
python3 tests/check_inherited_bus.py  # joint_bus.c so với bản gốc
python3 -m pytest -q tests
```

`tests/bl_motor_tests.cpp` (biên dịch cùng `BlMotor.cpp` và `GatewayCore.cpp` bởi
`check_bl_motor.py`, có ASan/UBSan) là nơi đối chiếu §5.3–§5.4 với mã chạy thật:
nó kiểm khung ghi MIT mẫu của vendor qua `motorPacket` (checksum `0x69`), kiểm
các trường `kd`/FF của khung MIT mà worker gửi cho hip và knee từng bên, và kiểm
thứ tự các lệnh trên bus giả. Nó **không** dựng đúng ví dụ 373 xung ở §5.2–§5.3;
ví dụ đó được tính riêng bằng Python khi viết tài liệu, không phải từ test.
