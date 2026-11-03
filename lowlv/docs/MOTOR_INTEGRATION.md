# Luồng động cơ HCR–BL4012e — cập nhật HAL Lumi 2026-11-02

Backend motor dùng `BlMotorBackend`/`EspBlMotor`. Bản ứng viên hiện tại **biên dịch
trực tiếp module `joint_bus.c` kế thừa từ BL4012e/3in1**, qua adapter C++ mỏng;
bản upstream mới dùng FIFO TX + nhả pad khi TX idle, gap100us và drain late reply.
Adapter không tự thêm OEN/RMT; deadline HCR được đưa xuống module. Build dùng Arduino 3.3.0 / ESP-IDF 5.5.0.
Xem [bản sửa và mức kiểm chứng](MISS_RECOVERY_VALIDATION.md),
[nguồn module và delta](../third_party/bl4012e/PROVENANCE.md).

Các kết quả nạp/đo trong lịch sử thuộc firmware trước. Bản kế thừa này chưa được
nạp hoặc thử chuyển động. Không coi các lần đọc encoder khi OFF thành công là
bằng chứng lỗi khi bắt đầu gập chân đã được sửa.

## Ánh xạ đã chốt

Người dùng đính chính: **UART2 là chân trái, UART1 là chân phải**.
Mapping và gain hiện lấy từ HAL Lumi-Quantum, revision
`7b86053c89620484a3c99e869cce3c23dd39d37f`.
[Provenance và các khác biệt](../include/gateway/motor/hal/PROVENANCE.md).
Hip dùng Kp=10, Kd=.036; gối Kp=20, Kd=.05, tách gain theo motor/profile.
Mỗi worker vẫn đọc giới hạn 0x32; không clamp gain để che cấu hình sai.
Lumi ghi nhận ID3 đo gối trực tiếp: hip thay đổi 3.15 rad trong phép đo,
encoder gối thay đổi dưới .25 rad. Đây là bằng chứng từ Lumi, chưa đo lại với
firmware thích nghi này.

| Kênh HCR | Motor | Bus DATA | ID | Dấu driver / tọa độ dùng trong backend |
|---|---|---|---:|---:|
| target[0], hip trái | hip | UART2 / GPIO15 | 2 | +1 |
| target[1], hip phải | hip | UART1 / GPIO17 | 2 | -1 |
| target[2], crank trái | gối (chuyển phi → knee) | UART2 / GPIO15 | 3 | +1 |
| target[3], crank phải | gối (chuyển phi → knee) | UART1 / GPIO17 | 3 | -1 |
| target[4], bánh trái | wheel | UART2 / GPIO15 | 1 | +1 |
| target[5], bánh phải | wheel | UART1 / GPIO17 | 1 | -1 |

GPIO16/18 là pad RX nối cùng DATA trong PCB, được đặt input-floating. Hai bus
motor 1 dây 1 Mbaud/8N1; UART0 TX43/RX44 vẫn là HCR1 921600/8N1 tới LF.
MPU6500 dùng GPIO10–14, không trùng chân motor.

Dấu khớp theo tọa độ model trong Lumi: hip trái/phải +/−, gối trái/phải +/−,
bánh trái/phải +/−. `MOTOR_HOME` xác lập zero từ tư thế model đã biết sau mỗi
boot. Không dùng trực tiếp chiều gập của GUI cho tọa độ gối model.
TARGET HCR vẫn chứa crank ảo `phi=hip+knee`; adapter dưới chuyển sang gối ID3.

## Luồng chạy

```text
LF TARGET [hipL hipR phiL phiR omegaL omegaR], profile/enables/FF
  -> GatewayCore: CRC, epoch, source history, TTL 20 ms, ARM/sensors
  -> BlMotorBackend: ánh xạ, kiểm range, tạo batch cùng sequence cho hai bus
  -> worker trái UART2 và worker phải UART1
  -> MIT absolute position/velocity (ID2,ID3), wheel speed RPM (ID1)
  -> reply hợp lệ từ cả hai bus -> completed batch -> applied_seq/applied_target
  -> TELEMETRY về LF

Encoder joint trong reply; wheel READ 0x24
  -> timestamp riêng từng motor -> đổi sign/zero, unwrap bánh
  -> q=[hipL hipR kneeL kneeR wheelL wheelR], qd tương ứng
```

Công thức đối xứng:

```text
model_angle = sign * raw_counts * (2*pi/16384) + zero
raw_target  = round(sign * (model_target-zero) * 16384/(2*pi))
wheel_rpm   = round(sign * omega_rad_s * 60/(2*pi))
c           = knee_crank_coupling (Lumi: 0; encoder crank: 1)
d_measured  = c * hip_measured + knee_measured
q_knee      = d_measured - c * hip_measured
qd_knee     = d_dot_measured - c * hip_dot_measured
d_ref       = phi_ref + (c-1) * hip_ref
d_dot_ff    = knee hinge FF + c * hip FF
applied_phi = d_quantized + (1-c) * hip_quantized
```

Khớp nhận MIT block Kp/Kd/position/RPM/tau, trong đó **tau_ff luôn bằng 0**.
Không gọi `leg_move()` (delta/trajectory) của bench. LF vẫn gửi 4 góc và 2 tốc
độ, không gửi torque. `Torque Enable 0x18` dùng bật/tắt driver.

`apply()` trả `Pending`, `Applied` hoặc `Fault`; bản ghi Applied chứa epoch,
sequence và reference của batch đã hoàn tất, có thể là batch trước lệnh mới
nhất. Chỉ một batch đang chờ được giữ cho cả hai bus. Khi chưa worker nào claim,
lệnh TARGET mới được thay vào batch chờ nếu cả hai bus sẵn sàng cho reference đó
(kể cả điều kiện wheel settle). Claim và thay batch dùng cùng khóa backend:
worker đang giữ snapshot cũ không được phát lệnh cho token đã bị thay.
Khi một bus đã claim, token/reference/deadline cố định cho cả hai bus tới khi
hoàn tất hoặc fault; TARGET mới chỉ gia hạn lease LF và chờ batch tiếp theo.
Batch bị thay không được ACK. Filter wheel luôn dựa reference hoàn tất gần nhất.
Trong startup/poll, lease mới được lấy giữa từng I/O; deadline của một I/O đã
bắt đầu không được kéo dài. I/O batch còn bị chặn bởi deadline gốc của batch. Không ACK khi enqueue hoặc khi chỉ một bus xong.
Không cần đợi trục vật lý tới đích. `applied_target` phản ánh counts/RPM đã làm
tròn và wheel reference sau lọc, không sao chép mù raw target từ LF.

Tắt cũng bất đồng bộ: `disable()` hủy generation và yêu cầu OFF; `disabled()`
chỉ true sau READ thanh ghi Torque Enable `0x18` bằng 0 của cả sáu motor.
E-stop buộc ghi/đọc lại mọi driver; disarmed kiểm lại định kỳ 100 ms. OFF là
trạng thái thanh ghi, không phải phép đo lực cơ khí hoặc bánh quay tự do. Kênh bị bỏ enable không được ghi lệnh
motion; wheel applied=0, các góc giữ reference cuối đã chốt (hoặc góc đo nếu
chưa có reference). Không diễn giải zero của DISABLED thành góc đích 0.

## HAL Lumi — 2026-11-02

Codec Lumi tạo MIT block trong production và kiểm lại frame phản hồi trên ESP.
Hai worker và module C giữ deadline/cancellation, kiểm echo/checksum và fault latch.
Khi OFF, cấu hình Response Level2 cho khớp nếu cần và đọc lại xác nhận; ghi mode
MIT3 rồi đọc lại xác nhận. Không chỉ dựa vào ACK cấu hình.

Đọc wheel Response Level0x10 khi khởi động. Nếu <2, wheel write kiểm echo đầy đủ,
chờ120us rồi READ thanh ghi đích; không chờ9ms cho ACK bị driver tắt.
`WriteEchoOnly` là kết quả nội bộ chưa chứng minh lệnh đã latch, không phải HCR ACK.
Nếu level>=2 vẫn kiểm ACK như trước; ACK mất chỉ được cứu bằng readback khớp.
Bật bánh cũng phải đọc lại Torque Enable=1, tắt phải đọc lại=0. Không bỏ qua
lỗi TX/echo/frame/status hoặc kéo dài deadline để đọc lại. Driver không hỗ trợ
READ response level/target bị phát hiện khi còn disarmed.

Build và mock đã qua; firmware này chưa nạp hoặc thử motor. Kiểm tra đầy đủ:
`tests/check_gateway.py`, `tests/check_bl_motor.py`, `tests/check_inherited_bus.py`,
12 test công cụ Python, build PlatformIO env `esp32-s3-hybrid-mcu`.

## Sửa tích hợp ngày 2026-10-31

Boot/configure khi disarmed không ghi wheel speed, chỉ OFF và đọc feedback.
Khi đã được phép điều khiển, nếu speed write thiếu reply thì đọc lại target
`0x20`; RPM phải khớp mới công nhận. Echo/TX/CRC/status lỗi, readback không
khớp hoặc timeout vẫn fault. READ present `0x24` chứng minh feedback, không
chứng minh target đã latch. TTL 20 ms và điều kiện cả hai bus hoàn tất giữ
nguyên; nhánh readback thêm độ trễ cần đo thực tế.

`MOTOR_DIAG` tách lỗi từng bus khỏi lỗi chung. `first_fail_id/inst/addr/bus_stage`
giữ giao dịch lỗi đầu tiên tới khi reset backend; OFF retries không ghi đè.
Stages (lỗi đầu): 0=none/status, 1=unavailable, 2=packet, 3=TX, 4=echo timeout,
5=echo mismatch, 6=reply timeout, 7=reply invalid, 8=readback mismatch,
9=deadline, 10=cancelled, 11=bus chưa yên sau drain. Mã nội bộ12=WriteEchoOnly
không phải fault/ACK và không được ghi vào first-failure khi đúng chế độ.
`recovered_writes` đếm mất ACK nhưng readback đúng. `enable_readback` theo
hip/driver ID3/wheel; 255=unknown. OFF mất ACK nhưng readback đúng có thể xuất hiện
trong first_fail mà không latch lỗi; đối chiếu cả local_fault và off.

Xem [báo cáo](MCU_VALIDATION.md) cho mức kiểm chứng hiện tại.

## Khởi động bánh và watchdog

BL4012e bench ghi nhận phải chờ >=20 ms sau bật lực trước ghi tốc độ mới;
backend giữ khoảng chờ 25 ms. Quy trình: ghi speed=0 khi OFF, bật bánh, đọc lại
thanh ghi speed target 0x20 để xác nhận 0. Trong thời gian chờ chỉ được ACK
reference bánh bằng 0 đã kiểm tra; **nonzero chưa được ghi/ACK**. HOME của LF
có hai tốc độ bằng 0, nên không phải tiêu gần hết ngân sách ACK 30 ms để chờ
25 ms này. Nếu đọc lại không đúng 0, fault và OFF. Khả năng READ 0x20 trên driver
thật đã xác nhận bằng READ khi OFF: cả hai bánh đọc target=0.
Chuỗi bật bánh/chờ 25 ms/ghi tốc độ vẫn chỉ có bằng chứng mock, chưa thử chuyển động.

Service chạy mỗi 500 us, không gọi I/O motor. Mỗi bus có task riêng priority18
trên core1; service priority20. Worker kiểm deadline độc lập giữa các giao dịch,
kể cả khi service/UART/console bị treo. Lệnh trùng không gia hạn lease; batch cũ
quá deadline bị hủy dù vẫn có lệnh mới tới. Lỗi một bus hủy cả hai bus.

Module mới dùng TX spin có trần1ms, echo/joint reply3ticks, wheel reply9ticks
và tối đa3ticks cho byte cuối khi cần. HCR cap thời gian chờ theo deadline
batch/lease, kiểm lại sau drain/gap và trước TX/ACK; không gia hạn khi retry.
OFF/configuration cũng có budget20ms mỗi lần gọi module. Đây là điều kiện phần
mềm, không phải hard deadline cắt lực; OFF có thể phải đợi read/TX đang chạy.
Xem báo cáo mới cho khác biệt giữa source/build/mock và timing vật lý chưa đo.

Timestamp encoder bánh chỉ cập nhật sau READ 0x24; ACK ghi tốc độ không làm
mới nó. Sáu encoder phải mới <=20 ms. Bánh unwrap modulo1024; khoảng cách mẫu
và cap950RPM giữ chuyển động dưới nửa vòng/mẫu. Mất mẫu vượt cửa sổ làm fault,
không đoán số vòng qua một khoảng đứt dài. Hip/crank dùng encoder signed
multi-turn, qd từ sai phân timestamp; không tự wrap biên int16 thành target hợp lệ.

## Độ khớp và những khác biệt được công khai

| Phần | Kết luận |
|---|---|
| 4 absolute angles + 2 wheel angular rates | Khớp; đã test cả producer/codec/consumer |
| IDs, GPIO và trái/phải | Dùng hiệu chuẩn GUI và đính chính của người dùng |
| Recovery FF | Đổi hip/knee hinge thành hip/gối (hoặc crank nếu cấu hình c=1) velocity trước RPM |
| Wheel recovery filter | tau=.075 s; reference sau lọc/làm tròn trả về LF; không thêm 50 ms delay |
| Joint gains HOME/RECOVERY/POLICY | Hip Kp10/Kd.036, gối Kp20/Kd.05 từ Lumi; cấu hình tách từng motor/profile |
| Gain max của driver | Đọc 0x32; nếu gain yêu cầu vượt max thì lỗi Configuration, không tự clamp |
| Giới hạn | Hip/knee lấy rb_dynamic.xml; counts/RPM kiểm biểu diễn; wheel cap950RPM kế thừa bench |
| Quỹ đạo/accel limit bench | Không nhập `leg_move`, jog ramp hoặc wheel ramp600RPM/s vào HCR |
| Feedback torque | Không tạo ước lượng giả; TORQUE_VALID=0 |
| Đáp ứng servo | Chưa chứng minh tương đương servo PD/PI và torque cap của MuJoCo |

Không thể kết luận gain MIT của Lumi giống mô phỏng policy Kp30/Kd1 hoặc
recovery PI có coupling hip/knee. Đây là adapter trực tiếp cho motor hiện có;
fit động học/gain/giới hạn lực phải được đo và thử sau khi nạp. Không thay
trọng số model hoặc thuật toán LF để che khác biệt đó.

Với wheel có write ACK, một batch cho đủ ba motor trên mỗi bus chiếm 97 byte trên dây, tính lệnh yêu
cầu và phản hồi: mỗi MIT joint 17+11=28 byte (2 joint: 56); wheel speed write
9+9=18; wheel encoder read 8+15=23 (lệnh đọc 6 byte dữ liệu). Tổng 56+18+23=97
byte. Nếu wheel không trả write ACK, WRITE9 + READ target8/11 thay cho
WRITE9/ACK9, nên batch107byte/bus; thời gian phản hồi vẫn cần đo. Byte echo của UART một dây là bản MCU tự đọc trong lúc phát, không phải bản
truyền thêm lần nữa trên dây. Với UART 1 Mbaud/8N1, mỗi byte chiếm 10 bit, nên
phần truyền tối thiểu là 970/1.000.000 = 0,97 ms mỗi batch, chưa kể khoảng chờ
phản hồi và lịch task. Có thể phục vụ đầu vào 200 Hz về mặt thời gian truyền thô,
nhưng tần số áp dụng thực phải đọc `cycle_max_us`, tuổi encoder và ACK khi thử
thật; build/mock không chứng minh 200 Hz.

Khi RPM không đổi và đã được xác nhận trong cùng phiên enable, gateway bỏ qua
WRITE lặp cho tới keepalive100ms; encoder vẫn được đọc mỗi batch/poll. Batch
bỏ WRITE còn79byte/bus. RPM thay đổi được gửi ngay; không nhập slot10ms của bench.
`BUS_RECOVERY` thống kê raw errors, late/sync, retries/recovered và skipped writes;
first-failure vẫn giữ khi retry thành công, không tự mang nghĩa local_fault.

Cấu hình nằm trong `include/gateway/motor/BlMotor.hpp` (`BlMotorConfig`). Sign và
limit/gain không nhận thay đổi trong lúc ARM; chỉnh cấu hình cần build lại.
Không đổi ID/baud của driver. Response level của khớp được đưa về2 và đọc lại
khi OFF như HAL Lumi; không cấu hình driver trong lúc ARM.

## Chuẩn bị vận hành sau khi nạp

1. `STATUS`: cả hai motor bus configured=1, off=1, error=0; IMU hợp lệ.
2. Đặt cơ khí vào tư thế home của model, hip L/R=2.35706715 rad, knee hinge L/R=0.
   Bấm **Disable** trên web LF, đợi OFF, xếp chân bằng tay rồi bấm
   **Reset encoder · Chụp HOME** (HCR1 `ENCODER_HOME`); hoặc gửi `MOTOR_HOME`
   qua console. Lệnh chỉ chụp zero; không reset bộ đếm driver, không di chuyển,
   không bật motor và không thay bias IMU.
   Hoặc `MOTOR_ZERO hipL hipR kneeL kneeR` khi đang ở một tư thế model đã đo khác.
   Cả sáu motor phải OFF, feedback mới và tốc độ tuyệt đối ≤.15rad/s.
3. Kiểm tra chiều/mốc feedback theo model bằng dữ liệu thật trước khi ARM.
   Zero chỉ giữ trong RAM: reboot/mất encoder/driver fault phải hiệu chuẩn lại.
4. Nút web chờ ACK chụp HOME rồi tự tạo phiên LF mới, vẫn Disable.
   Nếu dùng console và LF đã fault vì trước đó thiếu calibration, gửi `RESET` sau home, đợi
   DISARMED rồi `ARM`. RESET thông thường giữ home hợp lệ; CLEAR_FAULT cục bộ sau lỗi motor
   xóa home và yêu cầu worker OFF/configure lại. RESET LF không xóa E-stop/lỗi
   driver cục bộ; CLEAR_FAULT chỉ được chấp nhận sau verified OFF. Khi đó làm lại bước2 rồi RESET LF.
5. `DISARM` hoặc `ESTOP` yêu cầu OFF mọi motor. RESET không tự ARM.

Bằng chứng tích hợp ban đầu nằm tại `reports/motor_integration_20261030/`;
bằng chứng flash/kiểm tra kết nối và bản sửa gain nằm tại
`reports/bringup_20261030/` của worktree `luckfox_gateway_esp32`.
Chỉ đã dùng lệnh OFF/RESET và kiểm tra DISABLED, chưa HOME hoặc ARM.
ESP→LF đã nhận 24 HELLO hợp lệ, chiều LF→ESP chưa bắt tay; IMU chưa hiệu chuẩn.
Bản sửa gain đã qua mock tests và build, chưa flash: xem
`gain120_artifacts/checksums.json`. Kết quả mock không chứng minh đáp ứng motor thật.

## Lịch sử: thử sửa turnaround ngày 2026-10-31 (đã thay thế)

Giả thuyết của đợt sửa cũ là đợi TX qua scheduler rồi nhả pad làm mất đầu gói.
Đây không phải kết luận nguyên nhân cuối cùng; bản FIFO sau đó vẫn lỗi khi chuyển động.
Adapter nạp FIFO và đợi TX idle trong critical section có giới hạn, sau đó
gọi `gpio_ll_output_disable` để nhả chân. Bỏ lần đổi GPIO matrix dư trước
output-disable vì có thể nuốt start bit đầu. Không bỏ checksum, không thêm retry
để che lỗi. BUS_TRACE lưu timing/byte thô của lỗi đầu; UART_DIAG tách lỗi queue.
Các số liệu bring-up phía trên là lịch sử; trạng thái mới xem report 2026-10-31.

## Lịch sử chẩn đoán lỗi khi bắt đầu chuyển động (2026-10-31)

ARM thực đã qua HOME3s và STANDUP~0.14s, sau đó UART1/phải, wheelID1,
WRITE0x20 mất echo (0 byte), timeout~8ms. LF FAULT3 là phản hồi lỗi MCU;
không phải lỗi token hoặc lỗi tính bảng LQR đã sửa. Firmware bổ sung BUS_HW:
packetTX/echo, mức pad trước/phát xong/kết thúc, RX FIFO lúc nhả pad, ring bytes,
raw/enable interrupt, status và bitmask uart_event_type_t. Trace chỉ ghi nhận,
không bỏ kiểm echo/checksum, không retry để che lỗi.

`MOTOR_BUS_TEST`: chỉ nhận khi cả hai bus configured và verifiedOFF, không
maintenance/lỗi/batch hoạt động. Hủy HOME; mỗi bus chạy2000chu kỳ ghi wheel
speed0, READ0x20 kiểm0 và đọc encoder. Không gửi MIT hoặc enable. Trong test
không được HOME/ARM; sau test phải chụp HOME đúng tư thế. OTA/maintenance dừng
traffic. Đã chạy thật4000chu kỳ tổng, không lỗi; chưa tái hiện lỗi có chuyển động.
Bằng chứng: `reports/arm_deep_20261031/off_test.txt`.

Bản kế thừa hiện tại thay `BUS_HW`/`BUS_PAD` bằng `BUS_BYTES` và `BUS_TRACE`.
`BUS_DRIVER` ghi nguồn module và phiên bản IDF. `MOTOR_WAVE` báo unavailable;
không đưa các số 0 ra như thể đã đo mức điện tại chân.


## Chẩn đoán ARM ngày 2026-11-02

Lần ARM được người vận hành thực hiện sau calib đã vào HOME rồi FAULT sau
khoảng 0.868 s, chưa tới STANDUP. MCU báo backend `MotorError::Expired=5`,
local fault Driver; LF phản ánh thành FAULT3. Link UART vẫn connected; không
có queue overflow/stale_tx trong snapshot. Đây là bằng chứng lệnh motor bị
hủy vì deadline, chưa đủ để kết luận giao dịch cụ thể gây trễ.

Host SDK regression tìm được lỗi độc lập có thể gây Expired sớm: làm tròn
thời gian còn lại xuống tick và từ chối ngay khi còn <1 ms dù reply đã nằm
trong buffer. Bản sửa dùng đọc nonblocking trong phần tick cuối, vẫn kiểm
hạn tuyệt đối và cancellation. Test kiểm reply hợp lệ còn 800 us được nhận,
reply thiếu vẫn bị hủy đúng deadline. Chưa xác minh bản sửa bằng ARM vật lý.

`MOTOR_FAULT` mới đóng băng context của lần latch lỗi tại worker: thời điểm,
lease/batch deadline, sequence, ID/instruction/address, bus stage và timing
của giao dịch. `during_io=1` nghĩa là lỗi xảy ra trong request bus;
`during_io=0` thì ID/trace chỉ là request gần nhất, không chứng minh nó gây lỗi.
Các lần OFF sau đó không ghi đè context này. `MOTOR_DIAG first_fail_*` vẫn
là lỗi giao dịch đầu tiên trong phiên, có thể là mất write ACK lúc boot đã
được readback xác nhận; không được dùng riêng nó làm nguyên nhân ARM fault.

## Đo thời gian và xử lý batch chờ — 2026-11-02

`EXECUTION_DIAG` gửi số đo 10 Hz trên HCR1 tới LF, độc lập với TELEMETRY và
applied ACK. Web hiển thị tuổi TARGET, tuổi batch, chu kỳ L/R, I/O max L/R,
số batch chờ bị thay và RX queue max. Xem [contract](HCR1_PROTOCOL.md).
Các số đo không làm thay đổi TTL/source age, fault latch, điều kiện HOME/OFF
hoặc yêu cầu cả hai bus thực thi. Console `MOTOR_FAULT` giữ giao dịch gây latch;
`first_fail` có thể chỉ là giao dịch mất ACK đã được cứu bằng readback.

Regressions host bao phủ thay batch chưa claim, snapshot cũ, batch đã claim trên
một bus, deadline gốc dù có TARGET mới, lease mới giữa I/O, deadline cố định trong
I/O, duplicate không renew và disable hủy claim. Đây là bằng chứng mock;
chưa xác nhận bản thay đổi scheduling/chẩn đoán này trên UART/motor thật.

## Nhường bus từ feedback sang TARGET — 2026-11-02

Lần chạy vật lý sau OTA `fcb027d0830102da97afd3d4817211483d72e380a2ac039e7386e004767b3c51`
đã qua HOME và dừng ở Kneeling sau khoảng 4.4285 s. Terminal trace:
`expired_lease=0`, `expired_batch=1`, bus trái wheel ID1 READ0x24;
batch deadline326564775 us, fail326565248 us, lease326575482 us.
Giao dịch bắt đầu khi batch còn643 us, quá hạn473 us trong khi lease LF còn10234 us.
MCU CRC/queue/stale-TX/overflow đều0. LF parser tăng từ1 lên3 trong HOME trước
STANDUP, không tăng tại lần chuyển FAULT. Không coi toàn bộ transport hoàn hảo;
trace xác định nguyên nhân latch lần này là deadline batch, không phải hết hạn LF.
Hai bus đã xác nhận OFF; IMU vẫn valid. Lỗi motor làm mất HOME theo thiết kế.

Review phát hiện active worker có thể bắt đầu một sweep ba giao dịch ngay sau
batch khi feedback vẫn mới, hoặc đọc nốt sweep khi TARGET mới đã tới. Test host
đã tái hiện việc không nhường giữa I/O (FAIL trước sửa). Chưa có đủ trace để quy
mọi phần thời gian chờ trong lần chạy thật cho riêng sweep này.

Bản sửa bỏ sweep dư khi ba timestamp còn mới trong min(3 ms, feedback_age/2),
và kiểm batch chờ giữa các giao dịch feedback. Nếu có batch mới và worker đã
prepared thì kết thúc sweep, nhường cho batch ở step tiếp theo. Giao dịch đang
chạy không bị cắt giữa frame; cancellation, lease, deadline gốc và kiểm freshness
vẫn được thực hiện. Khi feedback tới kỳ vẫn đọc dù không có TARGET mới.
Đọc wheel trong mỗi batch, readback target và ACK cả hai bus giữ nguyên.
Đây là tối ưu công việc nền, không tăng TTL hoặc bỏ điều kiện ACK.

Bản sửa này cần kiểm lại trên robot; mock PASS không chứng minh đã khắc phục
mọi nguyên nhân chậm bus trên phần cứng. Không tự RESET/CLEAR_FAULT, HOME hoặc ARM
trong quá trình thu log và sửa source.

## Công bố feedback giữa các giao dịch — 2026-11-02

Lần stand với ELF `912a3a09c6f67a50d68a92402442aa53e765a6e03c5de8f1329210e963accb92`
dừng sau khoảng5.259 s từ ARM, ở Kneeling. Lần này MCU latch `Sensors=2`,
`motor_error=0`, không có worker fault/deadline; max applied latency11496 us.
Telemetry đầu chuyển FAULT có IMU_OK nhưng không ENCODER_OK. Hai bus vẫn
configured/homed, sau dừng đọc lại cả sáu Torque Enable=0. Web và HCR vẫn
connected; không có lỗi CRC/queue phía MCU hoặc parser LF tăng trong lần chạy.
Evidence: `.local/incident-runs/20261102-193226/`. Không đủ context đóng băng
để xác định chính xác tuổi từng cảm biến tại tick gây fault của firmware này.

Worker trước đây chỉ publish snapshot khi kết thúc toàn bộ step. Service2 kHz
có thể chạy trong lúc worker chờ reply tiếp theo và thấy timestamp cũ, dù
reply trước đó đã cập nhật sample trong worker. Regression host tái hiện:
hip shared stamp19 ms cũ; sau hai I/O700 us, hip thực tế mới700 us nhưng
snapshot shared đã20.4 ms. Test FAIL trước sửa, PASS sau sửa.

`publishSample` nay cập nhật timestamp/q/qd của riêng motor ngay sau response
đã kiểm status, permit và tính hợp lệ encoder. Nó không publish completed,
prepared, OFF hoặc reference batch. Reset/generation cũ bị từ chối; response
hỏng/hủy không làm mới timestamp. Một sample mới không che sample khác quá hạn.
Applied ACK vẫn cần toàn bộ batch của cả hai bus; TTL20 ms, source age30 ms,
encoder freshness20 ms và IMU freshness10 ms giữ nguyên.

Console `SENSOR_FAULT` giữ flags, tuổi IMU/encoder và readiness tại tick dừng,
trước khi OFF hoặc feedback mới che mất dấu lỗi. Timestamp encoder vẫn là
sample thật cũ nhất ngay cả khi invalid, chỉ phục vụ chẩn đoán; không cấp
ENCODER_OK. Context giữ qua feedback phục hồi/reconnect, được thay khi có
sensor fault mới hoặc xóa bởi CLEAR_FAULT thành công/reboot. Không đổi HCR1.

Host regressions bao phủ interleaving, ACK hai bus, sample khác thật sự stale,
reply CRC lỗi/hủy, reset generation cũ và context IMU/encoder fault. Đây là
bản sửa đã kiểm trên host/build MCU; chưa OTA và chưa xác nhận bằng stand thật.
