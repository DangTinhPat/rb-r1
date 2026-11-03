# Kiến trúc runtime tầng trên

## Luồng chính

`app.py` sở hữu transport và gọi `HybridCore.receive()` cho mỗi frame hợp lệ.
Core kiểm epoch, sequence, timestamp, flags cảm biến và ACK trước khi chọn đầu ra.
Mỗi telemetry có thể tạo một TARGET; đây là vòng điều khiển 200 Hz theo sample MCU.

Nhánh recovery gọi `StandupProcess` qua pipe nhị phân tới `standup_helper`.
Nhánh policy lập observation trên CPU rồi gọi ONNX trên host hoặc RKNN trên LF.
State machine chọn một chủ sở hữu target tại mỗi thời điểm. Policy chạy theo
mốc sample tuyệt đối 10 ms, không bị giảm xuống 50 Hz khi có jitter.

`app.py` có watchdog ngoài luồng nhận telemetry. Nếu sample ngừng tới trong khi
đang điều khiển, runtime vẫn phát disabled output và latch fault.

## Vì sao còn core recovery C++?

Helper cũ trực tiếp gọi controller tổng hợp để chạy `StandUp` và giữ Support.
Không thể xóa class đó rồi chỉ giữ model mà vẫn giữ hành vi hybrid hiện tại:
model chưa sở hữu output trước handoff, và Support có thể phải chờ sau giây 13.

Bản clean tổ chức core này thành `rb::recovery::RecoveryController` trong
`native/include/rb/recovery/` và `native/src/recovery_controller.cpp`.
Các cấu trúc cấu hình và nhánh toán học hiện có được giữ để tránh thay thuật toán
trong lúc quy hoạch source. Helper chỉ sử dụng giao diện cần cho recovery/support.

Build hiện chỉ gồm:

1. `recovery_controller.cpp`: sequencing recovery và supported balance.
2. `leg_height.cpp`: hình học, pose và quan hệ độ cao chân.
3. `lqr_balancer.cpp`: LQR và bảng gain.
4. `imu_state_estimator.cpp`: bộ xử lý state được recovery core sử dụng.
5. `whole_body_model.cpp`: hình học CoM/Jacobian/inertia.
6. `lqr_config.cpp`: phần cấu hình LQR đã tách khỏi embedded controller cũ.

Các bus transport, motor feedback/lock, board GPIO/CAN, generic embedded controller,
jump controller, flight observer và servo simulator đã được loại khỏi project.
Header còn lại là bao đóng include thực tế của 6 file trên.

Bảng LQR được chuẩn bị trước khi helper trả lời startup. Reset tạo instance từ
controller nguyên trạng đã có bảng gain; không chạy sensor giả để warm-up và
không làm thay đổi lịch sử estimator. Timeout startup là 2 s, exchange điều khiển 20 ms.

## Giao tiếp process recovery

Request little endian: 24 double (`192 byte`):

```text
elapsed, pitch, gyroY, gyroZ, accelX, accelZ,
q[6], qd[6], hipTorque[2], sensorsOK, ready, torqueValid, reset
```

Reply: 14 double (`112 byte`):

```text
Support, phase, enables, target[6], feedforward[4], reserved
```

Support âm báo lỗi helper. Phase: Idle, Folding, Kneeling, Catching, Support, Failed.
Target là reference góc/tốc độ; helper không chạy driver hoặc gửi torque.
Recovery helper hiện không có FIFO delay bánh50ms sau khi đồng bộ Lumi;
MCU không thêm delay này.

## Bàn giao model

Trong warm-up, actor nhận `last_action` được encode ngược từ
`TELEMETRY.applied_target`. Các prediction warm-up không được gửi xuống MCU.
Khi đủ 50 bước, đã Support và đến mốc handoff, prediction đúng deadline mới
nhận quyền. Hidden GRU không reset. Prediction muộn bị bỏ trong khi hidden vẫn
ghi nhận sample vừa xử lý; runtime giữ action/target trước.

## Các kênh giao tiếp khác nhau

| Kênh | Mục đích |
|---|---|
| UART `/dev/ttyS2` | HCR1 LF–MCU, 921600/8N1 |
| Unix MCU socket / loopback TCP tùy chọn | Thay transport để test; không tự kết nối robot |
| Unix command socket | ARM, DISARM, RESET, ESTOP, VELOCITY từ operator local |
| HTTP loopback | Dashboard, Bearer token, chuyển lệnh qua command socket |
| Pipe stdin/stdout | Python ↔ native recovery helper |

Lệnh operator không nằm trong frame HCR1. Command parser giới hạn 128 byte,
gom đủ dòng, từ chối tham số dư/nonfinite; kết nối chưa hoàn tất hết hạn sau 1 s.
Telemetry và watchdog vẫn được phục vụ trong lúc chờ dòng command.

## Web và vòng điều khiển độc lập

Dashboard HTTP chạy trong process Python riêng (`DashboardProcess`), scheduler
SCHED_OTHER/priority0. Vòng chính LF giữ UART HCR1, policy/recovery và watchdog;
web chỉ đọc snapshot JSON và gửi một lệnh qua Unix command socket cho mỗi POST.
Process web không nhận file descriptor UART hoặc đối tượng controller. Spawn
qua executable Python mới, không fork trạng thái ONNX/RKNN đang hoạt động.
Thông tin PID web có trong log ready; `web_process_alive` trong snapshot chỉ
cho biết process còn sống, không chứng minh HTTP phản hồi đúng hạn.

Web RB không dùng browser heartbeat để gia hạn TARGET. UI mất mạng, reload,
chuyển tab hoặc GET trạng thái chậm không phát DISARM/RESET/ARM và không thay đổi
trạng thái controller. Watchdog HCR1 giữa LF/ESP (TTL20ms, tuổi source30ms,
ACK30ms), lỗi driver và E-stop vẫn giữ nguyên. Lỗi bind/khởi động web được ghi
`web_unavailable`; runtime tiếp tục qua kênh local. Web crash không được tự
restart từ vòng telemetry; lúc đó cần khởi động lại dịch vụ web/runtime khi
phù hợp. Không coi browser reconnect là reset robot.

Trình duyệt chỉ giữ một GET trạng thái đang chạy. Timeout3s gồm cả đọc body,
retry sau1–5s; bỏ phản hồi thuộc phiên token/lệnh cũ. Dữ liệu cũ được đánh dấu rõ,
khóa ARM/RESET; nút DISARM/E-STOP vẫn cho phép gửi yêu cầu có token. Reload/đọc
lại thành công chỉ cập nhật giao diện. Lệnh nút bấm không tự retry: mất response
sau POST là kết quả chưa xác nhận, vì runtime có thể đã thực thi lệnh.

HTTP giới hạn8 handler đồng thời và timeout2s cho client chậm; reload/ngắt kết
nối được xử lý trong process web. Token Bearer và kiểm tra lệnh tại runtime
vẫn bắt buộc. Auth/session web không thay thế điều kiện ARM trong LF/ESP.

Kiểm tra host: test GUI bằng Node chạy script thực với DOM/network/timer giả;
HTTP test dùng localhost và Unix socket tạm; process test đóng băng web, kiểm
scheduler và không tự phát lệnh. `scripts/smoke_test.py --with-web` chạy ONNX +
core LF thật với MCU giả, đóng băng process web hơn20ms trong khi HOME/TARGET/ACK
vẫn tiếp tục, rồi đọc HTTP lại. Không mở UART thật hoặc vận hành motor.

## Định vị file và triển khai

Asset mặc định được tìm từ root project chứa `src/`, `models/`, `config/`.
`RB_RUNTIME_ROOT` có thể đổi asset root. `scripts/run.sh` thiết lập `PYTHONPATH`
và chạy từ root, nên không phụ thuộc thư mục terminal hiện tại.

Native host nằm `build/host/`; RV1106 ARMv7 dùng `build/arm/`.
Cross toolchain là công cụ build ngoài project, được chọn tường minh bằng
`RB_CROSS_ROOT`; nó không phải liên kết source tới một repository khác.
