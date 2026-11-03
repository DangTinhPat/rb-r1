# Kiến trúc runtime tầng trên

## Luồng chính

`app.py` sở hữu transport và gọi `HybridCore.receive()` cho mỗi frame hợp lệ.
Core kiểm epoch, sequence, timestamp, flags cảm biến và ACK trước khi chọn đầu ra.
Mỗi telemetry có thể tạo một TARGET; đây là vòng điều khiển 200 Hz theo sample MCU.

Nhánh gập (`fold.py`, bọc bộ gập C++ `libfold_planner.so`) tạo reference từ HOME tới tư thế gập.
Nhánh policy lập observation trên CPU rồi gọi RKNN trên NPU của LF.
State machine chọn một chủ sở hữu target tại mỗi thời điểm. Policy chạy theo
mốc sample tuyệt đối 10 ms, không bị giảm xuống 50 Hz khi có jitter.

`app.py` có watchdog ngoài luồng nhận telemetry. Nếu sample ngừng tới trong khi
đang điều khiển, runtime vẫn phát disabled output và latch fault.

## Kế hoạch gập

LF chỉ còn một bước đứng dậy: gập hông từ HOME xuống sàn rồi giữ. Việc đứng lên từ tư
thế gập thuộc về model. Phần Kneeling, Catching, Support và cân bằng LQR đã bị xóa cùng
core `rb::recovery`, `standup_helper` và pipe tới nó.

Bộ gập là hàm của thời gian MCU đã trôi qua kể từ lần gọi đầu, không cộng dồn từng bước,
nên mất frame telemetry không làm reference trễ:

```text
hip(t) = hip_đầu − min(2,4 rad/s · (t − t0), 3,05 rad)        # hip_đầu = trung bình hai hông đo
crank  = hip                                                   # gối giữ 0, crank = hip + knee
wheel  = 0, enables = legs + wheels (như HOME)
ff_hip = lọc bậc nhất τ = 25 ms của (Δhip/Δt), chặn ±6 rad/s; ff_knee = 0
```

Không có feedback nào đóng vòng: bảo vệ duy nhất là tư thế thân. `|pitch| > 0,80 rad` hoặc
đầu vào không hữu hạn là `logic_fault`. Khi hông chạm đủ hành trình, pha là `Folded`,
`folded = true` và reference giữ nguyên tới lúc model nhận quyền. Chuỗi pha: `Idle`,
`Folding`, `Folded`.

Mã nằm ở `native/fold_planner.cpp` (C++ freestanding: không libc/libm/STL, nên nạp được vào
Python của LF dù compiler dùng libc nào) và được bọc bởi `src/rb_runtime/fold.py` qua ctypes.
Kiểm pitch dùng so sánh bình phương `g0² > tan²(0,8)·(g1²+g2²)` thay cho `atan2`/`hypot`.
Test `tests/test_fold.py` so quỹ đạo với số liệu chụp từ pha Folding cũ trước khi xóa.

Hệ quả cần biết: gập xong lúc 4,275 s sau mốc ARM (3 s HOME + 1,275 s gập), robot giữ tư thế gập
thêm 1 s rồi giao cho model: policy bắt đầu chạy ngầm lúc 4,775 s (`warmup_start_s`) để đủ 50
bước warm-up (0,5 s), và nhận quyền lúc 5,275 s (`handoff_earliest_s`). Hai mốc này nằm trong
`config/runtime.json`; `warmup_start_s` phải sau khi gập xong (4,275 s) và cách
`handoff_earliest_s` ít nhất 0,5 s. Nếu đổi tốc độ hay hành trình gập thì phải đổi lại hai mốc.

## Bàn giao model

Trong warm-up, actor nhận `last_action` được encode ngược từ
`TELEMETRY.applied_target`. Các prediction warm-up không được gửi xuống MCU.
Khi đủ 50 bước, đã gập xong (`Folded`) và đến mốc handoff, prediction đúng deadline mới
nhận quyền. Hidden GRU không reset. Prediction muộn bị bỏ trong khi hidden vẫn
ghi nhận sample vừa xử lý; runtime giữ action/target trước.

## Các kênh giao tiếp khác nhau

| Kênh | Mục đích |
|---|---|
| UART `/dev/ttyS2` | HCR1 LF–MCU, 921600/8N1 |
| Unix MCU socket / loopback TCP tùy chọn | Thay transport để test; không tự kết nối robot |
| Unix command socket | ARM, DISARM, RESET, ESTOP, VELOCITY từ operator local |
| HTTP loopback | Dashboard, Bearer token, chuyển lệnh qua command socket |

Lệnh operator không nằm trong frame HCR1. Command parser giới hạn 128 byte,
gom đủ dòng, từ chối tham số dư/nonfinite; kết nối chưa hoàn tất hết hạn sau 1 s.
Telemetry và watchdog vẫn được phục vụ trong lúc chờ dòng command.

## Web và vòng điều khiển độc lập

Dashboard HTTP chạy trong process Python riêng (`DashboardProcess`), scheduler
SCHED_OTHER/priority0. Vòng chính LF giữ UART HCR1, policy, kế hoạch gập và watchdog;
web chỉ đọc snapshot JSON và gửi một lệnh qua Unix command socket cho mỗi POST.
Process web không nhận file descriptor UART hoặc đối tượng controller. Spawn
qua executable Python mới, không fork trạng thái RKNN đang hoạt động.
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
scheduler và không tự phát lệnh. `scripts/smoke_test.py --with-web` chạy
core LF thật (policy giả vì model chỉ chạy trên NPU) với MCU giả, đóng băng process web hơn20ms trong khi HOME/TARGET/ACK
vẫn tiếp tục, rồi đọc HTTP lại. Không mở UART thật hoặc vận hành motor.

## Định vị file và triển khai

Asset mặc định được tìm từ root project chứa `src/`, `models/`, `config/`.
`RB_RUNTIME_ROOT` có thể đổi asset root. `scripts/run.sh` thiết lập `PYTHONPATH`
và chạy từ root, nên không phụ thuộc thư mục terminal hiện tại.

Native host nằm `build/host/`; RV1106 ARMv7 dùng `build/arm/`.
Cross toolchain là công cụ build ngoài project, được chọn tường minh bằng
`RB_CROSS_ROOT`; nó không phải liên kết source tới một repository khác.
