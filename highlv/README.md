# reconect — Runtime hybrid tầng trên của RB-R1

Project chạy trên **Luckfox RV1106** để nhận telemetry từ ESP32, chạy core đứng dậy,
thực hiện inference policy Beni512 và truyền target trở lại MCU qua HCR1.

Source được tổ chức thành project độc lập trong repository
[`DangTinhPat/reconect`](https://github.com/DangTinhPat/reconect), không có
submodule hoặc liên kết tới source ngoài thư mục này. Core recovery cần thiết
được giữ trong `native/`; nguồn gốc thuật toán được ghi tại
[docs/source_provenance.md](docs/source_provenance.md).

## 1. Phạm vi và trách nhiệm

| Thành phần | Trách nhiệm |
|---|---|
| Luckfox / runtime này | UART, kiểm tra telemetry, máy trạng thái hybrid, core recovery C++, observation, inference RKNN, GRU, chọn và gửi target, watchdog và dashboard |
| ESP32 / firmware riêng | Đọc IMU và encoder, ước lượng attitude, homing, đổi tọa độ motor, áp dụng target, xác nhận applied reference và watchdog motor |
| Driver motor | Đóng vòng và thực thi lệnh do firmware MCU gửi |

Project này không chứa firmware ESP32, bus driver motor, môi trường training,
MuJoCo, CAD hoặc viewer. Các bài test dùng CPU, pipe và socket giả lập; không mở UART thật.

```text
ESP32 ── TELEMETRY / HCR1 ──► runtime LF
                               │
                  ┌────────────┴────────────┐
                  │                         │
           core recovery C++       observation → RKNN + GRU
                  │                         │
                  └────────────┬────────────┘
                        máy trạng thái
                               │
ESP32 ◄── TARGET / HCR1 ────────┘
```

Core recovery hiện có chức năng đứng dậy và giữ Support trong lúc chờ bàn giao.
Đây là lý do runtime cần một phần core điều khiển C++; model chỉ nhận quyền sau
khi các điều kiện chuyển pha đã đạt. Tên, namespace và build target đã được quy
hoạch theo vai trò trong RB-R1, không còn cây firmware `v3` hoặc thư mục vendor cũ.

## 2. Cấu trúc thư mục

```text
rb-r1/
├── README.md
├── AGENTS.md                  # Quy ước bảo trì source và contract
├── CMakeLists.txt             # Build native helper và tensor helper
├── pyproject.toml             # Package Python rb-runtime
├── src/rb_runtime/
│   ├── app.py                 # Event loop, transport, watchdog, status
│   ├── hybrid_core.py         # Máy trạng thái và lịch chuyển recovery → policy
│   ├── recovery.py            # Giao tiếp pipe với process recovery C++
│   ├── policy.py              # Observation 32D, action mapping, GRU
│   ├── rknn_backend.py        # Binding RKNN NPU qua ctypes
│   ├── protocol.py            # Codec HCR1 phía Python
│   ├── transport.py           # UART độc quyền và hàng đợi ghi
│   ├── commands.py            # Parser lệnh vận hành qua socket local
│   ├── dashboard.py           # HTTP dashboard có access token
│   └── paths.py               # Định vị asset và native build theo project
├── native/
│   ├── standup_main.cpp       # Adapter process recovery, pipe nhị phân
│   ├── tensor_math.c          # Quantization và gọi NPU trong một lượt native
│   ├── include/rb/recovery/   # Core recovery và hình học chân
│   ├── include/rb/control/    # LQR, estimator và mô hình hình học được core dùng
│   └── src/                   # 6 file triển khai cần cho recovery
├── config/runtime.json        # Cấu hình mặc định trên LF
├── models/                    # ONNX, RKNN, normalizer và manifest SHA256
├── protocol/include/          # Codec HCR1 C để đối chiếu với MCU
├── scripts/                   # Build, chạy, service, lệnh và chẩn đoán
├── tests/                     # Unit, contract, codec, native và ONNX reference
├── web/index.html             # Giao diện vận hành
├── deploy/S96rbruntime         # Mẫu init script Buildroot
├── docs/                      # Kiến trúc, contract, review và lịch sử kiểm chứng
└── build/{host,arm}/           # Sinh khi build; không phải source
```

Kết quả kiểm tra mới được ghi tại `reports/current/` khi chạy các lệnh có
`--report` hoặc khi lưu log. `build/` và `reports/` có thể tạo lại.

## 3. Dữ liệu và trình tự điều khiển

TELEMETRY gửi góc đo theo thứ tự
`[hipL, hipR, kneeL, kneeR, wheelL, wheelR]`, đơn vị rad; qd cùng thứ tự, rad/s.
Nếu encoder đo crank thì MCU đổi `knee = crank − hip` trước khi gửi.

TARGET có 6 giá trị chính:

```text
[hipL_ref, hipR_ref, crankL_ref, crankR_ref, wheelL_speed, wheelR_speed]
         4 góc tuyệt đối sau homing (rad)         2 tốc độ (rad/s)
```

Recovery có thêm 4 vận tốc feedforward hipL/R, kneeL/R, đơn vị rad/s.
TARGET không chứa torque command. Chi tiết payload, thứ tự và ACK nằm trong
[docs/protocol.md](docs/protocol.md).

| Trạng thái / thời gian từ sample đầu sau ARM | Hoạt động |
|---|---|
| `WAIT_LINK` | Chờ HELLO/ACK và lập epoch |
| `DISARMED` | Có liên kết; gửi TARGET disabled |
| `HOME`, 0–3 s | Giữ tư thế home |
| `STANDUP`, từ 3 s | Core recovery tạo reference 200 Hz |
| `WARMUP`, từ 12,5 s | Policy chạy 100 Hz để cập nhật GRU; recovery vẫn sở hữu target |
| `MODEL`, sớm nhất 13 s | Chuyển khi Support, đủ ít nhất 50 warm-up inference và prediction đúng deadline |
| `FAULT` | Disabled output; lỗi được giữ cho tới RESET |

Trong warm-up, `last_action` lấy từ reference MCU thực sự đã áp dụng.
Bàn giao giữ nguyên GRU. Giữa hai lượt policy, runtime giữ target và vẫn phục
vụ telemetry ở 200 Hz. Không thêm tanh hoặc clamp action vào `[-1,1]`.

Các giới hạn hiện tại: telemetry/ACK 30 ms; prediction muộn hơn 10 ms bị bỏ;
trên 15 ms hoặc 3 lần muộn liên tiếp gây FAULT; MCU phải thực thi TTL 20 ms riêng.
Đây là cơ chế xử lý deadline, không phải cam kết hard real-time của Linux.

## 4. Build và kiểm tra trên máy phát triển

Cần Linux, Python >=3.9, CMake >=3.20 và compiler C/C++17.
Unit test dùng thư viện chuẩn. Kiểm tra model CPU và smoke test cần NumPy,
ONNX Runtime. Có thể cài riêng trong venv:

```bash
cd /duong/dan/rb-r1
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[cpu]'
./scripts/build.sh host
RB_PYTHON=.venv/bin/python ./scripts/test.sh
.venv/bin/python scripts/smoke_test.py --report reports/current/smoke_test.json
```

Nếu Python hệ thống đã có các thư viện CPU, có thể dùng `python3` trực tiếp.
Test native cần build host trước; test native/ONNX sẽ báo SKIP nếu thiếu điều kiện,
nên cần xem cả số test bị bỏ qua khi đánh giá kết quả.

Smoke test tự khởi chạy runtime CPU, tạo các socket tạm và đóng chúng sau khi
kiểm tra. Không cần MCU hoặc robot. Thêm `--with-web` để kiểm tra mất/khôi phục HTTP
trong khi core tiếp tục HOME qua MCU giả. Nó kiểm tra handshake, lệnh local, HOME,
CRC lỗi, E-stop và RESET; không xác nhận động lực học đứng dậy hoặc cân bằng.

Chạy runtime host thủ công với MCU emulator riêng:

```bash
./scripts/run.sh --backend onnx \
  --socket /tmp/rb-hybrid-runtime.sock \
  --realtime-priority 0 --no-web
```

## 5. Build cho Luckfox RV1106

Cross build thực hiện trên máy Linux phát triển. Chỉ định toolchain bằng biến
môi trường; project không gắn cứng đường dẫn thư mục cá nhân:

```bash
export RB_CROSS_ROOT=/duong/dan/arm-cross-toolchain/root
./scripts/build.sh arm
```

Script hiện hỗ trợ layout toolchain Debian GCC 13 với compiler
`usr/bin/arm-linux-gnueabihf-{gcc,g++}-13`. Toolchain khác cần điều chỉnh cấu hình
compiler tương ứng. Kết quả:

- `build/arm/bin/standup_helper`: executable ARMv7 hard-float link static.
- `build/arm/lib/libtensor_math.so`: thư viện native không có phụ thuộc libc.

Trên LF cần Python thư viện chuẩn, RKNN runtime tại
`/oem/usr/lib/librknnmrt.so` và model RV1106 phù hợp. Runtime RKNN không cần NumPy,
ONNX Runtime, GLFW hoặc MuJoCo. Giữ cây thư mục project cùng `build/arm/` và
`models/` khi triển khai; runtime tự chọn native build ARM trên ARMv7.

## 6. Chạy trên LF và vận hành

`config/runtime.json` mặc định dùng `rknn`, `/dev/ttyS2`, 921600 baud, 8N1 và
SCHED_FIFO priority 20. Chạy dưới quyền phù hợp để mở UART và đặt scheduler.
Runtime kiểm tra chủ sở hữu UART rồi lấy TIOCEXCL; không tự dừng ứng dụng khác.

```bash
./scripts/run.sh
# Hoặc qua service wrapper:
./scripts/service.sh start
./scripts/service.sh status
./scripts/service.sh stop
```

Web chạy trong process riêng với scheduler thường; HTTP chậm/reload không giữ
GIL của vòng điều khiển. UI tự kết nối lại phần hiển thị, không tự phát lại lệnh.
Khi dữ liệu cũ, ARM/RESET bị khóa; DISARM/E-STOP vẫn có thể gửi yêu cầu có token.
Web hiển thị riêng chẩn đoán thực thi từ ESP khi firmware hỗ trợ: tuổi TARGET,
tuổi batch và thời gian từng bus motor; dữ liệu cũ được đánh dấu.
Web RB không có browser-heartbeat để gia hạn motor: watchdog LF–ESP hoạt động
độc lập. Mất response sau một lệnh được báo chưa xác nhận, không tự gửi lại.

Service wrapper dùng PID `/var/run/rb-runtime.pid`, log `/tmp/rb-runtime.log`.
Trạng thái mặc định: `/tmp/rb-hybrid-status.json`.
Mẫu `deploy/S96rbruntime` dùng project tại `/root/rb-r1`; có thể đặt
`RB_RUNTIME_HOME` cho vị trí khác. Việc chép source không tự cài init script
hoặc sửa service đang chạy trên LF.

Các lệnh local gửi qua `/tmp/rb-hybrid-command.sock`:

```bash
python3 scripts/command.py ARM
python3 scripts/command.py VELOCITY 0.3 0.0
python3 scripts/command.py DISARM
python3 scripts/command.py RESET_ENCODERS
python3 scripts/command.py ESTOP
python3 scripts/command.py RESET
```

ARM bắt đầu HOME rồi tự chuyển sang đứng dậy; không có chế độ HOME-only riêng.
Trước ARM cần telemetry mới, IMU/encoder/homing hợp lệ và tư thế gần home.
HOME feedback: hipL/R = 2,35706715 rad, kneeL/R = 0.
RESET tạo phiên mới và không tự ARM. DISARM giữ nguyên lỗi đang latch.

Chụp HOME trên web: **Disable → đợi sáu motor OFF → xếp hai chân về HOME bằng
tay → giữ yên → Reset encoder · Chụp HOME**. ESP chụp zero tại tư thế này,
không ra lệnh di chuyển. Chờ thông báo ESP đã chụp HOME; motor vẫn Disable,
ARM là thao tác riêng. Nút này khác RESET phiên/lỗi. Firmware ESP phải hỗ trợ
`ENCODER_HOME`; firmware cũ sẽ báo chưa xác nhận và không tự thử lại.
Chụp HOME cần làm sau mỗi boot ESP, không cần làm lại chỉ vì Disable/Enable.
IMU và zero encoder là hai việc độc lập: Disable/Enable và Reset encoder giữ
bias IMU. Sau reboot/OTA, IMU tự hiệu chuẩn nếu đủ điều kiện đứng yên; nếu
không đạt thì cần `CAL_IMU` qua console khi OFF và đứng yên. Reset encoder
không thay cho IMU hợp lệ trước ARM.
Lệnh vận tốc trên socket local được giữ tới khi có lệnh mới, DISARM, RESET hoặc lỗi;
client điều khiển phải chủ động gửi lệnh dừng.

Dashboard mặc định tại `127.0.0.1:8080`; access token nằm ở
`/tmp/rb-hybrid-web.token`. API status và command cần Bearer token. Dashboard có
ARM, Disable, Reset encoder, RESET phiên/lỗi và ESTOP; lệnh vận tốc dùng CLI. Có thể truy cập qua SSH tunnel:

```bash
ssh -L 8080:127.0.0.1:8080 root@DIA_CHI_LF
```

## 7. Kiểm chứng và tài liệu

- [Kiến trúc và phụ thuộc](docs/architecture.md)
- [Giao thức HCR1](docs/protocol.md)
- [Input/output model và normalizer](docs/model_contract.md)
- [Kết quả review và thay đổi](docs/review.md)
- [Kiểm chứng hiện tại và giới hạn](docs/validation.md)
- [Nguồn gốc source/model](docs/source_provenance.md)

Đợt clean này chỉ thay đổi source trên máy phát triển. Build host/ARM, test CPU
và mock transport không thay cho kiểm lại RKNN trên LF, UART vật lý hoặc motor.
Lịch sử thử phần cứng được lưu riêng trong `docs/history/`; báo cáo gần nhất
ghi nhận lỗi echo/reply bus motor khi bắt đầu Folding, chưa bàn giao sang model.
