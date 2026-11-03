# Nguồn gốc và phạm vi bản sao

RB-R1 được quản lý trong repository riêng `DangTinhPat/reconect` theo yêu cầu
người dùng. Không có submodule, symlink hoặc hardlink tới repository nguồn;
remote `origin` trỏ tới `https://github.com/DangTinhPat/reconect.git`.
Mọi source cần cho runtime nằm trong project.

## Core recovery

Thuật toán recovery/support, hình học và LQR được phát triển từ bản chụp source
Lumi-Quantum tại commit `d22e0e06c8267d96bf8be8010c288ff5de7b828a`.
Đợt clean 2026-11-01 thu gọn phụ thuộc, đổi tên class/namespace/file theo vai trò,
tách cấu hình LQR và bỏ firmware/bus/simulator. Đây là refactor của thuật toán
hiện có, không phải một controller mới được viết lại từ đầu.

Tên nguồn được giữ trong tài liệu provenance để ghi nhận lịch sử; build và
runtime không truy cập repository gốc. Không gán giấy phép mới cho source gốc.

### Đồng bộ quỹ đạo stand, 2026-11-02

Đối chiếu checkout Lumi-Quantum trên laptop (HEAD
`7b86053c89620484a3c99e869cce3c23dd39d37f`), dùng hash file để xác định chính
xác nội dung working tree:

- `firmware/control/include/rb1/control/legs/StandUp.hpp`:
  `d7bcead695789c8ec168483ae793b4ff2773781b1dea14985e3131927eacc656`.
- `firmware/control/src/RobotController.cpp`:
  `5ec2d7cd8286e603fe3f61212de0e08cc9178e2b3f1fcabd29a31396bcdd7904`.

Đồng bộ `tunedRecoveryRb()` và hai thay đổi planner: hãm nâng gối bằng
`catch_rise_brake_rad_s2`, bù hông theo tốc độ gối đo và lọc. Đầu vào recovery
dùng gối chậm hơn `min(kneeL,kneeR)` để giới hạn tiến quỹ đạo; tốc độ đo vẫn là
trung bình hai gối như nguồn. Adapter dùng lead mặc định 0,12 s và bỏ FIFO đầu
ra bánh 10 tick/50 ms. Delay 0,0976 s và lag 0,038 s của bộ dự báo vẫn là tham
số mô hình, không phải thời gian chờ trước khi phát lệnh.

Phạm vi đồng bộ là quỹ đạo và tuning, không thay toàn bộ RobotController mới:
RB-R1 vẫn giữ supported balance sau Support tới lúc policy nhận quyền, mapping
hip/crank tuyệt đối và feed-forward hip/knee của HCR1, lịch policy và warm-up.
Không chép thay đổi nguồn cho phép bỏ qua xung đột floor: RB-R1 tiếp tục latch
`FloorConstraint` theo guard hiện có. Vì vậy không khẳng định hai controller
giống nhau trong tình huống floor không khả thi hoặc sau bàn giao Support.

`tests/data/standup_lumi_reference.json` được tạo bằng cách biên dịch
`tests/fixtures/standup_trajectory_probe.cpp` với header nguồn ở trên, macro
`STANDUP_HEADER="rb1/control/legs/StandUp.hpp"` và
`STANDUP_NAMESPACE=rb1::control`. Trace dùng đầu vào cảm biến xác định ở 5 ms,
bao gồm gập/quỳ/catch, nâng/hãm và gối đứng yên; không phát sinh xung đột floor.
Tests thường chỉ dùng fixture đã đóng băng và source nội bộ, không cần repo
Lumi-Quantum. Test adapter dùng native controller thật, kiểm tra giới hạn gối
chậm, đầu ra bánh không trễ thêm, mapping crank và fault floor còn hoạt động.
Đây là bằng chứng host với cảm biến giả lập, không phải mô phỏng động lực học
hoặc xác nhận quỹ đạo trên robot thật.

## Model

ONNX actor FP32 Beni512 checkpoint 19500 của LU/RB v1.3, flat task:

```text
SHA256 model_19500.onnx
bfb8097f9240193e9075ee8afe283baeebdeb7ddeb84f98e87cd0a44b3e7c993
```

RKNN DFP16 và normalizer được giữ nguyên byte từ bộ runtime được copy vào project.
Hash từng artifact nằm trong `models/manifest.json`; `Policy` kiểm tra trước load.
Không kèm checkpoint training, critic hoặc optimizer.

## Bản trước khi clean

Snapshot đầy đủ được lưu ngoài project tại
`/home/dvt/rb-r1-backups/before-runtime-cleanup-20261101-134857.tar.gz`.
Nó chứa source cũ, mô phỏng/CAD, binary và báo cáo cũ để khôi phục/đối chiếu;
project sau clean không cần truy cập snapshot khi chạy.
