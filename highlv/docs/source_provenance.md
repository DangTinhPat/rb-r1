# Nguồn gốc và phạm vi bản sao

> **Bản ghi lịch sử.** Phần "Core recovery" dưới đây mô tả mã Lumi-Quantum đã từng nằm trong
> `native/` (StandUp, RecoveryController, LQR). Mã đó đã bị xóa khỏi project; model đảm nhận
> việc đứng lên. Chỉ kế hoạch gập (`fold.py`, `FoldLogic`) còn lại. Quỹ đạo Folding cũ được
> giữ làm số liệu tham chiếu trong `tests/data/fold_reference.json`. Lịch sử đầy đủ nằm trong git.

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

Policy hiện dùng là `fitted22500_dfp16_jumprec.rknn` cùng `fitted22500_normalizer.json`, sao
chép nguyên byte từ `/userdata/lu_v13_gru256_hil/` trên LF vào `models/`. Chưa có tài liệu về
nguồn huấn luyện, checkpoint hay observation/action contract của nó. Hash nằm trong
`models/manifest.json`; `Policy` kiểm tra trước khi load.

```text
SHA256 fitted22500_dfp16_jumprec.rknn
a8176c9bde0053757a1cae9e30ee38901f6def7b787670319f576fc9697e0f29
SHA256 fitted22500_normalizer.json
6162a8ba67cf2fb765a35777cfc3b1b0df246491738cfd4e69f55382011462ec
```

Model trước đó (Beni512 checkpoint 19500, ONNX FP32 `bfb8097f…`, RKNN `4e5b1dd4…`) đã bị gỡ khỏi
project; lịch sử có trong git. Không kèm checkpoint training, critic hoặc optimizer.

## Bản trước khi clean

Snapshot đầy đủ được lưu ngoài project tại
`/home/dvt/rb-r1-backups/before-runtime-cleanup-20261101-134857.tar.gz`.
Nó chứa source cũ, mô phỏng/CAD, binary và báo cáo cũ để khôi phục/đối chiếu;
project sau clean không cần truy cập snapshot khi chạy.
