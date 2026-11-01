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
