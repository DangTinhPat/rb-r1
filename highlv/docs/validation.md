# Kiểm chứng bản clean — 2026-11-01

## Kết quả trên máy phát triển

| Mức kiểm chứng | Kết quả |
|---|---|
| Build native host | PASS: recovery helper và tensor helper |
| Cross-build ARMv7 hard-float | PASS: helper link static; tensor `.so` không có DT_NEEDED |
| Unit/contract/integration HTTP | 27 test PASS, không SKIP trong môi trường kiểm này |
| ONNX CPU | 100 bước recurrent action/hidden đối chiếu supplied reference, sai số <1e-4 |
| Native trước/sau thu gọn | 6.600 reply byte-identical, 3 reset, input feedback tổng hợp |
| Smoke test runtime thật | PASS: ONNX CPU, recovery process, Unix MCU/command sockets tạm |
| Service wrapper và diagnostic lock | PASS: start/status/stop trên host; tool NPU/timing từ chối lock đang được runtime giữ |
| Package Python | PASS: cài editable và gọi console entry point `rb-runtime --help` |
| Lint/format | Python không có undefined/unused imports; Python/C/C++ đã format |

Native reference được build từ source cũ và thay lời gọi preparation API bị
thiếu bằng lời gọi builder thực đã có trong core. Bản clean dùng cùng cách đó.
Không chạy cập nhật sensor giả để chuẩn bị state. So sánh 6.600 mẫu quan sát
Folding/Kneeling/Catching; không xem feedback tổng hợp là mô phỏng robot hoặc
bằng chứng đứng dậy/Support trên plant thật.

Smoke test bao gồm HELLO/ACK, DISARMED, từ chối command sai tham số, ARM bị chia
nhiều lần gửi, HOME target, CRC lỗi, MCU E-stop, fault latch và RESET/epoch mới.
Không mở UART thật, không truy cập motor hoặc gửi lệnh tới LF đang vận hành.

## Lệnh tái lập

```bash
./scripts/build.sh host
./scripts/test.sh
python3 scripts/smoke_test.py --report reports/current/smoke_test.json

RB_CROSS_ROOT=/duong/dan/toolchain/root ./scripts/build.sh arm
readelf -d build/arm/lib/libtensor_math.so
file build/arm/bin/standup_helper
```

Unit test cần dùng Python có NumPy/ONNX Runtime và build host để chạy đủ.
Test dùng fallback fake policy để kiểm lịch/handoff, có test riêng chạy ONNX thật.
Runtime smoke test dùng ONNX thật nhưng chỉ đi tới HOME trước khi phát E-stop.

Các log/báo cáo đã tạo trong lượt này nằm tại `reports/current/`.
Bản so sánh native dùng executable reference được build tạm ngoài project;
muốn tái lập reference cần snapshot trước clean theo `source_provenance.md`.

## Chưa kiểm lại trong đợt này

- Load và inference RKNN bằng binary mới trên Luckfox RV1106.
- UART vật lý LF–ESP32 và firmware MCU hiện đang chạy.
- Response/gain/filter/giới hạn của driver motor.
- Đứng dậy, Support, handoff và cân bằng trên robot thật.
- Init/service sau reboot hoặc rootfs/bind mount của LF.

Build ARM chỉ xác nhận tạo được artifact cho kiến trúc đích. Không diễn giải
kết quả host hoặc báo cáo cũ trong `history/` thành xác nhận vận hành phần cứng.

Sau khi có yêu cầu triển khai/kiểm LF riêng, dùng các tool chẩn đoán khi runtime
đã dừng và có điều kiện kiểm phù hợp:

```bash
python3 scripts/check_standup_timing.py --report reports/current/standup_timing.json
python3 scripts/verify_npu.py --report reports/current/npu_reference.json
```

Các tool lấy status lock và từ chối chạy nếu runtime đang giữ lock tương ứng.
Nếu đổi status file, truyền cùng `--status-file` cho runtime và tool.
