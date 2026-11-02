# Review và clean source — 2026-11-01

## Phạm vi

Tổ chức lại bản copy thành runtime tầng trên: telemetry, hybrid recovery,
inference model, TARGET, command và dashboard. Người dùng chọn bỏ mô phỏng/CAD.
Không triển khai lên LF, không thay firmware MCU hoặc vận hành motor.

## Những vấn đề đã xử lý

| Vấn đề trong bản copy | Thay đổi |
|---|---|
| Native helper gọi `prepareLqrGainTableStep()` nhưng header copy không có hàm này; build thất bại | Gọi trực tiếp gain-table builder có sẵn với số điểm cấu hình trước startup; reset sao chép controller đã chuẩn bị |
| 11/16 unit test lỗi vì dùng API CONTROL/ARM cũ | Test dùng `HybridCore.command()`; parser HCR1 vẫn từ chối legacy message type 4 |
| Codec C vẫn có CONTROL encoder trong khi Python runtime đã bỏ | Bỏ enum/encoder cũ; giữ HELLO/TELEMETRY/TARGET/STATUS và layout HCR1 hiện tại |
| Tree trộn simulator, CAD, model trùng, runtime, firmware core và nested deployment | Đưa runtime lên root chuẩn; bỏ bundle simulator, duplicate ONNX, binary cũ, cache và file `.before_*` |
| CMake build cả các module firmware không cần cho helper | Giữ 6 implementation file và 15 header của recovery dependency closure; bỏ bus/board/motor/jump/simulator |
| Cấu hình LQR kéo theo EmbeddedController và hàng loạt header khác | Tách riêng `LqrBalancer::Config` và các giá trị LQR đã dùng, không kéo controller embedded vào runtime |
| Command socket đọc một `recv()` rồi parse, không đảm bảo đủ dòng; bỏ qua tham số dư ở ARM | Buffer dòng có giới hạn 128 byte/1 s, kiểm đúng số tham số và finite; không thực thi command chưa đủ dòng |
| Nhận client command mới có thể thay mất reference client trước | Đóng client đến thêm trong khi đang xử lý một client, giữ luồng hiện tại có hạn thời gian |
| Tool NPU có thể cạnh tranh runtime đang chạy | Chẩn đoán NPU/timing lấy cùng status lock; thêm tùy chọn status file cho cấu hình khác |
| Đường dẫn build phụ thuộc layout cũ và thư mục cá nhân | Root tương đối, package `rb_runtime`, `RB_CROSS_ROOT` được chỉ định tường minh, VS Code dùng workspace root |
| Comment mang lịch sử firmware/bench, tên class/file cũ gây hiểu nhầm | Tách namespace `rb::recovery`/`rb::control`, tên file snake_case, format Python/C/C++, bỏ comment lịch sử không còn áp dụng |
| Log PASS cũ và manifest release đã lệch source | Bỏ manifest release cũ; chỉ giữ hash artifact model hợp lệ và tách báo cáo lịch sử khỏi kết quả kiểm mới |

## Thành phần đã loại

- MuJoCo plant, CAD/mesh, viewer, simulation runner và servo simulation bridge.
- Cây `contoller hybrid/` và alias `hybrid_runtime`.
- Thư mục vendor layout cũ, bus transport/feedback/leg lock, board GPIO/CAN.
- Generic embedded controller, jump/flight/trajectory/stance module không được
  native helper cần, cùng source/header chỉ phục vụ chúng.
- Binary đóng gói cũ, cache Python và các bản `.before_*`.

Snapshot trước khi clean nằm ngoài project, xem `source_provenance.md`.

## Những gì được giữ nguyên về điều khiển

- Thuật toán đứng dậy và giữ Support, gain và hình học mà helper hiện sử dụng.
- Observation/action mapping, thứ tự kênh, unit và normalizer.
- File ONNX, RKNN, JSON normalizer và recurrent reference.
- HOME 3 s, warm-up 12,5 s, handoff sớm nhất 13 s, 50 bước và hidden GRU.
- Fault latch, deadline và điều kiện handoff; không hạ ngưỡng để làm test qua.
- HCR1 TARGET = 4 góc hip/crank + 2 tốc độ bánh, không có torque command.

RecoveryController vẫn giữ các cấu hình và nhánh toán học được core gọi từ
`update()`/default config. Đợt này thu gọn phụ thuộc và ranh giới module, không
viết lại core recovery thành thuật toán khác hoặc thay bằng phát pose theo thời gian.

## Giới hạn còn lại

Build/test host không chứng minh đáp ứng servo thật tương đương plant dùng khi
train/kiểm model. Lỗi echo/reply bus motor trong báo cáo lịch sử thuộc firmware/
hardware phía dưới và chưa được sửa hoặc kiểm lại trong đợt này.

Lệnh VELOCITY local được giữ tới khi thay lệnh, DISARM, RESET hoặc lỗi;
runtime chưa có lease riêng cho lệnh vận tốc local. Client cần chủ động gửi stop.
Deadline của Linux/NPU còn phụ thuộc tải máy; cần kiểm lại trên LF sau khi deploy.
