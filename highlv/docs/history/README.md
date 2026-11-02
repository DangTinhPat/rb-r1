# Báo cáo lịch sử trước khi clean

Các file trong thư mục này ghi nhận các đợt làm việc ngày 2026-10-31 với layout
và deployment cũ. Đường dẫn, trạng thái LF và các khẳng định PASS trong đó là
thông tin lịch sử, không xác nhận trạng thái source/deployment hiện tại.

| Báo cáo | Nội dung |
|---|---|
| `uart_fix_20261031.md` | UART LF–ESP32 khoảng 200 Hz, sensor/home và disabled target |
| `arm_rejected_20261031.md` | Deadline do khởi tạo bảng LQR và cách chuẩn bị trước startup |
| `arm_deep_20261031.md` | ARM được nhận, sau HOME gặp lỗi echo/reply bus motor trong Folding |

Các raw log/report được nhắc tới không được đưa lại vào cây source sạch;
bản trước khi clean còn trong snapshot được ghi tại `../source_provenance.md`.
Đánh giá source sau clean theo `../validation.md`.
