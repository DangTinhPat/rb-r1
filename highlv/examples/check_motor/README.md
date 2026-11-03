# GUI kiểm tra motor qua LF ↔ ESP

Bản chuyển từ `lumi-quantum/examples/check_motor`: GUI web thay Tkinter để chạy
trên Luckfox không có màn hình/Tk. Chỉ cần Python thư viện chuẩn trên LF; mở GUI
bằng trình duyệt laptop. Lệnh và telemetry motor đều đi qua UART, không đi qua
Wi-Fi của ESP.

```text
Trình duyệt → HTTP :8082 → Python trên LF → /dev/ttyS2 @ 921600
                                               ↕
                                  ESP UART0 TX43 / RX44
                                    ↙             ↘
                           UART1 DATA17       UART2 DATA15
                             chân phải          chân trái
```

Đây là **example bench độc lập**, dùng ASCII kế thừa Lumi-Quantum, không phải
HCR1 và không chạy cùng runtime production hoặc `check_hcr_link.py` trên một UART.
Không dùng policy, IMU hay logic đứng dậy. Không thay đổi contract HCR1 production.

## Chạy trên LF

ESP cần firmware `lowlv/examples/check_motor/firmware`, environment
`check-motor-ota` nếu cần giữ Wi-Fi/OTA. Firmware `check_hcr_link` hiện tại chỉ
trả lời hello, không nhận lệnh motor.

Dừng chương trình đang giữ UART, rồi trên LF:

```sh
python3 /userdata/highlv/examples/check_motor/server.py --port /dev/ttyS2
```

Server in URL `http://<IP-LF>:8082/#<token>`. Thay `<IP-LF>` bằng
`10.10.60.232`, giữ nguyên token rồi mở trên trình duyệt laptop. Token nằm trong `/tmp/rb-check-motor.token` (quyền 600), được giữ khi restart
server trong cùng lần boot để URL không đổi. Có thể dùng `--bind 127.0.0.1` và SSH tunnel nếu muốn.
Mỗi lần chỉ một trang GUI giữ quyền điều khiển.

1. Bấm **Kết nối GUI**, kiểm tra cả sáu motor online, OFF xác nhận.
2. **Online 6/6** nghĩa là sáu driver phản hồi; **HOME 0/6** là chưa chụp home.
   Bấm **RESET lỗi**, rồi **MOTOR_HOME**. Chờ HOME 6/6; xem ACK ESP ngay dưới thanh trạng thái.
3. Bật riêng motor muốn kiểm tra. Mặc định bánh 30 rpm, khớp 3 rpm/bước 5°.
4. Bánh: đặt rpm hoặc 0 rpm. Khớp: dịch từng bước hoặc bấm jog một nhịp tối đa0,3s. Mỗi click gửi đúng một lệnh.
5. **Tắt motor** tắt motor chọn; **STOP** tắt tất cả và latch lỗi. Ctrl+C ở
   terminal LF cũng gửi STOP.

HOME trong bench chỉ chụp tư thế encoder hiện tại lúc OFF, không di chuyển và
không ghi zero xuống driver; đây không phải homing tọa độ robot của production.
Không tự bật motor sau kết nối, HOME, RESET, reconnect hoặc reboot.

## Mapping kế thừa Lumi-Quantum

| m | Chân / bus | Motor | ID |
|---|---|---|---|
| 0 | Phải, GPIO17 | Bánh | 1 |
| 1 | Phải, GPIO17 | Crank | 3 |
| 2 | Phải, GPIO17 | Hip | 2 |
| 3 | Trái, GPIO15 | Bánh | 1 |
| 4 | Trái, GPIO15 | Crank | 3 |
| 5 | Trái, GPIO15 | Hip | 2 |

Bánh dương là tiến; jog/move dương là gập, dùng dấu motor của example gốc.
Góc hiển thị là độ encoder trong frame driver, không phải knee = crank − hip.
Giới hạn driver ±719° và lag guard 15° kế thừa bản gốc; chưa phải giới hạn cơ
khí đã hiệu chuẩn của robot. Gains được ESP clamp theo Kp_max/Kd_max đọc từ driver.

## Timeout và bằng chứng trên GUI

- LF gửi heartbeat UART mỗi 5 ms khi GUI giữ quyền và telemetry mới.
- ESP lease **20 ms**, kiểm tra watchdog mỗi 1 ms và trước lần gửi motor chủ động.
  Hết lease / STOP / lỗi driver hoặc giao dịch motor đang ON: latch lỗi, bỏ lệnh
  đang chờ, yêu cầu OFF cả hai bus; phải RESET và HOME lại.
- Heartbeat GUI qua WebSocket mỗi 50 ms, trong Web Worker riêng; thiếu GUI 300 ms hoặc thiếu telemetry ESP 150 ms:
  LF ngừng heartbeat, bỏ hàng đợi và gửi STOP. Chuyển tab/đóng trang/mất focus
  cũng yêu cầu STOP. Các timeout GUI này độc lập với lease UART 20 ms.
- `#A ... QUEUED`: ESP đã nhận và xếp lệnh; **không phải ACK applied** của HCR1.
  Xem encoder/tốc độ để kiểm tra motor phản hồi. `ON yêu cầu` là trạng thái worker,
  còn `OFF xác nhận` cần đọc lại thanh ghi torque-enable = 0.
- Driver bus có giao dịch blocking. Lease không cho bắt đầu/gửi lại lệnh chủ động
  đã quá hạn; chưa có phép đo chứng minh motor vật lý OFF trong đúng 20 ms khi
  một giao dịch đang chờ. Không dùng example này thay runtime cân bằng.

## Kiểm tra không phần cứng

Từ repository root:

```sh
python3 -m unittest discover -s highlv/examples/check_motor/tests -v
node highlv/examples/check_motor/tests/test_gui_session.js
```

Tests mở pseudo-UART và HTTP localhost tạm thời, không chạm UART thật.

Server ghi chẩn đoán cục bộ tại `/tmp/rb-check-motor-status.json` trên LF
(quyền 600): Online/HOME, ACK, số lệnh gửi/ACK nhận, lý do STOP và heartbeat gap.
File không chứa access token. `--status-file` và `--token-file` đổi vị trí nếu cần.
Trạng thái GUI được đọc riêng mỗi 200 ms. Heartbeat không chờ HTTP hoặc luồng
giao diện; phản hồi cũ đến sau Connect/STOP không được ghi đè phiên mới.
Lỗi GET trạng thái chỉ đánh dấu dữ liệu cũ, khóa các nút lệnh và tự thử đọc lại;
không đóng heartbeat đang tốt hoặc gửi STOP chỉ vì lỗi hiển thị. Nút STOP luôn
có thể gửi yêu cầu kể cả khi dữ liệu hiển thị cũ; khả năng nhận được yêu cầu
vẫn phụ thuộc mạng. Khi đọc lại được, GUI cập nhật trạng thái, không phát lại lệnh.
Hai nhãn Hiển thị/Điều khiển phân biệt dữ liệu cũ với phiên điều khiển đã dừng.
Nếu heartbeat điều khiển thực sự mất, vẫn STOP theo hạn hiện tại. Web đọc lại
trạng thái sau đó không tự giữ lại quyền, RESET, HOME hoặc bật motor.

Trên LF, UART chạy trong **tiến trình riêng** ở SCHED_FIFO priority 20. Tiến
trình web không giữ GIL hoặc khóa của UART. IPC cục bộ dùng Unix seqpacket có
hạn chờ; web chết/đóng IPC thì worker UART gửi STOP và thoát, ESP vẫn có lease
20ms độc lập. Chạy bằng root để có quyền realtime; scheduler thực tế được ghi
trong diagnostics. Không tự reconnect, RESET, HOME hoặc ON sau lỗi.

Heartbeat trình duyệt dùng một WebSocket liên tục trong Web Worker: gửi mỗi
50ms, không chờ pong trước rồi mới gửi tiếp. HTTP chỉ phục vụ giao diện, đọc
trạng thái và lệnh nút bấm. WebSocket xác thực bằng frame đầu, chỉ gia hạn đúng
phiên đã Connect, không nhận lệnh motor. Pong mất300ms hoặc kênh đóng thì STOP.
Mỗi lần Connect tạo mã phiên mới; kênh cũ đóng không làm mất quyền của phiên mới.

Tests thêm:

```sh
node highlv/examples/check_motor/tests/test_heartbeat.js
```

Test process dùng pseudo-UART/IPC tạm, chặn GIL phía web và mô phỏng web đóng;
test worker dùng đồng hồ giả, pong trễ250ms và mất phản hồi300ms; test WebSocket
dùng localhost tạm. Tất cả không chạm motor/UART thật.

Khi khởi động, server tắt Wi-Fi power save trên `wlan0` (nếu interface tồn tại
và có lệnh `iw`) và đọc lại trạng thái; không restart mạng. Dùng
`--wifi-interface ""` để bỏ bước này. Thiết lập này không sửa file hệ thống;
mỗi lần chạy example sẽ áp dụng lại. Giao diện cũ dùng HTTP heartbeat bị từ chối
với thông báo Ctrl+F5, để không vô tình tiếp tục chạy bản cũ đã biết lỗi.

Mỗi lần bấm nút motor gửi đúng một HTTP command; LF xếp và truyền một frame C
xuống ESP, không tự gửi lại. Jog gửi một JOG và ESP tự hết nhịp sau300ms; không
có vòng lặp jog80ms trên web. Heartbeat WebSocket/P vẫn chạy riêng để giám sát
kết nối, không mang target/gains hoặc lặp lệnh motor.
