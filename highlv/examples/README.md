# Kiểm tra UART LF ↔ ESP

Hai example trao đổi văn bản qua UART 921600 baud, 8N1:

```text
LF  -> ESP: hello
ESP -> LF:  im here
```

Nạp example ở `lowlv/examples/check_hcr_link` vào ESP. Trên LF, dừng runtime
để nhường UART rồi chạy:

```sh
python3 /userdata/highlv/examples/check_hcr_link.py --port /dev/ttyS2
```

Script gửi `hello` mỗi giây và in `im here` khi ESP trả lời, liên tục tới khi
nhấn `Ctrl+C`. Dùng `--interval 0.5` để gửi mỗi nửa giây, hoặc `--once` để dừng
sau một phản hồi. Nếu mất phản hồi quá 8 giây, script báo FAIL.
ESP cũng in cả hai dòng trên console
USB native hoặc console Wi-Fi TCP 8023. Đây là test UART bằng văn bản, không sử dụng frame HCR1; tên file
được giữ từ example ban đầu.

## GUI kiểm tra motor

Xem [check_motor/README.md](check_motor/README.md): GUI web chạy trên LF, lệnh
đi qua UART đến ESP, điều khiển sáu motor và đọc feedback bus.
