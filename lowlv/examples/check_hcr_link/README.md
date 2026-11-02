# Kiểm tra UART LF ↔ ESP

Firmware example nhận dòng `hello` từ LF rồi trả dòng `im here` qua UART0.
ESP lặp lại phản hồi cho mỗi dòng `hello`; script LF mặc định gửi mỗi giây
đến khi nhấn `Ctrl+C`.
Console USB native hoặc Wi-Fi TCP 8023 in hai dòng nhận/gửi để xem ở đầu ESP.

```text
LF  -> ESP: hello
ESP -> LF:  im here
```

UART 921600 baud, 8N1. ESP TX43 → LF RX, ESP RX44 ← LF TX, chung GND.

Trước khi build, chép `include/wifi_secrets.h` và `secrets.local.ini` từ cấu hình
`check_imu` đang dùng vào example này. Các file thật được Git bỏ qua.

Build riêng example:

```sh
cd lowlv/examples/check_hcr_link
pio run -e check-hcr-link
```

Sau khi nạp example vào ESP, chạy trên LF:

```sh
python3 /userdata/highlv/examples/check_hcr_link.py --port /dev/ttyS2
```

Hai đầu dùng văn bản thuần để kiểm tra dây UART, không dùng frame HCR1.
Đây là firmware test riêng; không điều khiển motor. Wi-Fi/ArduinoOTA được giữ
để tiếp tục cập nhật qua mạng; console ESP qua TCP 8023 cũng in hai dòng nhận/gửi.

Nạp qua OTA của firmware đang chạy trên ESP:

```sh
pio run -e check-hcr-link-air -t upload --upload-port <IP_ESP>
```
