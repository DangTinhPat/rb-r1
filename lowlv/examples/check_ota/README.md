# check_ota: kiểm tra Wi-Fi, OTA và truyền nhận hai chiều

Bài kiểm tra tối thiểu để xác nhận ESP32-S3 nối được mạng kiểm thử, nạp được firmware qua mạng và trao đổi được
dữ liệu hai chiều với máy tính. Không điều khiển gì.

## Chuẩn kết nối kiểm thử

| Mục | Giá trị |
| --- | --- |
| Mạng Wi-Fi | `LUMI` |
| Hostname của ESP32 | `rb1-check` (mDNS: `rb1-check.local`) |
| Cổng TCP | `8023` |
| Chuỗi kiểm tra | `henlo` |

Thông tin mạng đã có sẵn trong repo riêng tư (`include/wifi_secrets.h`, `secrets.local.ini`). Muốn đổi mạng thì sửa hai file này,
hoặc tạo lại từ file mẫu:

```bash
cd examples/check_ota
cp include/wifi_secrets.example.h include/wifi_secrets.h      # điền SSID, mật khẩu Wi-Fi, mật khẩu OTA
cp secrets.example.ini secrets.local.ini                 # điền mật khẩu OTA (cùng giá trị ở trên)
```

## Chạy

```bash
cd examples/check_ota

# 1. Nạp lần đầu bằng cáp USB (bắt buộc)
pio run -e check_ota -t upload
pio device monitor                       # xem IP và trạng thái, 115200 baud

# 2. Kiểm tra truyền nhận: máy tính gửi henlo, ESP32 trả lại henlo
python3 tools/henlo.py                   # dùng rb1-check.local
python3 tools/henlo.py --host 192.168.x.x
python3 tools/henlo.py --once            # gửi một lần, thoát với mã 0 nếu đạt

# 3. Nạp qua Wi-Fi (từ lần sau không cần cáp)
pio run -e check_ota_air -t upload
```

## Hành vi

- ESP32 nối mạng, in IP, rồi mở OTA (hostname `rb1-check`) và máy chủ TCP cổng 8023.
- Máy tính gửi `henlo` thì ESP32 trả đúng `henlo`; gửi chuỗi khác thì ESP32 trả `echo: <chuỗi>`.
- ESP32 tự gửi `henlo from esp32 #N` mỗi 2 giây.
- Gõ chữ vào Serial Monitor thì ESP32 chuyển sang máy tính (hiện trên `henlo.py`); gõ chữ vào `henlo.py` thì hiện
  trên Serial Monitor.
- Mỗi lúc chỉ nhận một máy tính; máy thứ hai nhận `busy`.

Máy tính và ESP32 phải cùng mạng `LUMI`. Nếu `rb1-check.local` không phân giải được, dùng IP in trên Serial Monitor.
