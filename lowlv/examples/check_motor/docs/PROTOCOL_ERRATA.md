# Ghi chú đối chiếu tài liệu joint UART — 2026-10-30

Nguồn: `docs/Robot joint control communication documentation via 1-wire UART V1.0.0.html`.
Tài liệu gốc được giữ nguyên. Code `firmware/src/libs/joint_bus/joint_bus.c` tính checksum đúng.

Mục 7.1 có ví dụ TX MIT 17 byte, Kp=10, Kd=.036, position=4096, RPM=0, tau=0.
Checksum được ghi là **64**, nhưng các byte ID đến cuối parameter cộng thành
0x96; NOT8(0x96) = **69**. Khung đúng theo công thức của tài liệu là:

```text
FF FF 01 0D 03 1A 10 27 24 00 00 10 00 00 00 00 69
```

Khung RX 11 byte trong cùng ví dụ có checksum BD hợp lệ. Lỗi ở ví dụ TX
không thay đổi quy ước 16384 counts/rev, 1 RPM/unit, gains x1000 hoặc tau x1000.

`python3 tests/check_motor_bus.py` biên dịch `joint_bus.c` thực với UART/GPIO
mock, kiểm tra khung MIT trên, position write, wheel read, reply offsets,
echo/checksum error, timeout và broadcast. Không mở UART hay chạy motor.
