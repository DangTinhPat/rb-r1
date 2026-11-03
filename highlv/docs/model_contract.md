# Policy fitted22500 — contract inference

Model hiện dùng: `models/fitted22500_dfp16_jumprec.rknn` với `models/fitted22500_normalizer.json`
(lấy từ `/userdata/lu_v13_gru256_hil/` trên LF). Recurrent GRU 512 chiều, policy 100 Hz.
Không có artifact ONNX, nên runtime chỉ có backend RKNN.

> **Chưa xác minh.** Phần dưới (observation 32D, action mapping, các command cố định) là contract
> mà code đang giả định. Chưa có tài liệu nào xác nhận model fitted22500 dùng đúng contract này.
> Tên `jumprec` gợi ý model có thể dùng lệnh nhảy/hồi phục (`obs[28:32]`, hiện cố định 0) hoặc
> scale action khác. Loader chỉ kiểm hình dạng tensor (obs 32, hidden 512, action 6) và hash
> artifact; đúng sai về điều khiển chỉ biết khi chạy thật.

## Input/output tensor

RKNN DFP16, hai input và hai output (loader từ chối model sai hình dạng):

| Tensor | Shape | Kiểu |
|---|---|---|
| obs (đã normalize) | `[1,32]` | float32 |
| hidden vào | `[1,512]` | float32 |
| action | `[1,6]` | float32 |
| hidden ra | `[1,512]` | float32 |

Hidden khởi tạo 0 khi reset, được giữ khi handoff gập → model.

## Observation 32D

| Slice | Dữ liệu |
|---|---|
| `[0:3]` | Gyro body `[wx,wy,wz]`, rad/s |
| `[3:6]` | Projected gravity body, vector đơn vị |
| `[6:9]` | Velocity command `[vx,0,wz]`, m/s và rad/s |
| `[9:13]` | q hipL/R,kneeL/R trừ `[-.7,-.7,.9,.9]`, rad |
| `[13:19]` | qd hipL/R,kneeL/R,wheelL/R, rad/s |
| `[19:25]` | Raw action trước, trước scale/offset |
| `[25:28]` | Pose command cố định `[0,0,.16]`, rad/rad/m |
| `[28:32]` | Jump command cố định `[0,0,0,0]` |

q knee là hinge: `crank − hip`. Góc tuyệt đối bánh, accel, torque và vận tốc
thân đo thật không nằm trong observation. Accel và optional hip torque
không nằm trong observation; kế hoạch gập chỉ dùng gravity và q hông.

## Action mapping

```text
hipL/R_ref    = -0.7 + 0.25 * a[0:2]       rad
crankL/R_ref  =  0.2 + 0.25 * a[2:4]       rad
wheelL/R_ref  = (0.8 / 0.035) * a[4:6]     rad/s
```

Đây là target tuyệt đối, không cộng vào góc đo hiện tại. Action không bị thêm
tanh hoặc clamp `[-1,1]`. Scale 0,035 là scale đã train, không tự đổi theo CAD.

Warm-up encode ngược từ applied target:

```text
a[0:2] = (hip_ref + 0.7) / 0.25
a[2:4] = (crank_ref - 0.2) / 0.25
a[4:6] = wheel_ref * 0.035 / 0.8
```

## Normalization và backend

RKNN nhận observation được normalize trên CPU:
`normalized[i] = (obs[i] − mean[i]) / divisor[i]`, từ JSON normalizer đi kèm.
Không normalize hai lần. Tensor RKNN được quantize/dequantize qua native helper;
runtime kiểm dimension action6/hidden512 và float finite.

`models/manifest.json` chứa SHA256 các artifact. Runtime kiểm tra trước load.
Không có tham chiếu số học cho model này (không có nguồn ONNX). `tests/data/policy_observations.json`
chỉ là chuỗi 100 observation để `scripts/verify_npu.py` phát lại trên LF và kiểm đầu ra hữu hạn,
deadline 10 ms và thời gian inference.

## Phạm vi command và servo

Runtime cho phép `vx` từ -2 tới +2 m/s, `wz` từ -π tới +π rad/s.
Pose command giữ cố định; jump chưa được bật vì thiếu command manager/phase gốc.

Model được kiểm trong plant dùng hip/crank PD Kp=30,Kd=1, torque cap tương ứng
±1,5/±5 Nm; wheel velocity gain 0,004 và cap ±0,13 Nm. Các giá trị này mô tả điều kiện của plant kiểm trước đây;
không phải torque payload HCR1 và không tự trở thành gain driver thật.
Firmware motor cần được đo/đối chiếu đáp ứng riêng.
