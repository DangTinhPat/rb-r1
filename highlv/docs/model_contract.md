# Beni512 checkpoint 19500 — contract inference

Actor của robot LU/RB v1.3, task flat, recurrent GRU một lớp 512 chiều.
Policy 100 Hz. Artifact ONNX FP32 và RKNN DFP16 hiện có trong `models/`.
Không kèm checkpoint PPO để train tiếp.

## Input/output tensor

| Tensor ONNX | Shape | Kiểu |
|---|---|---|
| `obs` | `[1,32]` | float32 |
| `h_in` | `[1,1,512]` | float32 |
| `actions` | `[1,6]` | float32 |
| `h_out` | `[1,1,512]` | float32 |

Actor: normalizer → GRU(32,512) → MLP 512→512→256→128→6, ELU.
Hidden khởi tạo 0 khi reset, được giữ khi handoff recovery → model.

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
vẫn có thể được core recovery dùng.

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

ONNX nhận raw observation; normalizer đã nhúng trong graph.
RKNN nhận observation được normalize trên CPU:
`normalized[i] = (obs[i] − mean[i]) / divisor[i]`, từ JSON normalizer đi kèm.
Không normalize hai lần. Tensor RKNN được quantize/dequantize qua native helper;
runtime kiểm dimension action6/hidden512 và float finite.

`models/manifest.json` chứa SHA256 các artifact. Runtime kiểm tra trước load.
Reference 100 bước recurrent nằm trong `tests/data/policy_reference.json`.

## Phạm vi command và servo

Runtime cho phép `vx` từ -2 tới +2 m/s, `wz` từ -π tới +π rad/s.
Pose command giữ cố định; jump chưa được bật vì thiếu command manager/phase gốc.

Model được kiểm trong plant dùng hip/crank PD Kp=30,Kd=1, torque cap tương ứng
±1,5/±5 Nm; wheel velocity gain 0,004 và cap ±0,13 Nm. Recovery sử dụng servo
tham chiếu khác. Các giá trị này mô tả điều kiện của plant kiểm trước đây;
không phải torque payload HCR1 và không tự trở thành gain driver thật.
Firmware motor cần được đo/đối chiếu đáp ứng riêng.
