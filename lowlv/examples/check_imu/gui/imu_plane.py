#!/usr/bin/env python3
"""Máy bay pygame nhận dữ liệu IMU thật để kiểm hướng trục và bộ lọc. Không thuộc luồng điều khiển chính.

    pip install pygame pyserial
    python3 imu_plane.py                      # tự tìm /dev/ttyUSB* hoặc /dev/ttyACM*
    python3 imu_plane.py --port /dev/ttyUSB0
    python3 imu_plane.py --port socket://10.10.61.85:8023   # qua Wi-Fi, không cần cáp
    python3 imu_plane.py --demo               # dữ liệu giả lập, không cần board

Hệ trục robot: +X phía trước (mũi máy bay), +Y bên trái, +Z lên trên. Pitch dương = cúi mũi xuống, roll dương =
cánh trái nhấc lên (hông phải hạ xuống). Cánh trái tô đỏ, cánh phải tô xanh để nhìn ra hướng Y.

Phím tắt (cũng bấm được bằng chuột):
    F  đổi chế độ: FUSION (gia tốc + con quay) / ACCEL (chỉ gia tốc) / GYRO (chỉ con quay)
    1  lọc thông thấp gia tốc       2  lọc thông thấp con quay      3  trừ độ lệch con quay (bias)
    M  đổi bản đồ trục chip -> thân C  đo bias con quay 2 giây (giữ robot đứng yên)
    R  đặt lại tư thế               Space  tạm dừng        +/-  tăng/giảm hệ số hiệu chỉnh FUSION
    Esc thoát
"""
import argparse
import glob
import math
import os
import sys
import threading
import time
from collections import deque

G = 9.80665

# ---------------------------------------------------------------- bản đồ trục chip -> thân (giống ImuMount.hpp)
# thân[i] = dấu[i] * chip[trục[i]]
MOUNTS = [
    ("Úp xuống: X trước, Y phải, Z xuống (hiện tại)", (0, 1, 2), (1.0, -1.0, -1.0)),
    ("Trùng trục thân: X trước, Y trái, Z lên", (0, 1, 2), (1.0, 1.0, 1.0)),
    ("Ngửa: chip Y trước, chip X phải, Z lên", (1, 0, 2), (1.0, -1.0, 1.0)),
    ("Ngửa: chip X trái, chip Y sau, Z lên", (1, 0, 2), (-1.0, 1.0, 1.0)),
]


def to_body(mount, chip):
    _, axis, sign = mount
    return [sign[i] * chip[axis[i]] for i in range(3)]


def to_chip(mount, body):
    """Ngược của to_body (dùng cho dữ liệu giả lập)."""
    _, axis, sign = mount
    chip = [0.0, 0.0, 0.0]
    for i in range(3):
        chip[axis[i]] = sign[i] * body[i]
    return chip


# ---------------------------------------------------------------- toán quaternion / ma trận
def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def qnorm(q):
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return tuple(c / n for c in q)


def quat_from_euler(yaw, pitch, roll):
    """ZYX: q = Rz(yaw) Ry(pitch) Rx(roll); pitch dương = cúi mũi xuống (quay phải tay quanh +Y)."""
    qz = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
    qy = (math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0)
    qx = (math.cos(roll / 2), math.sin(roll / 2), 0.0, 0.0)
    return qmul(qz, qmul(qy, qx))


def rot_matrix(q):
    """Ma trận quay thân -> thế giới."""
    w, x, y, z = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def euler_from_quat(q):
    r = rot_matrix(q)
    pitch = -math.asin(max(-1.0, min(1.0, r[2][0])))
    roll = math.atan2(r[2][1], r[2][2])
    yaw = math.atan2(r[1][0], r[0][0])
    return yaw, pitch, roll


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def accel_roll_pitch(a):
    roll = math.atan2(a[1], a[2])
    pitch = math.atan2(-a[0], math.hypot(a[1], a[2]))
    return roll, pitch


# ---------------------------------------------------------------- bộ lọc tư thế
class Attitude:
    MODES = ("FUSION", "ACCEL", "GYRO")

    def __init__(self):
        self.mode = "FUSION"
        self.accel_lpf = False       # thông thấp gia tốc
        self.gyro_lpf = False        # thông thấp con quay
        self.use_bias = True         # trừ bias con quay (bias chip đo bằng C hoặc tự học)
        self.auto_bias = True        # tự học bias khi robot đứng yên
        self.yaw_lock = True         # coi vận tốc góc bằng 0 khi robot đứng yên (chống trôi yaw)
        self.use_integral = True     # thành phần tích phân kiểu Mahony: bù bias con quay trục X, Y
        self.ki = 0.2                # hệ số tích phân [1/s^2]
        self.accel_fc_hz = 5.0
        self.gyro_fc_hz = 20.0
        self.kp = 2.0                # hệ số hiệu chỉnh FUSION [1/s]
        self.reset()

    def reset(self):
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.yaw = 0.0
        self.integral = [0.0, 0.0, 0.0]
        self._a = None
        self._g = None
        self._need_init = True       # căn lại theo gia tốc ở mẫu đầu tiên sau khi đặt lại

    def reset_yaw(self):
        _, pitch, roll = euler_from_quat(self.q)
        self.yaw = 0.0
        self.q = quat_from_euler(0.0, pitch, roll)

    def _lpf(self, prev, x, fc, dt):
        if prev is None:
            return list(x)
        alpha = dt / (dt + 1.0 / (2.0 * math.pi * fc))
        return [p + alpha * (v - p) for p, v in zip(prev, x)]

    def update(self, accel_body, gyro_body, dt):
        a, g = list(accel_body), list(gyro_body)
        if self.accel_lpf:
            self._a = self._lpf(self._a, a, self.accel_fc_hz, dt)
            a = self._a
        else:
            self._a = None
        if self.gyro_lpf:
            self._g = self._lpf(self._g, g, self.gyro_fc_hz, dt)
            g = self._g
        else:
            self._g = None
        if self.mode == "ACCEL":
            roll, pitch = accel_roll_pitch(a)
            self.yaw += g[2] * dt                      # accel không có yaw: lấy từ con quay
            self.q = quat_from_euler(self.yaw, pitch, roll)
            return
        w = list(g)
        if self.mode == "FUSION":
            norm = math.sqrt(sum(c * c for c in a))
            if norm > 1e-3 and abs(norm - G) < 3.0:  # bỏ hiệu chỉnh khi đang bị gia tốc mạnh
                an = tuple(c / norm for c in a)
                r = rot_matrix(self.q)
                up_body = (r[2][0], r[2][1], r[2][2])   # hướng "lên" của thế giới trong hệ thân
                if self._need_init or sum(an[i] * up_body[i] for i in range(3)) < 0.5:
                    # lệch quá 60° (hoặc vừa đặt lại): căn thẳng theo gia tốc, giữ yaw; nếu không, khi chip lật
                    # ngược so với thân thì phép hiệu chỉnh bằng 0 và máy bay trông như nằm phẳng
                    roll, pitch = accel_roll_pitch(a)
                    self.q = quat_from_euler(euler_from_quat(self.q)[0], pitch, roll)
                    self._need_init = False
                    return
                e = cross(an, up_body)
                if self.use_integral:
                    # Mahony PI: tích phân của sai số nghiêng bù bias không đổi của con quay. Trọng lực chỉ cho
                    # thấy bias vuông góc với nó, nên bias quanh trục yaw không ước lượng được ở đây.
                    self.integral = [max(-0.4, min(0.4, self.integral[i] + self.ki * e[i] * dt)) for i in range(3)]
                    # Bỏ thành phần song song với trọng lực: gia tốc kế không quan sát được nó, và khi board nghiêng
                    # phần tích phân cũ rò sang đó thành vận tốc xoay yaw giả.
                    along = sum(self.integral[i] * up_body[i] for i in range(3))
                    w = [w[i] + self.integral[i] - along * up_body[i] for i in range(3)]
                w = [w[i] + self.kp * e[i] for i in range(3)]
        dq = qmul(self.q, (0.0, w[0], w[1], w[2]))
        self.q = qnorm(tuple(self.q[i] + 0.5 * dq[i] * dt for i in range(4)))


class BiasTracker:
    """Nhận ra đứng yên và theo dõi bias con quay, theo cách các bộ hiệu chuẩn con quay lúc chạy (Android GyroCal) và
    ZUPT/ZARU trong dẫn đường quán tính làm:
      - cửa sổ phân tích 1.5 giây; "yên" khi độ lệch chuẩn của con quay và gia tốc đều nhỏ và |a| gần g;
      - lần đầu phải yên liên tục ACQUIRE_S giây mới nhận bias (hoặc đo tay bằng phím C);
      - sau đó chỉ coi là đứng yên khi trung bình con quay nằm trong MEAN_BOUND quanh bias hiện tại, nên chuyển động
        xoay đều (độ lệch chuẩn nhỏ nhưng trung bình xa bias) không bị nhầm thành độ lệch; bias cập nhật chậm;
      - yên rất lâu (REACQUIRE_S) mà trung bình vẫn lệch bias thì bias đã đổi (nhiệt độ...): lấy bias mới, nhưng chỉ
        khi độ nhảy nhỏ hơn REACQUIRE_JUMP.
    Giới hạn: không có từ kế nên không phân biệt được xoay đều chậm (dưới MEAN_BOUND, hoặc dưới REACQUIRE_JUMP và dài
    hơn REACQUIRE_S giây) với đứng yên.
    """

    WINDOW_S = 1.5        # giây
    GYRO_STD = 0.0087     # rad/s, tương đương phương sai 7.5e-5 (rad/s)^2 của GyroCal
    ACCEL_STD = 0.10      # m/s^2
    ACQUIRE_S = 3.0
    REACQUIRE_S = 30.0
    REACQUIRE_JUMP = 0.10 # rad/s: bias thật đổi chậm (nhiệt độ); nhảy lớn hơn mức này là chuyển động, không nhận
    MEAN_BOUND = 0.035    # rad/s, khoảng 2 độ/giây
    ACQUIRE_MAX = 0.35    # rad/s: bias đầu tiên không được lớn hơn mức này
    TAU_S = 5.0           # hằng số thời gian cập nhật bias

    def __init__(self):
        self.buf = deque()                                  # (t, con quay - ref, gia tốc - ref)
        self.ref_g = self.ref_a = None
        self._reset_sums()
        self.bias = [0.0, 0.0, 0.0]
        self.valid = False
        self.quiet = False    # dao động nhỏ
        self.still = False    # dao động nhỏ và khớp bias: dùng cho ZUPT
        self.quiet_s = 0.0
        self.acquired = False   # vừa nhận bias lần đầu: để đặt lại yaw
        self.mean = [0.0, 0.0, 0.0]
        self.t_last = None
        self._t_eval = None
        self._pushes = 0

    def _reset_sums(self):
        self.s_g, self.s_gg = [0.0] * 3, [0.0] * 3
        self.s_a, self.s_aa = [0.0] * 3, [0.0] * 3

    def set_bias(self, bias):
        self.bias, self.valid, self.quiet_s = list(bias), True, 0.0

    def push(self, t, chip_gyro, chip_accel):
        """Thêm một mẫu; giữ các tổng chạy trên cửa sổ WINDOW_S giây nên mỗi mẫu tốn O(1)."""
        if self.ref_g is None:
            self.ref_g, self.ref_a = list(chip_gyro), list(chip_accel)
        dg = [chip_gyro[i] - self.ref_g[i] for i in range(3)]
        da = [chip_accel[i] - self.ref_a[i] for i in range(3)]
        for i in range(3):
            self.s_g[i] += dg[i]
            self.s_gg[i] += dg[i] * dg[i]
            self.s_a[i] += da[i]
            self.s_aa[i] += da[i] * da[i]
        self.buf.append((t, dg, da))
        self.t_last = t
        while self.buf and t - self.buf[0][0] > self.WINDOW_S:
            _, og, oa = self.buf.popleft()
            for i in range(3):
                self.s_g[i] -= og[i]
                self.s_gg[i] -= og[i] * og[i]
                self.s_a[i] -= oa[i]
                self.s_aa[i] -= oa[i] * oa[i]
        self._pushes += 1
        if self._pushes % 20000 == 0:                       # chống sai số làm tròn tích lũy
            self._reset_sums()
            for _, g, a in self.buf:
                for i in range(3):
                    self.s_g[i] += g[i]
                    self.s_gg[i] += g[i] * g[i]
                    self.s_a[i] += a[i]
                    self.s_aa[i] += a[i] * a[i]

    def evaluate(self, auto):
        n = len(self.buf)
        if n < 50 or self.buf[-1][0] - self.buf[0][0] < 0.9 * self.WINDOW_S:
            self.quiet = self.still = False
            self._t_eval = self.t_last
            return
        mg = [self.ref_g[i] + self.s_g[i] / n for i in range(3)]
        sg = max(math.sqrt(max(0.0, self.s_gg[i] / n - (self.s_g[i] / n) ** 2)) for i in range(3))
        sa = math.sqrt(sum(max(0.0, self.s_aa[i] / n - (self.s_a[i] / n) ** 2) for i in range(3)))
        ma = [self.ref_a[i] + self.s_a[i] / n for i in range(3)]
        quiet = sg < self.GYRO_STD and sa < self.ACCEL_STD and abs(math.sqrt(sum(c * c for c in ma)) - G) < 1.0
        elapsed = 0.0 if self._t_eval is None or self.t_last is None else self.t_last - self._t_eval
        self._t_eval = self.t_last
        self.quiet_s = self.quiet_s + elapsed if quiet else 0.0
        self.quiet, self.mean = quiet, mg
        dev = max(abs(mg[i] - self.bias[i]) for i in range(3))
        if auto and quiet:
            if not self.valid:
                if self.quiet_s >= self.ACQUIRE_S and max(abs(c) for c in mg) < self.ACQUIRE_MAX:
                    self.bias, self.valid, self.acquired = list(mg), True, True
            elif dev >= self.MEAN_BOUND and self.quiet_s >= self.REACQUIRE_S and dev < self.REACQUIRE_JUMP:
                self.bias, self.quiet_s = list(mg), 0.0
        self.still = quiet and self.valid and dev < self.MEAN_BOUND


class Processor:
    """Từ mẫu thô của chip đến tư thế: trừ bias, ZUPT, bộ lọc. Dùng chung cho GUI và bài kiểm tra."""

    def __init__(self):
        self.att = Attitude()
        self.trk = BiasTracker()
        self.last_t = None

    def feed(self, sample, mount):
        att, trk = self.att, self.trk
        dt = 0.002 if self.last_t is None else max(0.0002, min(0.25, sample.t - self.last_t))
        self.last_t = sample.t
        a, g = sample.chip_accel, sample.chip_gyro
        trk.push(sample.t, g, a)
        if trk.still and att.auto_bias:
            k = min(1.0, dt / trk.TAU_S)
            trk.bias = [b + k * (m - b) for b, m in zip(trk.bias, trk.mean)]
        gyro_body = to_body(mount, [g[i] - (trk.bias[i] if att.use_bias else 0.0) for i in range(3)])
        if trk.still and att.yaw_lock:
            gyro_body = [0.0, 0.0, 0.0]               # đứng yên thì vận tốc góc thật bằng 0
        att.update(to_body(mount, a), gyro_body, dt)
        return gyro_body

    def end_frame(self):
        self.trk.evaluate(self.att.auto_bias)
        if self.trk.acquired:                         # vừa đo xong bias: yaw bắt đầu từ 0
            self.trk.acquired = False
            self.att.reset_yaw()


# ---------------------------------------------------------------- nguồn dữ liệu
class Sample:
    __slots__ = ("t", "chip_accel", "chip_gyro")

    def __init__(self, t, chip_accel, chip_gyro):
        self.t, self.chip_accel, self.chip_gyro = t, chip_accel, chip_gyro


def parse_line(line):
    """'$IMU,ms,ax,ay,az,gx,gy,gz' -> Sample hoặc None."""
    if not line.startswith("$IMU,"):
        return None
    parts = line.strip().split(",")
    if len(parts) != 8:
        return None
    try:
        v = [float(p) for p in parts[1:]]
    except ValueError:
        return None
    return Sample(v[0] / 1000.0, v[1:4], v[4:7])


class SerialSource:
    def __init__(self, port, baud):
        import serial
        # serial_for_url nhận cả /dev/ttyUSB0 lẫn socket://<IP>:8023 (dữ liệu qua Wi-Fi)
        self.ser = serial.serial_for_url(port, baudrate=baud, timeout=0.1)
        self.queue = deque(maxlen=4000)
        self.bad = 0
        self.info = ""
        self._stop = False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        buf = b""
        while not self._stop:
            try:
                data = self.ser.read(512)
            except Exception:
                time.sleep(0.2)
                continue
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if line.startswith("#"):
                    self.info = line
                    continue
                s = parse_line(line)
                if s is None:
                    if line:
                        self.bad += 1
                else:
                    self.queue.append(s)

    def poll(self):
        out = []
        while self.queue:
            out.append(self.queue.popleft())
        return out


class DemoSource:
    """Dữ liệu giả lập 200 Hz, tạo ra theo hệ trục chip của bản đồ đầu tiên (úp xuống)."""

    def __init__(self):
        self.t0 = time.monotonic()
        self.n = 0
        self.info = "# demo: du lieu gia lap"
        self.bad = 0
        self.mount = MOUNTS[0]
        self.noise = 0

    def poll(self):
        out = []
        target = int((time.monotonic() - self.t0) * 200)
        while self.n < target:
            self.n += 1
            t = self.n / 200.0
            roll = 0.5 * math.sin(2 * math.pi * 0.20 * t)
            pitch = 0.4 * math.sin(2 * math.pi * 0.13 * t + 1.0)
            d = 1e-3
            roll2 = 0.5 * math.sin(2 * math.pi * 0.20 * (t + d))
            pitch2 = 0.4 * math.sin(2 * math.pi * 0.13 * (t + d) + 1.0)
            p = (roll2 - roll) / d
            q = (pitch2 - pitch) / d
            body_accel = [-G * math.sin(pitch), G * math.cos(pitch) * math.sin(roll), G * math.cos(pitch) * math.cos(roll)]
            body_gyro = [p, q, 0.0]
            self.noise = (self.noise * 1103515245 + 12345) & 0x7FFFFFFF
            jitter = ((self.noise / 0x7FFFFFFF) - 0.5) * 0.4
            body_accel = [a + jitter for a in body_accel]
            out.append(Sample(t, to_chip(self.mount, body_accel), to_chip(self.mount, body_gyro)))
        return out


# ---------------------------------------------------------------- mô hình máy bay (hệ thân: +X mũi, +Y trái, +Z lên)
def plane_faces():
    """[(đỉnh..., màu)]"""
    grey, dark, red, green, orange, fin = (170, 180, 195), (120, 130, 150), (225, 60, 60), (60, 200, 90), (240, 150, 40), (80, 120, 220)
    w = 0.13
    nose, tail = (1.25, 0, 0), (-1.0, 0, 0)
    f1, f2 = 0.35, -0.55  # hai mặt cắt thân
    def sec(x, h): return [(x, -w, -w * h), (x, w, -w * h), (x, w, w * h), (x, -w, w * h)]
    a, b = sec(f1, 1.0), sec(f2, 1.0)
    faces = []
    # thân: mũi -> mặt cắt 1 -> mặt cắt 2 -> đuôi
    for i in range(4):
        j = (i + 1) % 4
        faces.append(([nose, a[i], a[j]], orange))
        faces.append(([a[i], a[j], b[j], b[i]], grey))
        faces.append(([b[i], b[j], tail], dark))
    # cánh (trái đỏ, phải xanh)
    faces.append(([(0.30, 0.13, 0.0), (-0.12, 1.15, 0.0), (-0.38, 1.15, 0.0), (-0.45, 0.13, 0.0)], red))
    faces.append(([(0.30, -0.13, 0.0), (-0.12, -1.15, 0.0), (-0.38, -1.15, 0.0), (-0.45, -0.13, 0.0)], green))
    # cánh đuôi ngang
    faces.append(([(-0.62, 0.1, 0.0), (-0.85, 0.5, 0.0), (-1.0, 0.5, 0.0), (-1.0, 0.1, 0.0)], dark))
    faces.append(([(-0.62, -0.1, 0.0), (-0.85, -0.5, 0.0), (-1.0, -0.5, 0.0), (-1.0, -0.1, 0.0)], dark))
    # đuôi đứng
    faces.append(([(-0.55, 0.0, 0.12), (-0.95, 0.0, 0.12), (-1.02, 0.0, 0.55), (-0.82, 0.0, 0.55)], fin))
    return faces


# ---------------------------------------------------------------- giao diện
class Button:
    def __init__(self, key, rect):
        self.key, self.rect = key, rect


def run(args):
    if args.screenshot:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame

    if args.demo:
        source = DemoSource()
    else:
        port = args.port
        if not port:
            ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
            if not ports:
                print("Không thấy cổng serial. Cắm board, hoặc chạy với --demo, hoặc chỉ định --port.")
                return 1
            port = ports[0]
        try:
            source = SerialSource(port, args.baud)
        except Exception as exc:
            print(f"Không mở được {port}: {exc}")
            return 1
        print(f"Đang đọc {port} ở {args.baud} baud")

    pygame.init()
    W, H, PANEL = 1180, 780, 400
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("RB1 · kiểm tra IMU")
    clock = pygame.time.Clock()
    fonts = ["dejavusans", "liberationsans", "arial", "freesans", None]
    font = pygame.font.SysFont(fonts, 16)
    big = pygame.font.SysFont(fonts, 22, bold=True)
    small = pygame.font.SysFont(fonts, 14)

    proc = Processor()
    att, trk = proc.att, proc.trk
    mount_idx = 0
    paused = False
    rate_hz, rate_n, rate_t = 0.0, 0, time.monotonic()
    last_t = None
    chip_a, chip_g = [0.0] * 3, [0.0] * 3
    cal = {"active": False, "t0": 0.0, "acc": [0.0, 0.0, 0.0], "n": 0}
    calib_msg = ""
    faces = plane_faces()
    frames = 0

    def project(v, cam):
        r, u, f, d = cam
        x = sum(v[i] * r[i] for i in range(3))
        y = sum(v[i] * u[i] for i in range(3))
        z = sum(v[i] * f[i] for i in range(3))
        s = 640.0 / (d + z)
        return (W - PANEL) / 2 + x * s * 1.0, H / 2 + 10 - y * s, z

    def camera(az_deg=200.0, el_deg=24.0, dist=4.2):
        az, el = math.radians(az_deg), math.radians(el_deg)
        pos = (dist * math.cos(el) * math.cos(az), dist * math.cos(el) * math.sin(az), dist * math.sin(el))
        f = tuple(-c / dist for c in pos)
        r = cross(f, (0, 0, 1))
        rn = math.sqrt(sum(c * c for c in r))
        r = tuple(c / rn for c in r)
        u = cross(r, f)
        return r, u, f, dist

    cam = camera()

    def layout():
        """(tiêu đề nhóm, danh sách nút). Mỗi nút: (Button, loại, nhãn, đang bật)."""
        x0, w = W - PANEL + 20, PANEL - 40
        heads, btns = [("CHẾ ĐỘ LỌC  (phím F)", 96), ("BỘ LỌC  BẬT / TẮT", 160), ("CÔNG CỤ", 388)], []
        seg_w = w // 3
        for i, mode in enumerate(Attitude.MODES):
            btns.append((Button("mode:" + mode, pygame.Rect(x0 + i * seg_w, 116, seg_w - 4, 34)), "segment", mode,
                         att.mode == mode))
        toggles = [("1", f"Lọc gia tốc  (1)  {att.accel_fc_hz:g} Hz", att.accel_lpf),
                   ("2", f"Lọc con quay  (2)  {att.gyro_fc_hz:g} Hz", att.gyro_lpf),
                   ("3", "Trừ bias con quay  (3)", att.use_bias),
                   ("4", "Tự học bias khi đứng yên  (4)", att.auto_bias),
                   ("5", "Khóa yaw khi đứng yên  (5)", att.yaw_lock),
                   ("6", f"Tích phân Mahony  (6)  Ki {att.ki:g}", att.use_integral)]
        for i, (key, label, on) in enumerate(toggles):
            btns.append((Button(key, pygame.Rect(x0, 180 + i * 35, w, 31)), "toggle", label, on))
        actions = [("m", "Bản đồ trục (M)"), ("c", "Đo bias (C)" if not cal["active"] else "Đang đo..."),
                   ("r", "Đặt lại (R)"), ("space", "Tạm dừng" if not paused else "Tiếp tục")]
        half = w // 2
        for i, (key, label) in enumerate(actions):
            btns.append((Button(key, pygame.Rect(x0 + (i % 2) * half, 408 + (i // 2) * 36, half - 4, 32)), "action",
                         label, False))
        return heads, btns

    def handle(key):
        nonlocal mount_idx, paused, calib_msg
        if key.startswith("mode:"):
            att.mode = key[5:]
            att.reset()
        elif key == "f":
            att.mode = Attitude.MODES[(Attitude.MODES.index(att.mode) + 1) % 3]
            att.reset()
        elif key == "1":
            att.accel_lpf = not att.accel_lpf
        elif key == "2":
            att.gyro_lpf = not att.gyro_lpf
        elif key == "3":
            att.use_bias = not att.use_bias
        elif key == "4":
            att.auto_bias = not att.auto_bias
        elif key == "5":
            att.yaw_lock = not att.yaw_lock
        elif key == "6":
            att.use_integral = not att.use_integral
        elif key == "m":
            mount_idx = (mount_idx + 1) % len(MOUNTS)
            att.reset()
        elif key == "c":
            cal.update(active=True, t0=time.monotonic(), acc=[0.0, 0.0, 0.0], n=0)
            calib_msg = "Giữ robot đứng yên 2 giây..."
        elif key == "r":
            att.reset()
        elif key == "space":
            paused = not paused
        elif key == "+":
            att.kp = min(10.0, att.kp * 1.25)
        elif key == "-":
            att.kp = max(0.05, att.kp / 1.25)

    keymap = {pygame.K_f: "f", pygame.K_1: "1", pygame.K_2: "2", pygame.K_3: "3", pygame.K_4: "4", pygame.K_5: "5", pygame.K_6: "6", pygame.K_m: "m", pygame.K_c: "c",
              pygame.K_r: "r", pygame.K_SPACE: "space", pygame.K_PLUS: "+", pygame.K_EQUALS: "+", pygame.K_KP_PLUS: "+",
              pygame.K_MINUS: "-", pygame.K_KP_MINUS: "-"}

    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                running = False
            elif ev.type == pygame.KEYDOWN and ev.key in keymap:
                handle(keymap[ev.key])
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                for btn, _kind, _label, _on in layout()[1]:
                    if btn.rect.collidepoint(ev.pos):
                        handle(btn.key)

        mount = MOUNTS[mount_idx]
        samples = source.poll()
        if paused:
            samples = []
        for s in samples:
            chip_a, chip_g = s.chip_accel, s.chip_gyro
            if cal["active"]:
                cal["acc"] = [cal["acc"][i] + chip_g[i] for i in range(3)]
                cal["n"] += 1
                if time.monotonic() - cal["t0"] > 2.0 and cal["n"] > 20:
                    trk.set_bias([c / cal["n"] for c in cal["acc"]])
                    att.reset_yaw()
                    cal["active"] = False
                    att.use_bias = True
                    calib_msg = "bias chip (rad/s): " + " ".join(f"{b:+.4f}" for b in trk.bias)
            proc.feed(s, mount)
            rate_n += 1
        proc.end_frame()
        now = time.monotonic()
        if now - rate_t >= 1.0:
            rate_hz, rate_n, rate_t = rate_n / (now - rate_t), 0, now

        # ---- vẽ
        screen.fill((24, 28, 36))
        pygame.draw.rect(screen, (34, 39, 50), (W - PANEL, 0, PANEL, H))
        r = rot_matrix(att.q)
        # lưới mặt đất
        for i in range(-4, 5):
            for a, b in (((i * 0.4, -1.6, -1.0), (i * 0.4, 1.6, -1.0)), ((-1.6, i * 0.4, -1.0), (1.6, i * 0.4, -1.0))):
                p1, p2 = project(a, cam), project(b, cam)
                pygame.draw.line(screen, (52, 60, 76), p1[:2], p2[:2], 1)
        # trục thế giới nhỏ ở góc: hướng X thế giới (mũi khi yaw = 0)
        polys = []
        light = (0.35, -0.25, 0.9)
        ln = math.sqrt(sum(c * c for c in light))
        light = tuple(c / ln for c in light)
        for verts, color in faces:
            wv = [tuple(sum(r[i][j] * v[j] for j in range(3)) for i in range(3)) for v in verts]
            n = cross(tuple(wv[1][k] - wv[0][k] for k in range(3)), tuple(wv[2][k] - wv[0][k] for k in range(3)))
            nn = math.sqrt(sum(c * c for c in n)) or 1.0
            shade = 0.45 + 0.55 * abs(sum(n[k] / nn * light[k] for k in range(3)))
            pts = [project(v, cam) for v in wv]
            depth = sum(p[2] for p in pts) / len(pts)
            polys.append((depth, [p[:2] for p in pts], tuple(int(c * shade) for c in color)))
        for depth, pts, color in sorted(polys, key=lambda t: -t[0]):
            pygame.draw.polygon(screen, color, pts)
            pygame.draw.polygon(screen, (15, 18, 24), pts, 1)
        # trục thân: X đỏ (trước), Y xanh lá (trái), Z xanh dương (lên)
        origin = project((0, 0, 0), cam)[:2]
        for vec, col, name in (((1.7, 0, 0), (255, 90, 90), "X trước"), ((0, 1.5, 0), (90, 255, 120), "Y trái"),
                               ((0, 0, 1.1), (110, 160, 255), "Z lên")):
            wv = tuple(sum(r[i][j] * vec[j] for j in range(3)) for i in range(3))
            end = project(wv, cam)[:2]
            pygame.draw.line(screen, col, origin, end, 2)
            screen.blit(small.render(name, True, col), (end[0] + 4, end[1] - 8))

        yaw, pitch, roll = euler_from_quat(att.q)
        body_a = to_body(mount, chip_a)
        body_g = to_body(mount, [chip_g[i] - (trk.bias[i] if att.use_bias else 0.0) for i in range(3)])
        x0 = W - PANEL + 20
        screen.blit(big.render("Kiểm tra IMU", True, (235, 240, 250)), (x0, 16))
        info = source.info if source.info else "Chưa nhận thông báo từ board"
        screen.blit(small.render(info[:52], True, (160, 170, 190)), (x0, 48))
        screen.blit(small.render(f"Tốc độ nhận {rate_hz:5.1f} Hz   dòng lỗi {source.bad}", True, (160, 170, 190)),
                    (x0, 68))
        mx, my = pygame.mouse.get_pos()
        heads, btns = layout()
        for text, hy in heads:
            screen.blit(small.render(text, True, (140, 150, 170)), (x0, hy))
        for btn, kind, label, on in btns:
            r = btn.rect
            hot = r.collidepoint((mx, my))
            idle = (62, 74, 98) if hot else (48, 56, 74)
            if kind == "segment":
                pygame.draw.rect(screen, (70, 130, 220) if on else idle, r, border_radius=6)
                txt = font.render(label, True, (255, 255, 255) if on else (200, 208, 222))
                screen.blit(txt, (r.centerx - txt.get_width() // 2, r.centery - txt.get_height() // 2))
            elif kind == "toggle":
                pygame.draw.rect(screen, idle, r, border_radius=6)
                screen.blit(font.render(label, True, (230, 236, 248)), (r.x + 10, r.centery - 9))
                pill = pygame.Rect(r.right - 82, r.y + 5, 72, r.h - 10)
                pygame.draw.rect(screen, (50, 170, 100) if on else (92, 98, 112), pill, border_radius=12)
                knob_x = pill.right - 12 if on else pill.x + 12
                pygame.draw.circle(screen, (245, 248, 250), (knob_x, pill.centery), 8)
                tag = small.render("BẬT" if on else "TẮT", True, (255, 255, 255))
                tag_x = pill.x + 9 if on else pill.right - 9 - tag.get_width()
                screen.blit(tag, (tag_x, pill.centery - tag.get_height() // 2))
            else:
                pygame.draw.rect(screen, idle, r, border_radius=6)
                txt = font.render(label, True, (230, 236, 248))
                screen.blit(txt, (r.centerx - txt.get_width() // 2, r.centery - txt.get_height() // 2))

        y = 484
        lines = [
            ("Bản đồ trục:", (160, 170, 190)),
            (mount[0], (255, 220, 120)),
            (f"Roll  {math.degrees(roll):+7.1f}°   Pitch {math.degrees(pitch):+7.1f}°   Yaw {math.degrees(yaw):+7.1f}°",
             (235, 240, 250)),
            (f"Chip  a {chip_a[0]:+6.2f} {chip_a[1]:+6.2f} {chip_a[2]:+6.2f}", (190, 200, 215)),
            (f"Chip  ω {chip_g[0]:+6.3f} {chip_g[1]:+6.3f} {chip_g[2]:+6.3f}", (190, 200, 215)),
            (f"Thân  a {body_a[0]:+6.2f} {body_a[1]:+6.2f} {body_a[2]:+6.2f}", (235, 240, 250)),
            (f"Thân  ω {body_g[0]:+6.3f} {body_g[1]:+6.3f} {body_g[2]:+6.3f}", (235, 240, 250)),
        ]
        if att.mode == "FUSION":
            lines.append((f"Hệ số hiệu chỉnh Kp = {att.kp:.2f}  (+/- để đổi)", (160, 170, 190)))
        if not trk.valid:
            status = ("Bias: chờ yên liên tục khoảng 4.5 giây để đo" if att.auto_bias else "Bias: chưa có, bấm C",
                      (255, 200, 120))
        elif trk.still:
            status = ("Trạng thái: ĐỨNG YÊN" + ("  (đang học bias)" if att.auto_bias else ""), (120, 230, 140))
        else:
            status = ("Trạng thái: chuyển động", (190, 200, 215))
        lines.append(status)
        lines.append((f"Bias chip  {trk.bias[0]:+.4f} {trk.bias[1]:+.4f} {trk.bias[2]:+.4f} rad/s", (160, 170, 190)))
        for text, col in lines:
            screen.blit(small.render(text, True, col), (x0, y))
            y += 19
        # kiểm tra nhanh khi nằm yên
        is_still = trk.still
        y += 4
        screen.blit(small.render("Kiểm tra khi nằm yên:", True, (160, 170, 190)), (x0, y))
        y += 20
        ok_z = is_still and abs(body_a[2] - G) < 1.0
        msg = ("đang chuyển động, hãy giữ yên" if not is_still else
               ("✔ Z thân ≈ +9.81 (đúng)" if ok_z else "✘ Z thân không ≈ +9.81: sai bản đồ trục"))
        screen.blit(small.render(msg, True, (120, 230, 140) if ok_z else (255, 150, 120)), (x0, y))
        if calib_msg:
            screen.blit(small.render(calib_msg[:54], True, (255, 220, 120)), (x0, y + 22))
        screen.blit(small.render("Cánh trái ĐỎ, cánh phải XANH, mũi CAM", True, (150, 160, 180)), (20, H - 28))

        pygame.display.flip()
        clock.tick(60)
        frames += 1
        if args.screenshot and frames >= args.frames:
            pygame.image.save(screen, args.screenshot)
            print(f"Đã lưu {args.screenshot}")
            running = False
    pygame.quit()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="cổng serial hoặc socket://<IP>:8023; mặc định tự tìm /dev/ttyUSB* hoặc /dev/ttyACM*")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--demo", action="store_true", help="dữ liệu giả lập, không cần board")
    ap.add_argument("--screenshot", help="lưu ảnh cửa sổ sau --frames khung hình rồi thoát (chạy không màn hình)")
    ap.add_argument("--frames", type=int, default=90)
    return run(ap.parse_args())


if __name__ == "__main__":
    sys.exit(main())
