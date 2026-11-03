"""Telemetry-driven 200 Hz hybrid state machine; policy ticks at 100 Hz."""

import math
import secrets
import struct
import time
from collections import deque
from . import protocol as p
from .policy import observation, encode_targets, decode_actions

TELEMETRY_PERIOD_US = 5000  # 200 Hz MCU telemetry
MODES = {
    "WAIT_LINK": 0,
    "DISARMED": 1,
    "HOME": 2,
    "STANDUP": 3,
    "WARMUP": 4,
    "MODEL": 5,
    "FAULT": 6,
}
PHASES = ["Idle", "Folding", "Folded"]
FAULTS = {
    "telemetry_timeout": 1,
    "invalid_telemetry": 2,
    "driver_fault": 3,
    "clock_gap": 4,
    "missing_applied_target": 5,
    "logic_fault": 6,
    "policy_fault": 7,
    "policy_deadline": 8,
    "fall": 9,
    "transport_fault": 10,
    "control_deadline": 11,
    "ack_timeout": 12,
}


class HybridCore:
    def __init__(self, policy, logic, config):
        self.policy, self.logic, self.config = policy, logic, config
        self.tx_seq = 0
        self.counters = {
            "telemetry": 0,
            "telemetry_received": 0,
            "rejected": 0,
            "inferences": 0,
            "deadline_misses": 0,
        }
        # What the telemetry sequence says about the UART, independent of the OS: frames the
        # MCU stamped but LF never saw show up as gaps in its 5 ms clock. Cumulative.
        self.link_quality = {"gaps": 0, "lost_frames": 0, "max_gap_us": 0, "max_burst": 0}
        self.new_epoch()

    def new_epoch(self):
        self.policy.reset()
        self.logic.reset()
        self.epoch = secrets.randbits(32) or 1
        self.mode, self.fault, self.reason = "WAIT_LINK", 0, ""
        self.link = False
        self.rx_seq = None
        self.last_stamp = self.last_rx = self.last_t = self.start_stamp = None
        self.last_infer_stamp = None
        self.next_infer_stamp = None
        self.last_ack_rx = self.last_ack = None
        self.first_target_seq = None
        self.sent_targets = deque(maxlen=32)
        self.sent_target_at = {}
        self.ack_latency_ms = deque(maxlen=32)
        self.folded, self.phase = False, "Idle"
        self.action, self.velocity = [0.0] * 6, [0.0, 0.0]
        self.target = [0.0] * 6
        self.handoff_us = None
        self.inference_max_ms = 0.0
        self.consecutive_misses = 0
        self.offset_us = None
        self.mcu_diagnostics = None
        self.mcu_diagnostics_rx = None
        self.mcu_diagnostics_offset_us = None
        self.last_telemetry_flags = None
        self.last_command_error = None
        self.encoder_source = None
        self.encoder_reset = {"state":"idle", "message":"Chưa chụp HOME trong phiên này"}
        self.imu_calibration = {"state":"idle", "message":"Chưa yêu cầu calib IMU trong phiên này"}

    def frame(self, kind, payload):
        seq = self.tx_seq
        self.tx_seq = (self.tx_seq + 1) & 0xFFFFFFFF
        return p.Frame(kind, seq, time.monotonic_ns() // 1000, payload)

    def hello(self):
        return self.frame(p.HELLO, p.HELLO_DATA.pack(self.epoch, 200, 100, p.SCHEMA))

    def status(self):
        return self.frame(
            p.STATUS,
            p.STATUS_DATA.pack(
                self.epoch, MODES[self.mode], self.fault, self.counters["rejected"]
            ),
        )

    def output(self, seq, profile=0, flags=0, target=None, ff=None, sent_at=None):
        """Send 4 absolute hip/crank angles + 2 wheel angular rates.

        ff is hipL/hipR/kneeL/kneeR angular velocity feedforward for recovery.
        Neither target nor ff contains a torque command.
        """
        frame = self.frame(
            p.TARGET,
            p.TARGET_DATA.pack(
                self.epoch,
                seq,
                profile,
                flags,
                *((target or [0.0] * 6) + (ff or [0.0] * 4)),
            ),
        )
        if profile and self.first_target_seq is None:
            self.first_target_seq = frame.seq
        if profile:
            if len(self.sent_targets) == self.sent_targets.maxlen:
                self.sent_target_at.pop(self.sent_targets[0], None)
            self.sent_targets.append(frame.seq)
            self.sent_target_at[frame.seq] = time.monotonic() if sent_at is None else sent_at
        return frame

    def fail(self, name, detail=""):
        if name != "invalid_telemetry" and self.encoder_reset["state"] == "pending":
            self.encoder_reset = {"state":"error", "message":"Reset encoder bị hủy vì runtime có lỗi; không tự gửi lại."}
        # A driver fault/E-stop takes precedence over missing startup
        # calibration: that startup fault permits a disarmed HOME capture.
        if self.mode != "FAULT" or name == "driver_fault":
            self.fault, self.reason = FAULTS[name], detail or name
        self.mode = "FAULT"
        return self.output(self.rx_seq or 0)

    def _target_ack_window_ms(self):
        """Estimate a bounded ACK window from recent applied latency."""
        floor_ms = self.config.get("target_ack_timeout_min_ms", 30)
        ceiling_ms = self.config["target_ack_timeout_ms"]
        margin_ms = self.config.get("target_ack_latency_margin_ms", 8)
        samples = sorted(self.ack_latency_ms)
        if not samples:
            return floor_ms
        if len(samples) < 8:
            observed_ms = samples[-1]
        else:
            observed_ms = samples[math.ceil(0.9 * len(samples)) - 1]
        return max(floor_ms, min(ceiling_ms, observed_ms + margin_ms))

    def _target_ack_timeout_detail(self, applied_seq, now):
        """Track applied progress and fault only after the adaptive ACK window.

        MCU TARGET TTL and telemetry freshness remain independent hard limits.
        """
        if self.first_target_seq is None:
            return None

        if applied_seq in self.sent_targets and (
            self.last_ack is None or p.newer(applied_seq, self.last_ack)
        ):
            sent_at = self.sent_target_at.get(applied_seq)
            if sent_at is not None and now >= sent_at:
                self.ack_latency_ms.append(1000 * (now - sent_at))
            self.last_ack, self.last_ack_rx = applied_seq, now

        ack_age = now - self.last_ack_rx if self.last_ack_rx is not None else float("inf")
        timeout_ms = self._target_ack_window_ms()
        if ack_age <= timeout_ms / 1000:
            return None

        newest_sent = self.sent_targets[-1] if self.sent_targets else None
        return (
            f"no applied ACK for {1000*ack_age:.1f} ms "
            f"(adaptive window {timeout_ms:.1f} ms); "
            f"telemetry applied_seq={applied_seq}, last_ack_seq={self.last_ack}, "
            f"newest_sent_seq={newest_sent}"
        )

    def watchdog(self, now):
        if self.encoder_reset["state"] == "pending" and now - self.encoder_reset["sent_at"] > 1.0:
            self.encoder_reset = {"state":"error", "message":"ESP chưa xác nhận Reset encoder; không tự gửi lại. Kiểm tra firmware/link."}
        if self.imu_calibration["state"] == "pending" and now - self.imu_calibration["sent_at"] > 1.0:
            self.imu_calibration = {"state":"error", "message":"ESP chưa xác nhận yêu cầu calib IMU; không tự gửi lại."}
        if self.imu_calibration["state"] == "running" and now - self.imu_calibration["started_at"] > 5.0:
            self.imu_calibration = {"state":"error", "message":"IMU chưa hợp lệ sau 5 s; giữ robot đứng yên rồi thử calib lại."}
        if self.mode in ("HOME", "STANDUP", "WARMUP", "MODEL"):
            if (
                self.last_rx is None
                or now - self.last_rx > self.config["telemetry_timeout_ms"] / 1000
            ):
                return [self.fail("telemetry_timeout"), self.status()]
        return []

    def receive(self, frame, now):
        # MCU can reboot while LF retains its old sequence/timestamp baseline.
        if frame.kind == p.HELLO:
            if frame.payload == p.HELLO_DATA.pack(0, 200, 100, p.SCHEMA):
                disabled = self.output(frame.seq)
                self.new_epoch()
                return [disabled, self.hello(), self.status()]
            self.counters["rejected"] += 1
            return []
        if frame.kind not in (p.HELLO_ACK, p.TELEMETRY, p.ENCODER_HOME_ACK, p.CAL_IMU_ACK, p.EXECUTION_DIAG):
            self.counters["rejected"] += 1
            return []
        if (
            len(frame.payload) < 4
            or struct.unpack_from("<I", frame.payload)[0] != self.epoch
        ):
            self.counters["rejected"] += 1
            return []
        if not p.newer(frame.seq, self.rx_seq):
            self.counters["rejected"] += 1
            return []
        if frame.kind == p.EXECUTION_DIAG:
            # Optional measurements share sequence ordering, but cannot renew
            # telemetry/ACK watchdogs, seed encoder source, or emit TARGET.
            try:
                data = p.decode_execution_diagnostics(frame.payload)
            except (ValueError, struct.error):
                self.counters["rejected"] += 1
                return []
            if not self.link:
                self.counters["rejected"] += 1
                return []
            offset = now * 1e6 - frame.stamp
            baselines = [offset]
            if self.mcu_diagnostics_offset_us is not None:
                baselines.append(self.mcu_diagnostics_offset_us)
            if self.offset_us is not None:
                baselines.append(self.offset_us)
            self.mcu_diagnostics_offset_us = min(baselines)
            data["transport_age_s"] = max(0.0,
                (offset - self.mcu_diagnostics_offset_us) / 1e6)
            self.mcu_diagnostics, self.mcu_diagnostics_rx = data, now
            self.rx_seq = frame.seq
            return []
        self.rx_seq = frame.seq
        try:
            if frame.kind == p.ENCODER_HOME_ACK:
                _, request_seq, result = p.ENCODER_HOME_ACK_DATA.unpack(frame.payload)
                if self.encoder_reset["state"] != "pending" or request_seq != self.encoder_reset["request_seq"]:
                    self.counters["rejected"] += 1
                    return []
                if result == p.HOME_APPLIED:
                    # This button explicitly resets the software reference and LF
                    # session. MCU local fault latch/HOME remain untouched by epoch.
                    imu_calibration = self.imu_calibration.copy()
                    disabled = self.output(self.rx_seq or 0)
                    self.new_epoch()
                    self.encoder_reset = {"state":"done", "message":"ESP đã chụp HOME. Giữ Disable; ARM là thao tác riêng."}
                    self.imu_calibration = imu_calibration
                    return [disabled, self.hello(), self.status()]
                messages = {p.HOME_NOT_OFF:"ESP chưa xác nhận đủ 6 motor OFF; bấm Disable và đợi.",
                            p.HOME_FEEDBACK:"Feedback encoder cũ hoặc thiếu; chưa chụp HOME.",
                            p.HOME_REFUSED:"ESP từ chối: encoder còn chuyển động, bus/lỗi hoặc chưa OFF."}
                self.encoder_reset = {"state":"error", "message":messages.get(result,"Kết quả Reset encoder không hợp lệ")}
                return []
            if frame.kind == p.CAL_IMU_ACK:
                _, request_seq, result = p.CAL_IMU_ACK_DATA.unpack(frame.payload)
                if (self.imu_calibration["state"] != "pending" or
                    request_seq != self.imu_calibration["request_seq"]):
                    self.counters["rejected"] += 1
                    return []
                if result == p.CAL_IMU_ACCEPTED:
                    self.imu_calibration = {"state":"running", "started_at":now,
                                            "message":"ESP đang lấy mẫu IMU; giữ robot đứng yên."}
                else:
                    messages = {
                        p.CAL_IMU_NOT_OFF:"Calib IMU bị từ chối: chưa xác nhận motor OFF.",
                        p.CAL_IMU_FEEDBACK:"Calib IMU bị từ chối: telemetry đã cũ.",
                        p.CAL_IMU_BUSY:"ESP đang xử lý một yêu cầu calib khác.",
                        p.CAL_IMU_REFUSED:"ESP từ chối calib IMU ở trạng thái hiện tại.",
                    }
                    self.imu_calibration = {"state":"error",
                                            "message":messages.get(result,"Kết quả calib IMU không hợp lệ.")}
                return []
            if frame.kind == p.HELLO_ACK:
                if p.HELLO_DATA.unpack(frame.payload) != (
                    self.epoch,
                    200,
                    100,
                    p.SCHEMA,
                ):
                    raise ValueError("interface mismatch")
                offset = now * 1e6 - frame.stamp
                self.mcu_diagnostics_offset_us = (offset if self.mcu_diagnostics_offset_us is None else
                    min(offset, self.mcu_diagnostics_offset_us))
                if not self.link:
                    self.link, self.mode = True, "DISARMED"
                return [self.status()]
            if not self.link:
                self.counters["rejected"] += 1
                return [self.hello()]
            self.counters["telemetry_received"] += 1
            return self.telemetry(frame, now)
        except (ValueError, struct.error, OverflowError) as error:
            return [self.fail("invalid_telemetry", str(error)), self.status()]

    def command(self, op, now, vx=0.0, wz=0.0):
        """Handle operator intent received locally on LF, outside HCR1 UART."""
        self.last_command_error = None
        if not math.isfinite(vx) or not math.isfinite(wz):
            raise ValueError("nonfinite command")
        if op not in ("ARM", "DISARM", "RESET", "VELOCITY", "RESET_ENCODERS", "CAL_IMU"):
            self.counters["rejected"] += 1
            return [self.status()]
        if op == "RESET_ENCODERS":
            if (not self.link or self.mode not in ("DISARMED", "FAULT") or
                (self.mode == "FAULT" and self.fault != FAULTS["invalid_telemetry"]) or
                self.encoder_reset["state"] == "pending" or
                self.imu_calibration["state"] in ("pending", "running") or self.encoder_source is None or
                not 0 <= now-self.encoder_source[1] <= .03 or
                not (self.last_telemetry_flags or 0) & p.ENCODER_OK or
                (self.last_telemetry_flags or 0) & (p.ESTOP | p.DRIVER_FAULT)):
                self.counters["rejected"] += 1
                self.last_command_error = "Reset encoder: Disable trước, cần feedback encoder mới và không có lỗi driver/E-stop."
                return [self.status()]
            request = self.frame(p.ENCODER_HOME, p.ENCODER_HOME_DATA.pack(self.epoch,self.encoder_source[0]))
            self.encoder_reset = {"state":"pending", "request_seq":request.seq, "sent_at":now,
                                  "message":"Đang chờ ESP xác nhận chụp HOME; không tự bật motor."}
            return [request]
        if op == "CAL_IMU":
            flags = self.last_telemetry_flags or 0
            if (not self.link or self.mode not in ("DISARMED", "FAULT") or
                (self.mode == "FAULT" and self.fault != FAULTS["invalid_telemetry"]) or
                self.encoder_reset["state"] == "pending" or
                self.imu_calibration["state"] in ("pending", "running") or
                self.encoder_source is None or not 0 <= now-self.encoder_source[1] <= .03 or
                not flags & p.ENCODER_OK or flags & (p.ESTOP | p.DRIVER_FAULT)):
                self.counters["rejected"] += 1
                self.last_command_error = "Calib IMU: cần link/telemetry mới, Disable, encoder hợp lệ và không có E-stop/lỗi driver."
                return [self.status()]
            request = self.frame(p.CAL_IMU, p.CAL_IMU_DATA.pack(self.epoch,self.encoder_source[0]))
            self.imu_calibration = {"state":"pending", "request_seq":request.seq,
                                    "sent_at":now, "message":"Đang chờ ESP nhận yêu cầu calib IMU."}
            return [request]
        if op == "RESET":
            disabled = self.output(self.rx_seq or 0)
            self.new_epoch()
            return [disabled, self.hello(), self.status()]
        if op == "DISARM":
            # Faults remain latched until an explicit RESET.
            if self.mode != "FAULT":
                self.mode = "DISARMED"
                self.policy.reset()
                self.logic.reset()
            self.velocity = [0.0, 0.0]
            return [self.output(self.rx_seq or 0), self.status()]
        if op == "ARM" and (self.encoder_reset["state"] == "pending" or
                             self.imu_calibration["state"] in ("pending", "running")):
            self.counters["rejected"] += 1
            self.last_command_error = "ARM: đang chờ ESP hoàn tất calib IMU hoặc chụp HOME"
            return [self.status()]
        if op == "ARM" and self.mode == "DISARMED":
            if (
                self.last_rx is None
                or now - self.last_rx > self.config["telemetry_timeout_ms"] / 1000
            ):
                self.counters["rejected"] += 1
                self.last_command_error = (
                    f"ARM: chưa có telemetry hợp lệ đủ mới (giới hạn {self.config['telemetry_timeout_ms']:g} ms)"
                )
                return [self.status()]
            if any(
                abs(q - h) > self.config["home_tolerance_rad"]
                for q, h in zip(self.last_t.q[:4], self.config["home_joints"])
            ):
                self.counters["rejected"] += 1
                self.last_command_error = (
                    "ARM: góc khớp lệch HOME quá giới hạn "
                    + f"{self.config['home_tolerance_rad']:.2f}".replace(".", ",") + " rad"
                )
                return [self.status()]
            self.policy.reset()
            self.start_stamp = None
            self.last_infer_stamp = None
            self.next_infer_stamp = None
            self.first_target_seq = self.last_ack = self.last_ack_rx = None
            self.sent_targets.clear()
            self.sent_target_at.clear()
            self.ack_latency_ms.clear()
            self.action, self.velocity = [0.0] * 6, [0.0, 0.0]
            self.folded, self.phase, self.handoff_us = False, "Idle", None
            self.mode = "HOME"
            return [self.status()]
        if op == "VELOCITY" and self.mode in ("HOME", "STANDUP", "WARMUP", "MODEL"):
            if abs(vx) > 2 or abs(wz) > math.pi:
                raise ValueError("command outside trained range")
            self.velocity = [vx, wz]
            return [self.status()]
        self.counters["rejected"] += 1
        self.last_command_error = f"{op}: trạng thái {self.mode}, lỗi {self.fault}: {self.reason or 'lệnh không phù hợp trạng thái hiện tại'}"
        return [self.status()]

    def estop(self):
        return [self.fail("driver_fault", "operator E-stop"), self.status()]

    def telemetry(self, frame, now):
        t = p.Telemetry.decode(frame.payload)
        self.last_telemetry_flags = t.flags
        self.encoder_source = (frame.seq, now)
        if self.imu_calibration["state"] == "running" and t.flags & p.IMU_OK:
            self.imu_calibration = {"state":"done", "message":"IMU đã hiệu chuẩn và đang hợp lệ."}
        if t.flags & (p.ESTOP | p.DRIVER_FAULT):
            return [self.fail("driver_fault"), self.status()]
        if t.flags & 7 != 7:
            raise ValueError("IMU/encoder/calibration validity missing")
        norm = math.sqrt(sum(x * x for x in t.gravity))
        if not 0.9 <= norm <= 1.1:
            raise ValueError("gravity must be a body-frame unit vector")
        if self.last_stamp is not None:
            dt = frame.stamp - self.last_stamp
            if dt <= 0 or dt > self.config["telemetry_timeout_ms"] * 1000:
                return [self.fail("clock_gap"), self.status()]
        # Clock offset estimated without assuming synchronized MCU/Linux clocks.
        offset = now * 1e6 - frame.stamp
        self.offset_us = (
            offset if self.offset_us is None else min(offset, self.offset_us)
        )
        if offset - self.offset_us > self.config["telemetry_timeout_ms"] * 1000:
            return [
                self.fail("clock_gap", "queued telemetry exceeds age limit"),
                self.status(),
            ]
        if self.last_stamp is not None:
            gap = frame.stamp - self.last_stamp
            lost = (gap + TELEMETRY_PERIOD_US // 2) // TELEMETRY_PERIOD_US - 1
            quality = self.link_quality
            quality["max_gap_us"] = max(quality["max_gap_us"], gap)
            if lost > 0:
                quality["gaps"] += 1
                quality["lost_frames"] += lost
                quality["max_burst"] = max(quality["max_burst"], lost)
        self.last_stamp, self.last_rx, self.last_t = frame.stamp, now, t
        self.counters["telemetry"] += 1
        if self.mode in ("DISARMED", "FAULT", "WAIT_LINK"):
            return [self.output(frame.seq)]
        if self.start_stamp is None:
            self.start_stamp = frame.stamp
            self.next_infer_stamp = frame.stamp + round(
                self.config["warmup_start_s"] * 1e6
            )
            self.last_ack_rx = now
        elapsed = (frame.stamp - self.start_stamp) / 1e6
        if self.first_target_seq is not None:
            detail = self._target_ack_timeout_detail(t.applied_seq, now)
            if detail is not None:
                return [self.fail("ack_timeout", detail), self.status()]
        if elapsed >= self.config["warmup_start_s"]:
            if not t.flags & p.TARGET_VALID:
                return [self.fail("missing_applied_target"), self.status()]
        # A fall latches a fault; the robot is never re-armed automatically.
        if self.mode == "MODEL" and -t.gravity[2] / norm < math.cos(math.radians(70)):
            return [self.fail("fall"), self.status()]
        due = frame.stamp >= self.next_infer_stamp
        if elapsed >= self.config["warmup_start_s"] and due:
            if self.mode != "MODEL":
                self.action = encode_targets(t.applied_target)
            handoff_ready = (
                elapsed >= self.config["handoff_earliest_s"]
                and self.folded
                and self.policy.steps >= self.config["warmup_steps"]
                and self.mode != "MODEL"
            )
            try:
                prediction = self.policy.step(
                    observation(t, self.action, self.velocity)
                )
            except Exception as error:
                return [self.fail("policy_fault", str(error)), self.status()]
            self.counters["inferences"] += 1
            self.inference_max_ms = max(self.inference_max_ms, self.policy.last_ms)
            self.last_infer_stamp = frame.stamp
            # Keep an absolute 100 Hz schedule despite telemetry timestamp jitter.
            self.next_infer_stamp += 10000
            while self.next_infer_stamp <= frame.stamp:
                self.next_infer_stamp += 10000
            if self.policy.last_ms > self.config["inference_deadline_ms"]:
                self.counters["deadline_misses"] += 1
                self.consecutive_misses += 1
                if (
                    self.policy.last_ms > self.config["inference_hard_deadline_ms"]
                    or self.consecutive_misses
                    >= self.config["max_consecutive_deadline_misses"]
                ):
                    native_ms = getattr(
                        getattr(self.policy, "model", None), "last_native_ms", None
                    )
                    return [
                        self.fail(
                            "policy_deadline",
                            f"inference {self.policy.last_ms:.3f} ms; native {native_ms} ms",
                        ),
                        self.status(),
                    ]
                # Discard a late prediction, retain the last actually commanded action.
                # GRU has observed this sample; do not roll its memory backwards.
            else:
                self.consecutive_misses = 0
                if handoff_ready:
                    self.mode, self.handoff_us = "MODEL", frame.stamp
                if self.mode == "MODEL":
                    self.action = prediction
                    self.target = decode_actions(prediction)
        if elapsed < self.config["home_hold_s"]:
            self.mode = "HOME"
            h = self.config["home_joints"]
            self.target = [h[0], h[1], h[0] + h[2], h[1] + h[3], 0.0, 0.0]
            return [self.output(frame.seq, p.HOME_PD, 3, self.target, sent_at=now)]
        if self.mode == "MODEL":
            return [self.output(frame.seq, p.POLICY_PD, 3, self.target, sent_at=now)]
        try:
            self.folded, phase, flags, self.target, ff = self.logic.step(elapsed, t)
            self.phase = PHASES[phase] if 0 <= phase < len(PHASES) else "Unknown"
        except Exception as error:
            return [self.fail("logic_fault", str(error)), self.status()]
        self.mode = "WARMUP" if elapsed >= self.config["warmup_start_s"] else "STANDUP"
        return [self.output(frame.seq, p.RECOVERY, flags, self.target, ff, sent_at=now)]

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        diagnostics = None
        if self.mcu_diagnostics is not None:
            diagnostics = dict(self.mcu_diagnostics)
            diagnostics["age_s"] = max(0.0, now - self.mcu_diagnostics_rx)
        return {
            "mcu_diagnostics": diagnostics,
            "ack_diagnostics": {
                "window_ms": self._target_ack_window_ms(),
                "age_ms": None if self.last_ack_rx is None else max(0.0, 1000 * (now - self.last_ack_rx)),
                "latency_samples": len(self.ack_latency_ms),
                "latency_ms_recent_max": max(self.ack_latency_ms, default=0.0),
            },
            "encoder_reset": self.encoder_reset.copy(),
            "imu_calibration": self.imu_calibration.copy(),
            "epoch": self.epoch,
            "mode": self.mode,
            "phase": self.phase,
            "fault_code": self.fault,
            "reason": self.reason,
            "policy_steps": self.policy.steps,
            "handoff_us": self.handoff_us,
            "inference_max_ms": self.inference_max_ms,
            "consecutive_deadline_misses": self.consecutive_misses,
            "counters": self.counters,
            "link": self.link,
            "link_quality": dict(self.link_quality),
            "last_telemetry_flags": self.last_telemetry_flags,
        }
