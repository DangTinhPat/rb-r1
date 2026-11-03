import json
import math
import os
from collections import deque
import select
import struct
import subprocess
import time
from .protocol import TORQUE_VALID

REQUEST = struct.Struct("<24d")
REPLY = struct.Struct("<14d")
# Order of rb::recovery::StandUp::Failure in native/include/rb/recovery/standup.hpp.
FAILURES = ("None", "InvalidInput", "PitchLimit", "Timeout", "HipTracking", "FloorConstraint")


def fault_message(code, values):
    """Name the helper failure and keep the inputs it rejected, for post-mortem."""
    index = int(code) if math.isfinite(code) and 0 <= code < len(FAILURES) and code == int(code) else None
    name = FAILURES[index] if index is not None else "unknown"
    v = [round(x, 4) for x in values]
    return (
        f"stand-up fault {name} (code {code:g}) "
        f"t={v[0]} pitch={v[1]} gyroY={v[2]} gyroZ={v[3]} accel=[{v[4]},{v[5]}] "
        f"hip={v[6:8]} knee={v[8:10]} wheel_pos={v[10:12]} "
        f"hip_rate={v[12:14]} knee_rate={v[14:16]} wheel_rate={v[16:18]}"
    )


TRACE_FILE = "/tmp/rb-standup-trace.json"
TRACE_SAMPLES = 400  # 2 s at the 5 ms telemetry period
REQUEST_FIELDS = (
    "t", "pitch", "gyroY", "gyroZ", "accelX", "accelZ",
    "hipL", "hipR", "kneeL", "kneeR", "wheelPosL", "wheelPosR",
    "hipRateL", "hipRateR", "kneeRateL", "kneeRateR", "wheelRateL", "wheelRateR",
)
REPLY_FIELDS = (
    "support", "phase", "enables", "hipL_ref", "hipR_ref", "crankL_ref", "crankR_ref",
    "wheelL_cmd", "wheelR_cmd", "ffHipL", "ffHipR", "ffKneeL", "ffKneeR",
)


class StandupProcess:
    def __init__(self, executable, trace_file=TRACE_FILE):
        self.trace_file = trace_file
        self.trace = deque(maxlen=TRACE_SAMPLES)
        self.process = subprocess.Popen(
            [str(executable)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0
        )
        # Gain-table solves are boot work, before UART/ARM is served. Reset and
        # control exchanges retain the 20 ms helper timeout.
        try:
            self.exchange([0.0] * 23 + [1.0], timeout=2.0)
        except Exception:
            self.close()
            raise

    def exchange(self, values, timeout=0.02):
        raw = REQUEST.pack(*values)
        self.process.stdin.write(raw)
        response = bytearray()
        deadline = time.monotonic() + timeout
        while len(response) < REPLY.size:
            remaining = deadline - time.monotonic()
            if (
                remaining <= 0
                or not select.select([self.process.stdout], [], [], remaining)[0]
            ):
                raise RuntimeError("logic helper timeout")
            data = os.read(self.process.stdout.fileno(), REPLY.size - len(response))
            if not data:
                raise RuntimeError("logic helper exited")
            response.extend(data)
        result = REPLY.unpack(response)
        self.trace.append((tuple(values[:18]), result[:13]))
        if not all(math.isfinite(v) for v in result) or result[0] < 0:
            self.dump_trace(result[1])
            raise RuntimeError(fault_message(result[1], values))
        return result

    def dump_trace(self, code):
        """Keep the samples before a fault: what the core saw and what it commanded."""
        try:
            samples = [
                {"in": dict(zip(REQUEST_FIELDS, request)), "out": dict(zip(REPLY_FIELDS, reply))}
                for request, reply in self.trace
            ]
            with open(self.trace_file, "w") as stream:
                json.dump({"failure_code": code, "samples": samples}, stream)
        except Exception:
            pass  # post-mortem data must never change the fault path

    def reset(self):
        self.exchange([0.0] * 23 + [1.0])

    def step(self, elapsed, t):
        pitch = math.atan2(t.gravity[0], math.hypot(t.gravity[1], t.gravity[2]))
        s = [elapsed, pitch, t.gyro[1], t.gyro[2], t.accel[0], t.accel[2]]
        s += (
            t.q
            + t.qd
            + t.hip_torque
            + [1.0, 1.0, float(bool(t.flags & TORQUE_VALID)), 0.0]
        )
        r = self.exchange(s)
        return bool(r[0]), int(r[1]), int(r[2]), list(r[3:9]), list(r[9:13])

    def close(self):
        self.process.stdin.close()
        try:
            self.process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.process.stdout.close()
