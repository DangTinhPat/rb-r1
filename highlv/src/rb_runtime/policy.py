import hashlib
import json
import math
import time
from pathlib import Path

DEFAULT = [-0.7, -0.7, 0.9, 0.9]


def encode_targets(target):
    """Servo references -> raw previous action (hip/crank rad, wheel rad/s)."""
    return (
        [(x + 0.7) / 0.25 for x in target[:2]]
        + [(x - 0.2) / 0.25 for x in target[2:4]]
        + [x * 0.035 / 0.8 for x in target[4:]]
    )


def decode_actions(action):
    """Raw actor action -> [hipL,hipR,phiL,phiR,omegaL,omegaR], no torque.

    phi is the crank motor coordinate hip+knee, not the knee hinge angle.
    Keep the trained .035 m scale; this is not a hardware wheel-radius setting.
    """
    return (
        [-0.7 + 0.25 * x for x in action[:2]]
        + [0.2 + 0.25 * x for x in action[2:4]]
        + [0.8 / 0.035 * x for x in action[4:]]
    )


def observation(t, previous_action, velocity):
    return (
        t.gyro
        + t.gravity
        + [velocity[0], 0.0, velocity[1]]
        + [x - d for x, d in zip(t.q[:4], DEFAULT)]
        + t.qd
        + list(previous_action)
        + [0.0, 0.0, 0.16]
        + [0.0] * 4
    )


class Policy:
    def __init__(self, root, backend="rknn"):
        self.root = Path(root)
        manifest = json.loads((self.root / "models/manifest.json").read_text())
        for name, expected in manifest["sha256"].items():
            data = (self.root / name).read_bytes()
            if hashlib.sha256(data).hexdigest() != expected:
                raise ValueError("artifact hash mismatch: " + name)
        self.backend = backend
        if backend == "rknn":
            from .rknn_backend import Model

            self.model = Model(
                str(self.root / "models/beni512_dfp16.rknn"), emit_io=False
            )
            if self.model.hidden_dim != 512:
                raise ValueError("expected GRU512")
            n = json.loads(
                (self.root / "models/beni512_19500_normalizer.json").read_text()
            )
            self.mean, self.divisor = n["mean"], n["divisor"]
        elif backend == "onnx":
            import numpy as np
            import onnxruntime as ort

            self.np = np
            opts = ort.SessionOptions()
            opts.intra_op_num_threads = opts.inter_op_num_threads = 1
            self.model = ort.InferenceSession(
                str(self.root / "models/model_19500.onnx"),
                sess_options=opts,
                providers=["CPUExecutionProvider"],
            )
        else:
            raise ValueError("unsupported backend")
        self.reset()
        # Allocate/touch tensor and runtime paths before serving any motor command.
        warm = [0.0] * 32
        warm[5], warm[27] = -1.0, 0.16
        self.startup_ms = []
        for _ in range(8):
            self.step(warm)
            self.startup_ms.append(self.last_ms)
        self.reset()

    def reset(self):
        self.hidden = [0.0] * 512
        self.steps = 0
        self.last_ms = 0.0

    def step(self, obs):
        if len(obs) != 32 or not all(math.isfinite(v) for v in obs):
            raise ValueError("invalid observation")
        start = time.monotonic_ns()
        if self.backend == "rknn":
            x = [(v - m) / d for v, m, d in zip(obs, self.mean, self.divisor)]
            action, hidden, _ = self.model.step(x, self.hidden)
        else:
            np = self.np
            action, hidden = self.model.run(
                ["actions", "h_out"],
                {
                    "obs": np.asarray(obs, np.float32).reshape(1, 32),
                    "h_in": np.asarray(self.hidden, np.float32).reshape(1, 1, 512),
                },
            )
            action, hidden = action.reshape(-1).tolist(), hidden.reshape(-1).tolist()
        self.last_ms = (time.monotonic_ns() - start) / 1e6
        # Native RKNN unpack checks every hidden value without a Python 512D loop.
        valid = all(math.isfinite(v) for v in action)
        if self.backend == "onnx":
            valid = valid and all(math.isfinite(v) for v in hidden)
        if len(action) != 6 or len(hidden) != 512 or not valid:
            raise ValueError("invalid policy result")
        self.hidden = hidden
        self.steps += 1
        return action

    def close(self):
        if self.backend == "rknn":
            self.model.close()
