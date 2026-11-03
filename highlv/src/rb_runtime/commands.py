"""Operator commands travel over a local socket, separate from HCR1."""

import math


def parse_command(data):
    tokens = data.decode("ascii", "strict").strip().split()
    if not tokens:
        raise ValueError("empty command")
    op = tokens[0].upper()
    if op == "VELOCITY" and len(tokens) == 3:
        vx, wz = float(tokens[1]), float(tokens[2])
        if not math.isfinite(vx) or not math.isfinite(wz):
            raise ValueError("nonfinite velocity")
        return op, vx, wz
    if op in ("ARM", "DISARM", "RESET", "ESTOP", "RESET_ENCODERS") and len(tokens) == 1:
        return op, 0.0, 0.0
    raise ValueError("use ARM, DISARM, RESET, ESTOP, RESET_ENCODERS, or VELOCITY vx wz")
