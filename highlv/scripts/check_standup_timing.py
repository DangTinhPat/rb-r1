#!/usr/bin/env python3
"""Measure the logic helper with synthetic inputs; never opens UART or motors."""

import argparse
import fcntl
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from rb_runtime.recovery import StandupProcess
from rb_runtime.paths import native_directory


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--executable", default=str(native_directory() / "bin/standup_helper")
    )
    ap.add_argument("--max-ms", type=float, default=5.0)
    ap.add_argument("--status-file", default="/tmp/rb-hybrid-status.json")
    ap.add_argument("--report", type=Path)
    args = ap.parse_args()
    # A second helper also performs boot-time LQR solves. Do not compete with
    # the live controller on a single-core LF, even if this tool has no UART.
    runtime_lock = open(args.status_file + ".lock", "a")
    try:
        fcntl.flock(runtime_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        runtime_lock.close()
        ap.error("stop the LF runtime before running the isolated timing probe")
    sample = SimpleNamespace(
        gravity=[0.0, 0.0, -1.0],
        gyro=[0.0] * 3,
        accel=[0.0, 0.0, 9.81],
        q=[2.35706715, 2.35706715, 0.0, 0.0, 0.0, 0.0],
        qd=[0.0] * 6,
        hip_torque=[0.0, 0.0],
        flags=7,
    )
    start = time.monotonic()
    logic = StandupProcess(args.executable)
    result = {"boot_ms": (time.monotonic() - start) * 1000, "runs": []}
    reference = None
    try:
        for run in range(6):
            start = time.monotonic()
            logic.reset()
            reset_ms = (time.monotonic() - start) * 1000
            times, outputs = [], []
            for tick in range(100):
                start = time.monotonic()
                outputs.append(logic.step(3.0 + tick * 0.005, sample))
                times.append((time.monotonic() - start) * 1000)
                time.sleep(0.005)
            if reference is None:
                reference = outputs
            result["runs"].append(
                dict(
                    run=run,
                    reset_ms=reset_ms,
                    first_ms=times[0],
                    max_ms=max(times),
                    reset_equivalent=outputs == reference,
                )
            )
    finally:
        logic.close()
        runtime_lock.close()
    result["pass"] = all(
        r["max_ms"] < args.max_ms
        and r["reset_ms"] < args.max_ms
        and r["reset_equivalent"]
        for r in result["runs"]
    )
    text = json.dumps(result, indent=2) + "\n"
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(text)
    print(text, end="")
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
