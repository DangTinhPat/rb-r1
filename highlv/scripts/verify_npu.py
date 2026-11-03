#!/usr/bin/env python3
"""Replay a recorded observation sequence through the RKNN policy on LF with the runtime stopped.

There is no numerical reference for the shipped model (no ONNX source), so this checks what can
be checked on the board: finite outputs, the 10 ms deadline and the inference time."""

import argparse
import fcntl
import gc
import json
import os
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from rb_runtime.policy import Policy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--status-file", default="/tmp/rb-hybrid-status.json")
    parser.add_argument(
        "--report", type=Path, default=ROOT / "reports/current/npu_reference.json"
    )
    args = parser.parse_args()
    # Hold the same lock as runtime so a diagnostic cannot compete with control.
    with open(args.status_file + ".lock", "a") as guard:
        try:
            fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("stop the LF runtime before running the NPU reference check")
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(20))
        model = Policy(ROOT)
        observations = json.loads((ROOT / "tests/data/policy_observations.json").read_text())["obs"]
        actions, times = [], []
        gc.collect()
        gc.disable()
        try:
            for obs in observations:
                release = time.monotonic()
                actions.append(model.step(obs))  # raises on a nonfinite action or hidden state
                times.append(model.last_ms)
                time.sleep(max(0.0, 0.01 - (time.monotonic() - release)))
        finally:
            model.close()
        flat = [abs(x) for action in actions for x in action]
        report = {
            "scheduler": "SCHED_FIFO 20",
            "steps": len(observations),
            "action_abs_max": max(flat),
            "inference_median_ms": statistics.median(times),
            "inference_max_ms": max(times),
            "deadline_misses": sum(t > 10 for t in times),
            "worst_step": times.index(max(times)),
            "startup_ms": model.startup_ms,
            "numerical_reference": False,
        }
        report["passed"] = report["deadline_misses"] == 0
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
