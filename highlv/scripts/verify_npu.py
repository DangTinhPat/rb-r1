#!/usr/bin/env python3
"""Compare RKNN against recurrent ONNX reference on LF with runtime stopped."""

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
        rows = json.loads((ROOT / "tests/data/policy_reference.json").read_text())
        action_errors, hidden_errors, times = [], [], []
        gc.collect()
        gc.disable()
        try:
            for row in rows:
                release = time.monotonic()
                action = model.step(row["obs"])
                times.append(model.last_ms)
                action_errors.extend(abs(x - y) for x, y in zip(action, row["action"]))
                hidden_errors.extend(
                    abs(x - y) for x, y in zip(model.hidden, row["hidden"])
                )
                time.sleep(max(0.0, 0.01 - (time.monotonic() - release)))
        finally:
            model.close()
        report = {
            "scheduler": "SCHED_FIFO 20",
            "steps": len(rows),
            "action_max_abs": max(action_errors),
            "action_mean_abs": statistics.mean(action_errors),
            "hidden_max_abs": max(hidden_errors),
            "inference_median_ms": statistics.median(times),
            "inference_max_ms": max(times),
            "deadline_misses": sum(t > 10 for t in times),
            "worst_step": times.index(max(times)),
            "startup_ms": model.startup_ms,
        }
        report["passed"] = max(action_errors) < 0.05 and report["deadline_misses"] == 0
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
