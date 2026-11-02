"""Resolve assets relative to this source checkout, on host or Luckfox."""

import os
import platform
from pathlib import Path

PROJECT_ROOT = Path(
    os.environ.get("RB_RUNTIME_ROOT", str(Path(__file__).resolve().parents[2]))
).resolve()


def native_directory():
    target = "arm" if platform.machine().lower().startswith("arm") else "host"
    return PROJECT_ROOT / "build" / target
