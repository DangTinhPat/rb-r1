"""Fold-down sequence: the only stand-up step the LF still plans.

From the HOME pose both hips ramp at a fixed rate until the thighs rest on the floor, then
hold; standing up from the folded pose belongs to the policy. The planner is C++
(native/fold_planner.cpp, built as libfold_planner.so); this wraps it for HybridCore. The ramp
is a function of elapsed MCU time, so a lost telemetry frame cannot make the reference lag.
Nothing is fed back: the only guard is the body pitch (|pitch| > 0.80 rad).
"""

import ctypes as C
import math

from .paths import native_directory

PITCH_LIMIT_RAD = 0.80
FOLDING, FOLDED = 1, 2

_lib = None


def _library():
    global _lib
    if _lib is None:
        path = native_directory() / "lib/libfold_planner.so"
        try:
            lib = C.CDLL(str(path))
        except OSError as error:
            raise RuntimeError(f"cannot load {path}: run scripts/build.sh") from error
        lib.hcr_fold_state_size.restype = C.c_size_t
        lib.hcr_fold_reset.argtypes = [C.c_void_p]
        lib.hcr_fold_step.argtypes = [C.c_void_p] + [C.c_double] * 6 + [C.POINTER(C.c_double)]
        lib.hcr_fold_step.restype = C.c_int
        _lib = lib
    return _lib


class FoldLogic:
    def __init__(self):
        self._lib = _library()
        self._state = C.create_string_buffer(self._lib.hcr_fold_state_size())
        self._out = (C.c_double * 12)()
        self.reset()

    def reset(self):
        self._lib.hcr_fold_reset(self._state)

    def step(self, elapsed, t):
        """Return (folded, phase, flags, target[6], ff[4]); raise RuntimeError on a fault."""
        g = t.gravity
        code = self._lib.hcr_fold_step(self._state, elapsed, t.q[0], t.q[1], g[0], g[1], g[2], self._out)
        if code == -1:
            raise RuntimeError("fold fault InvalidInput: nonfinite elapsed, pitch or hip")
        if code == -2:
            pitch = math.atan2(g[0], math.hypot(g[1], g[2]))
            raise RuntimeError(f"fold fault PitchLimit: pitch={pitch:.4f} rad exceeds {PITCH_LIMIT_RAD}")
        v = self._out[:]
        return code == 1, int(v[11]), int(v[10]), v[0:6], v[6:10]
