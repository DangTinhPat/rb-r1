// Fold-down planner of the LF runtime, in C++.
//
// From the HOME pose both hips ramp at a fixed rate until the thighs rest on the floor, then
// hold; standing up from the folded pose belongs to the policy. The ramp is a function of
// elapsed MCU time, so a lost telemetry frame cannot make the reference lag. Nothing is fed
// back: the only guard is the body pitch (|pitch| > 0.80 rad).
//
// Freestanding on purpose, like tensor_math.c: no libc, libm, STL or exceptions, so the shared
// object loads into the LF's own Python whatever libc the cross compiler used. The interface
// is plain C (extern "C") for ctypes; see rb_runtime/fold.py.
#include <stddef.h>
#include <stdint.h>

namespace {
inline bool finite64(double v) { return __builtin_isfinite(v); }
inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

extern "C" {

struct FoldState {
  uint32_t started;
  double origin_s, hip_start, previous_s, previous_hip, ff;
};

}  // extern "C"

namespace {
constexpr double kRateRadS = 2.4;       // hip fold speed
constexpr double kTravelRad = 3.05;     // hip travel from the HOME stop to the floor
constexpr double kTanSquared = 1.060155558164756;  // tan(0.80 rad)^2: pitch limit without libm
constexpr double kFfTauS = 0.025;       // first-order filter on the hip rate feed-forward
constexpr double kFfLimitRadS = 6.0;
constexpr double kFlags = 3.0;          // legs and wheels enabled; wheel speed stays 0, as in HOME
}  // namespace

extern "C" {

size_t hcr_fold_state_size() { return sizeof(FoldState); }

void hcr_fold_reset(void *state) {
  FoldState *s = static_cast<FoldState *>(state);
  s->started = 0;
  s->origin_s = s->hip_start = s->previous_s = s->previous_hip = s->ff = 0.0;
}

// out[0..5] target (hipL, hipR, crankL, crankR, wheelL, wheelR), out[6..9] feed-forward
// (hipL, hipR, kneeL, kneeR), out[10] flags, out[11] phase (1 Folding, 2 Folded).
// Returns 0 while folding, 1 once folded, -1 for non-finite input, -2 if |pitch| > 0.80 rad
// (|g0| > tan(0.80) * hypot(g1, g2), compared squared so no sqrt or atan2 is needed).
int hcr_fold_step(void *state, double elapsed_s, double hip_left, double hip_right, double g0, double g1,
                  double g2, double *out) {
  FoldState *s = static_cast<FoldState *>(state);
  if (!finite64(elapsed_s) || !finite64(hip_left) || !finite64(hip_right) || !finite64(g0) || !finite64(g1) ||
      !finite64(g2))
    return -1;
  if (g0 * g0 > kTanSquared * (g1 * g1 + g2 * g2)) return -2;
  if (!s->started) {
    s->started = 1;
    s->origin_s = s->previous_s = elapsed_s;
    s->hip_start = s->previous_hip = 0.5 * (hip_left + hip_right);
  }
  const double travel = clampd(kRateRadS * (elapsed_s - s->origin_s), 0.0, kTravelRad);
  const double hip = s->hip_start - travel;
  const double dt = elapsed_s - s->previous_s;
  if (dt > 0.0) {
    const double rate = clampd((hip - s->previous_hip) / dt, -kFfLimitRadS, kFfLimitRadS);
    const double gain = dt / kFfTauS;
    s->ff += (gain < 1.0 ? gain : 1.0) * (rate - s->ff);
    s->previous_s = elapsed_s;
    s->previous_hip = hip;
  }
  const bool folded = travel >= kTravelRad;
  out[0] = out[1] = out[2] = out[3] = hip;  // crank = hip + knee, and the knee stays at 0
  out[4] = out[5] = 0.0;
  out[6] = out[7] = s->ff;
  out[8] = out[9] = 0.0;
  out[10] = kFlags;
  out[11] = folded ? 2.0 : 1.0;
  return folded ? 1 : 0;
}

}  // extern "C"
