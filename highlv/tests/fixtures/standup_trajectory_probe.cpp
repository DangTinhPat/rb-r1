// Deterministic sensor inputs, not a motor or dynamics simulator.
// Override these two macros only when regenerating the upstream reference.
#ifndef STANDUP_HEADER
#define STANDUP_HEADER "rb/recovery/standup.hpp"
#define STANDUP_NAMESPACE rb::recovery
#endif
#include STANDUP_HEADER
#include <cstdio>

using StandUp = STANDUP_NAMESPACE::StandUp;

static void trace(const char *name, StandUp::Config cfg, int ticks, bool stalled) {
  cfg.enabled = true;
  StandUp su(cfg);
  su.begin(0.F, 0.F);
  StandUp::Input in{};
  in.dt_s = .005F;
  in.body_clearance_valid = in.floor_bounds_valid = in.floor_bounds_feasible = true;
  in.body_clearance_m = in.floor_predicted_clearance_m = .1F;
  in.floor_hip_rate_lower_rad_s = -3.F;
  in.floor_hip_rate_upper_rad_s = 3.F;
  StandUp::Output out{};
  std::printf("\"%s\":[", name);
  bool first = true;
  for (int tick = 0; tick < ticks; ++tick) {
    const auto previous_phase = su.phase();
    const float old_hip = in.hip_measured_rad;
    in.hip_measured_rad = out.hip_target_rad;
    in.hip_measured_rate_rad_s = (in.hip_measured_rad - old_hip) / in.dt_s;
    in.knee_measured_rate_rad_s = stalled ? 0.F : .7F * out.knee_rate_rad_s;
    in.knee_measured_rad += in.knee_measured_rate_rad_s * in.dt_s;
    in.wheel_measured_rad_s = {out.wheel_rad_s, out.wheel_rad_s};
    // Close the rise gate while opening, then restore it. No pitch-limit trip.
    in.capture_lean_rad = ticks < 1000 && tick >= 220 && tick < 300 ? .5F : .02F;
    out = su.update(in);
    if (tick % 20 == 0 || out.phase != previous_phase || tick == ticks - 1) {
      std::printf("%s[%d,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g]", first ? "" : ",",
                  tick, int(out.phase), int(out.failure), double(out.hip_target_rad),
                  double(out.knee_target_rad), double(out.hip_feedback_rate_rad_s),
                  double(out.knee_rate_rad_s), double(out.wheel_rad_s), double(out.released));
      first = false;
    }
  }
  std::printf("]");
}

int main() {
  const auto rb = StandUp::tunedRecoveryRb();
  std::printf("{\"tuning\":[");
  bool first = true;
  for (float value : {rb.catch_wheel_lead_s, rb.catch_accel_gain, rb.catch_accel_damping,
                     rb.catch_hip_gain, rb.catch_hip_rate_max_rad_s, rb.catch_hip_accel_rad_s2,
                     rb.catch_wheel_max_rad_s, rb.catch_speed_gain, rb.catch_speed_integral_gain,
                     rb.catch_speed_integral_max_rad, rb.catch_speed_lean_max_rad,
                     rb.catch_speed_filter_s, rb.catch_hip_lean_deadband_rad,
                     rb.recovery_body_gain, rb.recovery_body_rate_max_rad_s,
                     rb.catch_wheel_delay_s, rb.catch_wheel_lag_s, rb.catch_rise_rate_rad_s,
                     rb.catch_rise_accel_rad_s2, rb.catch_rise_brake_rad_s2,
                     rb.rise_hip_ff, float(rb.rise_hip_ff_measured)}) {
    std::printf("%s%.9g", first ? "" : ",", double(value));
    first = false;
  }
  std::printf("],");
  trace("rb_sequence", rb, 1800, false);
  StandUp::Config rise;
  rise.fold_travel_rad = 0.F;
  rise.kneel_hold_s = .02F;
  rise.catch_settle_s = 100.F;  // Isolate the rise, without handing off.
  rise.catch_rise = true;
  rise.catch_rise_knee_rad = .9F;
  rise.catch_rise_start_dwell_s = .02F;
  rise.catch_rise_rate_rad_s = .8F;
  rise.catch_rise_accel_rad_s2 = .75F;
  rise.catch_rise_brake_rad_s2 = 4.F;
  rise.rise_hip_ff = .65F;
  rise.rise_hip_ff_measured = true;
  std::printf(",");
  trace("rise_brake_measured", rise, 600, false);
  std::printf(",");
  trace("stalled_knee", rise, 600, true);
  std::printf("}\n");
}
