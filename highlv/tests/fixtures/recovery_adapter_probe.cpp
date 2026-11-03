// Exercise the real adapter and controller with synthetic feedback only.
#define main recovery_pipe_main
#include "../../native/standup_main.cpp"
#undef main
#include <cassert>

int main() {
  StandupAdapter adapter;
  auto &cfg = adapter.controller.config().standup;
  cfg = StandUp::Config{};
  cfg.enabled = true;
  cfg.fold_travel_rad = 0.F;
  cfg.kneel_hold_s = .02F;
  cfg.catch_settle_s = 100.F;
  cfg.catch_rise = true;
  cfg.catch_rise_knee_rad = .9F;
  cfg.catch_rise_start_dwell_s = .02F;
  cfg.rise_hip_ff_measured = true;
  cfg.rise_hip_ff = .65F;
  std::array<double, 24> in{};
  std::array<double, 14> out{};
  const float offset = rb::recovery::LegHeight::hangHipCad();
  in[5] = 9.81;
  in[6] = in[7] = offset;
  in[8] = .5;  // One knee is ahead; the other has stalled at zero.
  in[14] = .8;
  in[15] = .2;
  in[20] = in[21] = 1;
  float max_knee = 0.F;
  unsigned immediate_wheel = 0;
  for (unsigned tick = 0; tick < 600; ++tick) {
    in[0] = 3. + tick * .005;
    adapter.step(in, out);
    assert(out[0] >= 0);
    const auto &su = adapter.controller.standupState();
    assert(su.failure == StandUp::Failure::None);
    assert(std::abs(out[5] - out[3] - su.knee_target_rad) < 1e-6);
    assert(std::abs(out[6] - out[4] - su.knee_target_rad) < 1e-6);
    if (su.phase == StandUp::Phase::Catching) {
      max_knee = std::max(max_knee, su.knee_target_rad);
      // No extra FIFO delay between controller output and the UART target.
      assert(std::abs(out[7] - su.wheel_rad_s) < 1e-6);
      assert(std::abs(out[8] - su.wheel_rad_s) < 1e-6);
      if (std::abs(su.wheel_rad_s) > .01F) ++immediate_wheel;
    }
    in[6] = out[3];
    in[7] = out[4];
    // Centre the measured CoM to let the rise proceed. This is test input,
    // not a plant model or a command to a physical joint.
    in[1] -= su.capture_lean_rad;
  }
  assert(max_knee > .05F && max_knee <= .15001F);
  assert(immediate_wheel > 0);
  // Synchronizing the nominal trajectory must not remove the existing floor fault.
  StandUp::Config safe;
  safe.fold_travel_rad = 0.F;
  safe.kneel_hold_s = 0.F;
  safe.catch_settle_s = 100.F;
  safe.floor_guard = true;
  StandUp guard(safe);
  guard.begin(0.F, 0.F);
  StandUp::Input feedback{};
  feedback.dt_s = .005F;
  feedback.body_clearance_valid = feedback.floor_bounds_valid = feedback.floor_bounds_feasible = true;
  feedback.body_clearance_m = feedback.floor_predicted_clearance_m = .1F;
  feedback.floor_hip_rate_lower_rad_s = -2.F;
  feedback.floor_hip_rate_upper_rad_s = 2.F;
  StandUp::Output guarded{};
  for (unsigned tick = 0; tick < 20; ++tick) {
    guarded = guard.update(feedback);
    feedback.hip_measured_rad = guarded.hip_target_rad;
  }
  assert(guarded.floor_armed);
  feedback.floor_predicted_clearance_m = -.004F;
  feedback.floor_bounds_feasible = false;
  feedback.floor_hip_rate_lower_rad_s = .6F;
  feedback.floor_hip_rate_upper_rad_s = .55F;
  guarded = guard.update(feedback);
  assert(guarded.phase == StandUp::Phase::Failed);
  assert(guarded.failure == StandUp::Failure::FloorConstraint);
  assert(!guarded.wheels_on);
  std::printf("PASS adapter: slower knee cap %.6f, immediate wheel samples %u, hip+ knee crank mapping\n",
              double(max_knee), immediate_wheel);
}
