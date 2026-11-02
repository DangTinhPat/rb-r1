#include "rb/control/lqr_config.hpp"
#include <algorithm>
#include "rb/control/generated_geometry.hpp"
namespace rb::control {
LqrBalancer::Config tunedLqrConfig() {
  LqrBalancer::Config lqr;
  lqr.wheel_radius_m = rb::control::geometry::wheel_radius_m;
  // Half the distance between the two wheel contact centres (axle spacing 0.2402 m on RB v1.3).
  lqr.half_wheel_base_m = 0.5F * rb::control::geometry::axle_spacing_m;
  lqr.left_wheel_sign = 1.0F;
  // The speed actuator is torque-limited at 0.13 N.m. Keep the operator
  // envelope wider than the validated cruise range so the feedback loop can
  // spend speed to catch pitch without violating the physical cap.
  // Stay below the native Moving Speed frame envelope while leaving enough
  // translational authority to catch a pitch transient after the wheel has
  // already crossed the quiet station speed.
  lqr.max_ground_velocity_m_s = 1.50F;
  const float wheel_torque_accel_m_s2 =
      2.0F * 0.13F / (lqr.wheel_radius_m * rb::control::geometry::total_mass_kg);
  lqr.max_ground_accel_m_s2 = std::min(1.40F, wheel_torque_accel_m_s2);
  lqr.wheel_ground_mu = 0.8F;
  lqr.wheel_velocity_reference_from_measured = true;
  lqr.target_velocity_slew_rate_m_s2 = 0.30F;
  lqr.target_velocity_jerk_m_s3 = 1.0F;
  const float brake_capacity_m_s2 =
      std::min(wheel_torque_accel_m_s2, lqr.wheel_ground_mu * lqr.gravity_m_s2);
  lqr.stop_velocity_slew_rate_m_s2 = 0.65F * brake_capacity_m_s2;
  lqr.stop_velocity_jerk_m_s3 = 4.0F;
  lqr.target_yaw_slew_rate_rad_s2 = 1.0F;
  lqr.yaw_rate_feedback_kp = 1.0F;
  // Fail safe before the supported floor-side posture observed in the
  // native model (~0.69 rad).
  lqr.fall_pitch_rad = 0.55F;
  // Let the leg-side gimbal absorb the small supported pitch error. The
  // wheel LQR remains the fast catch controller for larger transients.
  lqr.wheel_pitch_feedback_scale = 0.70F;
  // Startup pendulum length: 1.077 x the COM height (0.1993 m) with the legs straight (knee 110
  // deg, hip -67.4 deg, axle 184.8 mm below the hip), from WholeBodyModel for RB v1.3.
  lqr.leg_length_m = 0.215F;
  lqr.pendulum_length_per_com_height = 1.077F;
  lqr.min_pendulum_length_m = 0.10F;
  lqr.max_pendulum_length_m = 0.27F;
  // LQR is solved for the fresh 500 Hz sensor interval, not the 3 kHz
  // actuator/cache loop. Gains are rebuilt only when the live COM height
  // changes enough to warrant a new reduced-order model.
  lqr.sampling_period_s = 0.002F;
  // Pitch capture is the primary safety objective.  A former MPC-shaped
  // velocity weight made the LQR brake odometry before it arrested pitch;
  // station return remains an adaptive outer target, not the fast state.
  lqr.position_cost_scale = 100.0F;
  lqr.pitch_cost_scale = 400.0F;
  lqr.velocity_cost_scale = 300.0F;
  lqr.pitch_rate_cost_scale = 40.0F;
  lqr.input_cost_scale = 10.0F;
  lqr.station_position_cost_scale = 200.0F;
  lqr.station_pitch_cost_scale = 400.0F;
  lqr.station_velocity_cost_scale = 300.0F;
  lqr.station_pitch_rate_cost_scale = 40.0F;
  lqr.station_input_cost_scale = 10.0F;
  lqr.station_gain_pitch_start_rad = 0.12F;
  lqr.station_gain_pitch_quiet_rad = 0.05F;
  lqr.station_gain_rate_start_rad_s = 0.60F;
  lqr.station_gain_rate_quiet_rad_s = 0.20F;
  lqr.station_gain_entry_dwell_s = 0.10F;
  lqr.station_gain_transition_s = 0.20F;
  lqr.station_pitch_return_gain_rad_per_m_s = 0.0F;
  lqr.station_pitch_return_max_rad = 0.0F;
  lqr.riccati_iterations = 80U;
  lqr.position_anchor_hold = true;
  lqr.station_velocity_decay_gain_hz = 0.0F;
  // The estimator's corrected rate remains exposed and calibrated, but the
  // current CAD plant profile keeps raw rate in the fast MPC until the
  // corrected-rate adoption trial clears all posture/stop envelopes.
  lqr.use_bias_corrected_pitch_rate = false;
  lqr.station_hold_gain_hz = 5.0F;
  lqr.station_hold_yaw_return = true;
  lqr.station_hold_max_v = 0.06F;
  lqr.station_posture_hold_max_v = 0.25F;
  // Keep the integrated station command inside the posture envelope. The
  // live capture schedule still decides when this ceiling is approached;
  // it is distinct from the wider internal MPC catch acceleration limit.
  lqr.station_hold_command_max_v = 0.10F;
  // The station command starts quiet, then expands with the measured capture
  // distance in LqrBalancer. Do not cap a real posture catch at the quiet
  // 0.25 m/s envelope; that leaves the wheel without authority exactly when
  // pitch/velocity evidence says it must brake.
  lqr.station_hold_dynamic_max_v = lqr.station_posture_hold_max_v;
  // Once pitch/rate have been quiet, station return may move the LQR speed
  // reference back toward the local anchor.  Keep that outer reference
  // slower than the wheel pitch-capture loop so the two objectives cannot
  // fight during the first transient.
  lqr.station_hold_command_slew_rate_m_s2 = 0.10F;
  // station_target is fed into the LQR's own velocity_target, so this term
  // does not act as an independent outer damper: it scales the velocity
  // state the DARE already weights. At 5.0 it made x2 = 6*v + 15*pos_err,
  // i.e. ~6x the designed velocity feedback and ~48x K0's position gain,
  // which sat on the 0.25 m/s station clamp and limit-cycled. Keep this
  // small and let position/velocity regulation come from the costs below,
  // where the Riccati solve co-designs it against the pitch gains.
  lqr.station_hold_velocity_damping_gain = 1.00F;
  // Fade the slow station return to 25% inside a 100 mm capture envelope.
  // The fast pitch-capture MPC remains untouched, so this removes quiet
  // encoder chatter without sacrificing recovery authority.
  lqr.station_adaptive_full_capture_distance_m = 0.10F;
  lqr.station_quiet_gain_scale = 0.25F;
  lqr.station_capture_gain = 0.0F;
  lqr.station_hold_velocity_servo_gain_hz = 0.0F;
  // Posture transitions must remain in place. The leg motion changes the
  // encoder-derived wheel odometry and COM geometry at the same time; if the
  // station anchor is released, the balance loop can turn that transient
  // into metres of unintended rolling. EmbeddedController re-anchors at the
  // actual endpoint, so this does not pull the robot back after a pose.
  lqr.hold_station_during_pose = true;
  return lqr;
}
} // namespace rb::control
