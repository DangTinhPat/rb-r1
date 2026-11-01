#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "rb/control/generated_geometry.hpp"

namespace rb::control {

constexpr std::size_t kJointCount = 6;

enum class JointId : std::size_t {
  LeftHip = 0,
  LeftKnee = 1,
  LeftWheel = 2,
  RightHip = 3,
  RightKnee = 4,
  RightWheel = 5,
};

using JointArray = std::array<float, kJointCount>;

struct ImuState {
  float pitch_rad = 0.0F;
  // Elapsed time between fresh MPU6500 packets. Zero keeps legacy callers on
  // RobotState::dt_s; native multi-rate runners fill this with the actual
  // sensor period so CMF debounce is not stretched by a faster control loop.
  float sample_dt_s = 0.0F;
  float pitch_rate_rad_s = 0.0F;
  // Bias-corrected rate for the reduced-order balance model. Keep the raw
  // pitch_rate_rad_s channel for lifecycle/impact gates until those users are
  // explicitly migrated to an estimator-confidence contract.
  float pitch_rate_bias_corrected_rad_s = 0.0F;
  bool pitch_rate_bias_corrected_valid = false;
  // Estimated pitch-axis gyro bias. The native bring-up keeps the public
  // pitch-rate channel raw until calibration confidence is validated; this
  // field is exposed for estimator supervision and later hardware binding.
  float gyro_bias_rad_s = 0.0F;
  float yaw_rad = 0.0F;
  float yaw_rate_rad_s = 0.0F;
  float roll_rate_rad_s = 0.0F;
  float accel_magnitude_m_s2 = 9.81F;
  // One-shot/short-lived event latched from a raw accelerometer + gyro
  // touchdown signature. The IMU callback owns the latch because a real
  // impact can last only one or two samples while the control loop may run at
  // a different rate. It is sensor-backed; it is not a pose/contact ground
  // truth shortcut.
  bool hard_impact_event = false;
};

struct WheelOdometry {
  float position_m = 0.0F;
  float velocity_m_s = 0.0F;
  // Native +Y wheel encoder motion converted to ground-tangent metres.  These
  // fields preserve per-side data for relative trajectory observation; the
  // averaged fields above remain the balance controller's existing interface.
  float left_position_m = 0.0F;
  float right_position_m = 0.0F;
  float left_velocity_m_s = 0.0F;
  float right_velocity_m_s = 0.0F;
};

struct JointState {
  JointArray position_rad{};
  JointArray velocity_rad_s{};
  JointArray effort_nm{};
  bool effort_feedback_valid = false;
};

struct RobotState {
  float dt_s = 0.005F;
  // Optional transport sequence numbers. Zero means legacy callers do not
  // expose freshness and every update is treated as a new sample. Native
  // high-rate runners use these to prevent a 3-4 kHz control loop from
  // integrating a 500 Hz IMU/encoder packet six to eight times.
  std::uint64_t imu_sample_sequence = 0;
  std::uint64_t encoder_sample_sequence = 0;
  bool floor_contact = true;
  bool left_floor_contact = true;
  bool right_floor_contact = true;
  bool emergency_stop = false;
  ImuState imu{};
  WheelOdometry wheel{};
  JointState joints{};
};

struct ControlCommand {
  float target_forward_velocity_m_s = 0.0F;
  float target_yaw_rate_rad_s = 0.0F;
  JointArray target_joint_position_rad{};
  float target_body_height_m = geometry::neutral_spawn_z_m;
  bool request_enable = false;
  bool request_disable = false;
  bool request_emergency_stop = false;
  bool request_pose_change = false;
  bool request_height_change = false;
  bool request_stand_up = false;
  bool request_fixture_release = false;
};

struct MotorCommand {
  JointArray position_rad{};
  JointArray velocity_rad_s{};
  JointArray torque_nm{};
  JointArray kp_scale{};
  JointArray kd_scale{};
  // Additive feedforward torque, added to the joint PD term before the
  // torque_nm clamp in the leg command path. Zero by
  // default; used by LandingImpedance's virtual spring-damper force and by
  // JumpController's own feedforward.
  JointArray feedforward_nm{};
  bool enable_motors = false;
};

} // namespace rb::control
