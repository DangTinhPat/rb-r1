#pragma once

#include <array>

namespace rb::control {

// World-frame IMU-point velocity from gyro + encoders (see
// WholeBodyModel::imuVelocityThroughLeg). Valid only while supported.
struct ImuKinematicVelocity {
  float horizontal_m_s = 0.0F;
  float vertical_m_s = 0.0F;
  bool valid = false;
};

class ImuStateEstimator {
public:
  struct Config {
    float gravity_m_s2 = 9.81F;
    float q_angle = 0.001F;
    float q_bias = 0.003F;
    float r_measure = 0.03F;
    float accel_trust_gain = 40.0F;
    float accel_max_deviation_m_s2 = 4.0F;
    float accel_max_pitch_rate_rad_s = 1.5F;
    float accel_max_innovation_rad = 0.35F;
    // The accelerometer tilt channel is invalid while the commanded wheel
    // acceleration is large enough that actuator delay/model error dominates
    // the specific-force reading. The gyro remains the attitude source for
    // that short dynamic interval.
    float accel_command_gate_m_s2 = 0.5F;
    // Encoder/odometry body acceleration is the primary specific-force
    // compensation. The MPC command is only a bounded auxiliary feed-forward
    // term, never the sole dynamic measurement.
    float body_accel_limit_m_s2 = 4.0F;
    float commanded_accel_blend = 0.15F;
    // Filter encoder/gyro velocity before differentiating it. The resulting
    // acceleration and accelerometer branches then get the same one-pole lag.
    float kinematic_velocity_filter_tau_s = 0.02F;
    float kinematic_accel_trust_scale_m_s2 = 1.5F;
    float accel_correction_gain_hz = 55.0F;
    float seed_min_accel_m_s2 = 5.0F;
    float seed_max_accel_m_s2 = 20.0F;
    // The seed ends at seed_window_samples samples or seed_window_s of them, whichever comes first,
    // so a lower IMU rate does not stretch the window (50 samples = 0.1 s at 500 Hz).
    unsigned seed_window_samples = 50;
    float seed_window_s = 0.1F;
    // The qualification window is also a stationary gyro-bias calibration.
    // Keep the acceptance band conservative: a moving/impact sample must not
    // be learned as a sensor offset.
    float seed_max_gyro_rate_rad_s = 0.20F;
    // The qualification window supplies the initial attitude/bias seed. Keep
    // its covariance tight during bring-up; live accelerometer samples are
    // heavily contaminated by the first contact transient.
    float initial_angle_variance = 1.0e-8F;
    float initial_bias_variance = 1.0e-10F;
    float bias_quiet_max_accel_deviation_m_s2 = 0.35F;
    float bias_quiet_max_gyro_rate_rad_s = 0.12F;
    // Continuous-time gain for the gated stationary gyro-bias low-pass.
    // This is intentionally slow; a transient must not become calibration.
    float bias_learning_gain = 0.50F;
    // Weak integral correction from the accelerometer tilt innovation. It is
    // gated by the same quiet/force checks and bounded, so it can follow a
    // slowly wandering bias without learning drive acceleration as bias.
    float bias_innovation_gain_hz = 0.20F;
    float bias_innovation_limit_rad = 0.05F;
    float max_bias_rad_s = 0.20F;
    float impact_accel_m_s2 = 35.0F;
    float impact_pitch_rate_rad_s = 4.0F;
    float impact_latch_s = 0.06F;
  };

  struct Output {
    bool valid = false;
    bool seeded = false;
    bool accel_tilt_valid = false;
    bool hard_impact_event = false;
    float gyro_bias_rad_s = 0.0F;
    float pitch_rad = 0.0F;
    // Bias-corrected pitch rate for the controller. Impact detection remains
    // based on the raw gyro inside update().
    float pitch_rate_rad_s = 0.0F;
    float raw_pitch_rate_rad_s = 0.0F;
    float roll_rate_rad_s = 0.0F;
    float yaw_rate_rad_s = 0.0F;
    float accel_magnitude_m_s2 = 9.81F;
    float body_accel_x_m_s2 = 0.0F;
    float accel_tilt_x_m_s2 = 0.0F;
    bool body_accel_compensation_valid = false;
    // Continuous 0..1 weight applied to the accelerometer attitude correction.
    float accel_trust = 0.0F;
  };

  using KinematicVelocity = ImuKinematicVelocity;

  ImuStateEstimator() = default;
  explicit ImuStateEstimator(Config config) : config_(config) {}

  void setConfig(Config config) noexcept { config_ = config; }

  void reset() noexcept;
  // now_s is the local monotonic/simulation time used only for the bounded
  // impact latch. dt_s is the elapsed time between IMU samples, not the
  // controller loop period.
  [[nodiscard]] Output update(const std::array<float, 3> &accel_m_s2,
                              const std::array<float, 3> &gyro_rad_s, float dt_s, float now_s,
                              float commanded_ground_accel_m_s2 = 0.0F, bool pose_changing = false,
                              bool quiet_hint = false,
                              KinematicVelocity imu_velocity = {}) noexcept;

private:
  Config config_{};
  float pitch_rad_ = 0.0F;
  float pitch_lagged_rad_ = 0.0F;
  std::array<float, 2> accel_lagged_{};
  std::array<float, 2> kinematic_velocity_{};
  std::array<float, 2> kinematic_accel_{};
  bool kinematic_initialized_ = false;
  float gyro_bias_ = 0.0F;
  float p_[2][2] = {{0.0F, 0.0F}, {0.0F, 0.0F}};
  std::array<float, 3> seed_sum_{};
  float seed_gyro_sum_ = 0.0F;
  unsigned seed_gyro_count_ = 0;
  unsigned seed_count_ = 0;
  float seed_time_s_ = 0.0F;
  bool seeded_ = false;
  float impact_until_s_ = 0.0F;
};

} // namespace rb::control
