#pragma once
#include <array>
#include <cstdint>
#include "gateway/protocol/hcr_protocol.h"

namespace gateway {
// Driver boundary: SI units, MJCF coordinates, absolute homed angles (no
// q_default subtraction). HCR joint order: LH,RH,LK,RK,LW,RW.
// Feedback LK/RK are knee hinge angles; a crank encoder must be converted
// using knee=phi-hip. Target channels 2/3 below are phi, a different coordinate.
struct MotorFeedback {
  // Oldest actual encoder sample, possibly stale/zero when invalid. A stamp
  // alone never grants validity; consumers must check encoders_valid as well.
  std::uint64_t stamp_us = 0;
  std::array<float, 6> q{}, qd{};
  std::array<float, 2> hip_torque{};
  bool encoders_valid = false, homed = false, torque_valid = false;
  bool driver_fault = false, estop = false;
};

inline std::array<float, 6> coreToHcr(const std::array<float, 6>& core) {
  return {core[0], core[3], core[1], core[4], core[2], core[5]};
}

enum class ApplyResult { Pending, Applied, Fault };
struct MotorApplied {
  std::uint32_t epoch = 0, seq = 0;
  std::array<float, 6> target{};
};

// Optional execution measurements. These are diagnostic snapshots only;
// they never count as an applied ACK or renew a command deadline.
struct MotorExecutionStats {
  bool supported=false;
  std::uint32_t error=0, pending_seq=0, pending_age_us=0, superseded=0, batch_flags=0;
  // batch_flags: claimed L/R bits0/1, pending bit2, expired lease/batch
  // at frozen worker fault: L bits8/9, R bits10/11.
  std::uint32_t applied_latency_max_us=0;
  std::array<std::uint32_t,2> cycle_us{}, cycle_max_us{}, io_us{}, io_max_us{}, bus_error{}, bus_stage{};
};

class MotorBackend {
 public:
  virtual ~MotorBackend() = default;
  virtual bool ready() const noexcept = 0;
  virtual MotorFeedback feedback(std::uint64_t now) noexcept = 0;
  // All methods run on the dedicated service task; they must be bounded and
  // nonblocking. Bus workers must enforce the original receipt deadline even
  // if this service task stalls. Driver-side power-loss protection is separate.
  virtual void disable() noexcept = 0;
  virtual void emergencyStop() noexcept { disable(); }
  virtual void beginMaintenance() noexcept { emergencyStop(); }
  // disable() requests cancellation; only disabled() confirms physical replies.
  virtual bool disabled() const noexcept = 0;
  virtual bool resetFault() noexcept { return true; }
  virtual MotorExecutionStats executionStats(std::uint64_t) const noexcept { return {}; }
  // Software zero only. The backend independently verifies all six OFF,
  // fresh stationary feedback and the configured home limits.
  virtual bool captureHome(const std::array<float,4>&, std::uint64_t) noexcept { return false; }
  // Nonblocking submit/poll. Pending does not ACK or fault. Applied identifies
  // the completed batch (possibly older than the newest request); never ACK an
  // enqueue. deadline_us is the original receipt + TTL, never a tick heartbeat.
  // Targets: hipL,hipR,crankL,crankR,wheelL,wheelR: 4 rad + 2 rad/s, NO torque.
  // Feedforward: hipL,hipR,kneeL,kneeR angular velocities in rad/s.
  // A physical crank motor tracks phi_ref directly after sign/zero conversion;
  // do not send phi_ref-hip_ref to a driver that measures the crank coordinate.
  // A physical knee driver instead tracks phi_ref-hip_ref, with knee FF.
  // Torque in MotorFeedback is optional upstream telemetry, not a downlink command.
  // No actor clipping here. See HCR1_PROTOCOL.md and FLOW_AUDIT.md.
  virtual ApplyResult apply(const HcrTarget&, std::uint32_t seq,
                            std::uint64_t deadline_us, std::uint64_t now,
                            MotorApplied& applied) noexcept = 0;
};

// Backend for a robot without motor drivers. It never claims encoder, homing,
// torque or applied-target validity and cannot arm.
class UnavailableMotors final : public MotorBackend {
 public:
  bool ready() const noexcept override { return false; }
  MotorFeedback feedback(std::uint64_t) noexcept override { return {}; }
  void disable() noexcept override {}
  bool disabled() const noexcept override { return true; }
  ApplyResult apply(const HcrTarget&, std::uint32_t, std::uint64_t,
                    std::uint64_t, MotorApplied&) noexcept override {
    return ApplyResult::Fault;
  }
};
}  // namespace gateway
