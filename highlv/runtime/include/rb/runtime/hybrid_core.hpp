// Telemetry-driven 200 Hz hybrid state machine; the policy ticks at 100 Hz.
// Port of rb_runtime/hybrid_core.py. Pure logic: no sockets, no clocks except the
// frame stamp, so every path can be driven by tests.
#pragma once
#include <array>
#include <deque>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "rb/runtime/json.hpp"
#include "rb/runtime/logic.hpp"
#include "rb/runtime/policy.hpp"
#include "rb/runtime/protocol.hpp"

namespace rb::runtime {

enum class Mode { WaitLink = 0, Disarmed, Home, Standup, Warmup, Model, Fault };
const char *modeName(Mode mode);

enum Fault : std::uint32_t {
  kNoFault = 0,
  kTelemetryTimeout = 1,
  kInvalidTelemetry,
  kDriverFault,
  kClockGap,
  kMissingAppliedTarget,
  kLogicFault,
  kPolicyFault,
  kPolicyDeadline,
  kFall,
  kTransportFault,
  kControlDeadline,
  kAckTimeout,
};

struct CoreConfig {
  double telemetry_timeout_ms = 30, target_ack_timeout_ms = 60, target_ack_timeout_min_ms = 30,
         target_ack_latency_margin_ms = 8, target_ack_wheel_settle_ms = 25,
         target_ack_unknown_cycle_ms = 15, target_ack_diag_max_age_ms = 500;
  double inference_deadline_ms = 10, inference_hard_deadline_ms = 15;
  int max_consecutive_deadline_misses = 3;
  double home_hold_s = 3, warmup_start_s = 12.5, handoff_earliest_s = 13;
  int warmup_steps = 50;
  std::array<double, 4> home_joints{2.35706715, 2.35706715, 0.0, 0.0};
  double home_tolerance_rad = 0.15;
  static CoreConfig fromJson(const Json &json);  // throws on a missing/invalid key
};

// Operator request that waits for an MCU reply (HOME capture, IMU calibration).
struct PendingRequest {
  std::string state = "idle";  // idle, pending, running, done, error
  std::string message;
  std::optional<std::uint32_t> request_seq;
  std::optional<double> sent_at, started_at;
  Json toJson() const;
};

struct Counters {
  std::uint64_t telemetry = 0, telemetry_received = 0, rejected = 0, inferences = 0, deadline_misses = 0;
};

struct Diagnostics {
  std::array<std::uint32_t, hcr::kDiagWords> words{};
  double transport_age_s = 0;
  std::uint32_t operator[](unsigned i) const { return words[i]; }
};

class HybridCore {
 public:
  HybridCore(Policy &policy, Logic &logic, CoreConfig config);

  void newEpoch();
  hcr::Frame hello();
  hcr::Frame status();
  hcr::Frame fail(const std::string &name, const std::string &detail = "");
  std::vector<hcr::Frame> watchdog(double now);
  std::vector<hcr::Frame> receive(const hcr::Frame &frame, double now);
  // Operator intent from the local command socket. Throws std::invalid_argument for a
  // nonfinite or out-of-range request (nothing is changed).
  std::vector<hcr::Frame> command(const std::string &op, double now, double vx = 0, double wz = 0);
  std::vector<hcr::Frame> estop();
  Json snapshot(std::optional<double> now = std::nullopt);

  // Session state (read by the app loop, the status writer and tests).
  Policy &policy;
  Logic &logic;
  CoreConfig config;
  std::uint32_t tx_seq = 0, epoch = 0, fault = 0;
  Counters counters;
  Mode mode = Mode::WaitLink;
  std::string reason, phase = "Idle";
  bool link = false, support = false;
  std::optional<std::uint32_t> rx_seq, first_target_seq, last_ack;
  std::optional<std::int64_t> last_stamp, start_stamp, last_infer_stamp, next_infer_stamp, handoff_us;
  std::optional<double> last_rx, last_ack_rx, offset_us, mcu_diagnostics_offset_us, mcu_diagnostics_rx;
  std::optional<hcr::Telemetry> last_t;
  std::optional<std::uint32_t> last_telemetry_flags;
  std::optional<std::pair<std::uint32_t, double>> encoder_source;  // telemetry seq, rx time
  std::optional<Diagnostics> mcu_diagnostics;
  std::optional<std::string> last_command_error;
  std::deque<std::uint32_t> sent_targets;
  std::unordered_map<std::uint32_t, double> sent_target_at;
  std::deque<double> ack_latency_ms;
  Action action{};
  std::array<double, 2> velocity{};
  std::array<double, 6> target{};
  double inference_max_ms = 0;
  int consecutive_misses = 0;
  PendingRequest encoder_reset, imu_calibration;

 private:
  hcr::Frame frame(std::uint8_t kind, std::vector<std::uint8_t> payload);
  hcr::Frame output(std::uint32_t seq, std::uint16_t profile = 0, std::uint16_t flags = 0,
                    const std::array<double, 6> &target = {},
                    const std::array<double, 4> &ff = {}, std::optional<double> sent_at = std::nullopt);
  double targetAckWindowMs(double now) const;
  std::optional<std::string> targetAckTimeoutDetail(std::uint32_t applied_seq, double now);
  std::vector<hcr::Frame> telemetry(const hcr::Frame &frame, double now);
  std::vector<hcr::Frame> receiveDiagnostics(const hcr::Frame &frame, double now);
  std::vector<hcr::Frame> receiveEncoderHomeAck(const hcr::Frame &frame);
  std::vector<hcr::Frame> receiveImuAck(const hcr::Frame &frame, double now);
  std::vector<hcr::Frame> failWithStatus(const std::string &name, const std::string &detail = "");
  bool controlActive() const { return mode >= Mode::Home && mode <= Mode::Model; }
};

}  // namespace rb::runtime
