// HCR1 frames as the LF master sees them; see docs/protocol.md. Framing and CRC come
// from protocol/include/hcr_protocol.h so there is a single implementation per layer.
#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include "hcr_protocol.h"

namespace rb::runtime::hcr {

constexpr std::uint32_t kSchema = HCR_SCHEMA;
constexpr std::uint8_t kHello = HCR_HELLO, kHelloAck = HCR_HELLO_ACK, kTelemetry = HCR_TELEMETRY,
                       kTarget = HCR_TARGET, kStatus = HCR_STATUS, kEncoderHome = HCR_ENCODER_HOME,
                       kEncoderHomeAck = HCR_ENCODER_HOME_ACK, kCalImu = HCR_CAL_IMU,
                       kCalImuAck = HCR_CAL_IMU_ACK, kExecutionDiag = HCR_EXECUTION_DIAG;
constexpr std::uint32_t kHomeApplied = HCR_HOME_APPLIED, kHomeNotOff = HCR_HOME_NOT_OFF,
                        kHomeFeedback = HCR_HOME_FEEDBACK, kHomeRefused = HCR_HOME_REFUSED;
constexpr std::uint32_t kCalAccepted = HCR_CAL_IMU_ACCEPTED, kCalNotOff = HCR_CAL_IMU_NOT_OFF,
                        kCalFeedback = HCR_CAL_IMU_FEEDBACK, kCalRefused = HCR_CAL_IMU_REFUSED,
                        kCalBusy = HCR_CAL_IMU_BUSY;
constexpr std::uint32_t kImuOk = HCR_IMU_OK, kEncoderOk = HCR_ENCODER_OK, kCalibrated = HCR_CALIBRATED,
                        kEstop = HCR_ESTOP, kTorqueValid = HCR_TORQUE_VALID,
                        kTargetValid = HCR_TARGET_VALID, kDriverFault = HCR_DRIVER_FAULT;
constexpr std::uint16_t kDisabled = HCR_DISABLED, kHomePd = HCR_HOME_PD, kRecovery = HCR_RECOVERY,
                        kPolicyPd = HCR_POLICY_PD;
constexpr std::size_t kTelemetrySize = 128, kTargetSize = 52, kHelloSize = 12, kDiagWords = 24;

enum DiagIndex : unsigned { kDiagCycleL = HCR_DIAG_CYCLE_L_US, kDiagCycleR = HCR_DIAG_CYCLE_R_US,
                            kDiagCycleLMax = HCR_DIAG_CYCLE_L_MAX_US, kDiagCycleRMax = HCR_DIAG_CYCLE_R_MAX_US };

// Names of the 24 diagnostic words, in wire order (status file keys).
extern const char *const kDiagFields[kDiagWords];

struct Frame {
  std::uint8_t kind = 0;
  std::uint32_t seq = 0;
  std::uint64_t stamp = 0;
  std::vector<std::uint8_t> payload;
  std::vector<std::uint8_t> encode() const;  // throws std::length_error if payload > 256
};

class Parser {
 public:
  std::vector<Frame> feed(const std::uint8_t *data, std::size_t size);
  std::uint32_t errors() const { return state_.errors; }

 private:
  HcrParser state_{};
};

// True when `current` is strictly newer than `previous` (or there is no previous).
inline bool newer(std::uint32_t current, const std::optional<std::uint32_t> &previous) {
  if (!previous) return true;
  std::uint32_t d = current - *previous;
  return d > 0 && d < 0x80000000u;
}

struct Telemetry {
  std::uint32_t epoch = 0, flags = 0, applied_seq = 0;
  std::array<double, 3> gyro{}, gravity{}, accel{};
  std::array<double, 6> q{}, qd{};
  std::array<double, 2> hip_torque{};
  std::array<double, 6> applied_target{};
  static Telemetry decode(const std::vector<std::uint8_t> &payload);  // throws std::invalid_argument
  std::vector<std::uint8_t> encode() const;
};

std::vector<std::uint8_t> helloData(std::uint32_t epoch);  // epoch, 200 Hz, 100 Hz, schema
std::vector<std::uint8_t> statusData(std::uint32_t epoch, std::uint32_t mode, std::uint32_t fault,
                                     std::uint32_t rejected);
std::vector<std::uint8_t> targetData(std::uint32_t epoch, std::uint32_t seq, std::uint16_t profile,
                                     std::uint16_t flags, const std::array<double, 6> &target,
                                     const std::array<double, 4> &ff);
std::vector<std::uint8_t> pair(std::uint32_t epoch, std::uint32_t source_seq);  // ENCODER_HOME / CAL_IMU
std::vector<std::uint8_t> ack(std::uint32_t epoch, std::uint32_t request_seq, std::uint32_t result);

struct HelloData { std::uint32_t epoch; std::uint16_t telemetry_hz, policy_hz; std::uint32_t schema; };
HelloData decodeHello(const std::vector<std::uint8_t> &payload);  // throws on wrong size
struct AckData { std::uint32_t epoch, request_seq, result; };
AckData decodeAck(const std::vector<std::uint8_t> &payload);       // throws on wrong size
std::array<std::uint32_t, kDiagWords> decodeDiagnostics(const std::vector<std::uint8_t> &payload);

}  // namespace rb::runtime::hcr
