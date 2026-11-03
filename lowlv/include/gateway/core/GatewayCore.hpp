#pragma once
#include "gateway/imu/Attitude.hpp"
#include "gateway/motor/MotorBackend.hpp"

namespace gateway {
struct WirePacket {
  std::uint16_t size = 0;
  std::array<std::uint8_t, HCR_MAX_FRAME> bytes{};
};
enum class LocalFault : std::uint32_t {
  None, TargetTimeout, Sensors, Driver, Estop, Transport, RemoteFault
};
// Console-only context, frozen BEFORE stop changes the state/validity flags.
struct SensorFaultContext {
  std::uint64_t at_us=0, imu_stamp_us=0, encoder_stamp_us=0;
  std::uint32_t flags=0;
  bool motor_ready=false, encoders_valid=false, homed=false;
  std::uint16_t target_flags=0;
  std::uint32_t target_seq=0;
  std::array<std::uint64_t,6> channel_stamp_us{};
};
struct Diagnostics {
  std::uint32_t epoch = 0, rx_targets = 0, rejected = 0, applied_seq = 0;
  std::uint32_t remote_mode = 0, remote_fault = 0, telemetry_flags = 0;
  LocalFault fault = LocalFault::None;
  bool connected = false, armed = false, enabled = false, motors_ready = false;
  SensorFaultContext sensor_fault{};
};

// Owned by one service/motor task. UART is isolated by bounded queues; expiry
// is checked here even when UART/console/network stalls. No heap allocation.
class GatewayCore {
 public:
  explicit GatewayCore(MotorBackend& motor) : motor_(motor) {}
  WirePacket receive(const HcrFrame&, std::uint64_t now,
                     std::uint64_t received_us = 0) noexcept;
  WirePacket telemetry(std::uint64_t now) noexcept;
  WirePacket executionDiagnostics(std::uint64_t now) noexcept;
  WirePacket requestSession(std::uint64_t now) noexcept;
  // Call only for a telemetry packet actually written by UART, with its
  // original sample timestamp, never for HELLO_ACK packets.
  void telemetrySent(std::uint32_t epoch, std::uint32_t seq,
                     std::uint64_t sample_us) noexcept;
  void setImu(const ImuObservation& imu) noexcept {
    imu_ = imu;
    if (imu.valid) imu_calibration_active_ = false;
  }
  bool takeImuCalibrationRequest() noexcept {
    const bool requested = imu_calibration_requested_;
    imu_calibration_requested_ = false;
    return requested;
  }
  void tick(std::uint64_t now) noexcept;
  void stop(LocalFault reason = LocalFault::None) noexcept;
  void estop() noexcept { stop(LocalFault::Estop); motor_.emergencyStop(); }
  bool clearLocalFault() noexcept;
  // Sticky until reboot. No session/RESET/ARM can escape OTA maintenance.
  void enterMaintenance() noexcept {
    if (maintenance_) return;
    maintenance_ = true; stop(); motor_.beginMaintenance();
  }
  bool maintenance() const noexcept { return maintenance_; }
  bool maintenanceReady() const noexcept {
    return maintenance_ && !armed_ && !enabled_ && motor_.disabled();
  }
  Diagnostics diagnostics(std::uint64_t now) const noexcept;
 private:
  struct SentSample { std::uint32_t seq = 0; std::uint64_t stamp = 0; bool valid = false; };
  bool freshSensors(std::uint64_t now) const noexcept;
  std::uint32_t flags(std::uint64_t now) const noexcept;
  bool sourceValid(std::uint32_t seq, std::uint64_t now) const noexcept;
  void newSession(std::uint32_t epoch) noexcept;
  MotorBackend& motor_;
  ImuObservation imu_{};
  MotorFeedback feedback_{};
  HcrTarget target_{};
  std::array<float, 6> applied_{};
  std::array<SentSample, 32> history_{};
  unsigned history_next_ = 0;
  std::uint32_t epoch_ = 0, tx_seq_ = 0, rx_seq_ = 0, target_seq_ = 0, applied_seq_ = 0;
  std::uint32_t rx_targets_ = 0, rejected_ = 0, remote_mode_ = 0, remote_fault_ = 0;
  std::uint64_t rx_stamp_ = 0, deadline_ = 0, last_tick_ = 0;
  std::uint64_t last_rx_us_ = 0, last_target_rx_us_ = 0;
  std::uint32_t rx_queue_max_us_ = 0;
  bool connected_ = false, have_rx_ = false, armed_ = false, enabled_ = false;
  bool applied_valid_ = false, reset_pending_ = false;
  bool maintenance_ = false;
  bool disable_ack_pending_ = false;
  bool imu_calibration_requested_ = false, imu_calibration_active_ = false;
  std::uint64_t imu_calibration_started_us_ = 0;
  LocalFault fault_ = LocalFault::None;
  SensorFaultContext sensor_fault_{};
};
}  // namespace gateway
