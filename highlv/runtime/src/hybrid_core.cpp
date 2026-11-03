#include "rb/runtime/hybrid_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>

namespace rb::runtime {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kPi = 3.14159265358979323846;
constexpr const char *kModeNames[] = {"WAIT_LINK", "DISARMED", "HOME", "STANDUP", "WARMUP", "MODEL", "FAULT"};
constexpr const char *kPhases[] = {"Idle", "Folding", "Kneeling", "Catching", "Support", "Failed"};
struct FaultName { const char *name; Fault code; };
constexpr FaultName kFaults[] = {
    {"telemetry_timeout", kTelemetryTimeout}, {"invalid_telemetry", kInvalidTelemetry},
    {"driver_fault", kDriverFault},           {"clock_gap", kClockGap},
    {"missing_applied_target", kMissingAppliedTarget}, {"logic_fault", kLogicFault},
    {"policy_fault", kPolicyFault},           {"policy_deadline", kPolicyDeadline},
    {"fall", kFall},                          {"transport_fault", kTransportFault},
    {"control_deadline", kControlDeadline},   {"ack_timeout", kAckTimeout}};

Fault faultCode(const std::string &name) {
  for (const auto &f : kFaults)
    if (name == f.name) return f.code;
  throw std::invalid_argument("unknown fault " + name);
}

std::uint64_t monotonicUs() {
  using namespace std::chrono;
  return static_cast<std::uint64_t>(duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}
double monotonicS() { return static_cast<double>(monotonicUs()) / 1e6; }

std::string fixed(double v, int digits) {
  char buf[48];
  std::snprintf(buf, sizeof buf, "%.*f", digits, v);
  return buf;
}
std::string fixed1(double v) { return fixed(v, 1); }
std::string optionalSeq(const std::optional<std::uint32_t> &v) { return v ? std::to_string(*v) : "None"; }

double number(const Json &j, const char *key) { return j.at(key).number(); }
double number(const Json &j, const char *key, double fallback) {
  const Json *v = j.find(key);
  return v ? v->number() : fallback;
}

PendingRequest message(const char *state, const std::string &text) {
  PendingRequest r;
  r.state = state;
  r.message = text;
  return r;
}
}  // namespace

const char *modeName(Mode mode) { return kModeNames[static_cast<int>(mode)]; }

CoreConfig CoreConfig::fromJson(const Json &j) {
  CoreConfig c;
  c.telemetry_timeout_ms = number(j, "telemetry_timeout_ms");
  c.target_ack_timeout_ms = number(j, "target_ack_timeout_ms");
  c.target_ack_timeout_min_ms = number(j, "target_ack_timeout_min_ms", c.target_ack_timeout_min_ms);
  c.target_ack_latency_margin_ms = number(j, "target_ack_latency_margin_ms", c.target_ack_latency_margin_ms);
  c.target_ack_wheel_settle_ms = number(j, "target_ack_wheel_settle_ms", c.target_ack_wheel_settle_ms);
  c.target_ack_unknown_cycle_ms = number(j, "target_ack_unknown_cycle_ms", c.target_ack_unknown_cycle_ms);
  c.target_ack_diag_max_age_ms = number(j, "target_ack_diag_max_age_ms", c.target_ack_diag_max_age_ms);
  c.inference_deadline_ms = number(j, "inference_deadline_ms");
  c.inference_hard_deadline_ms = number(j, "inference_hard_deadline_ms");
  c.max_consecutive_deadline_misses = static_cast<int>(number(j, "max_consecutive_deadline_misses"));
  c.home_hold_s = number(j, "home_hold_s");
  c.warmup_start_s = number(j, "warmup_start_s");
  c.handoff_earliest_s = number(j, "handoff_earliest_s");
  c.warmup_steps = static_cast<int>(number(j, "warmup_steps"));
  c.home_tolerance_rad = number(j, "home_tolerance_rad");
  const auto &home = j.at("home_joints").items();
  if (home.size() != 4) throw std::invalid_argument("home_joints needs 4 values");
  for (unsigned i = 0; i < 4; ++i) c.home_joints[i] = home[i].number();
  return c;
}

Json PendingRequest::toJson() const {
  Json j = Json::object();
  j.set("state", state);
  if (request_seq) j.set("request_seq", *request_seq);
  if (sent_at) j.set("sent_at", *sent_at);
  if (started_at) j.set("started_at", *started_at);
  j.set("message", message);
  return j;
}

HybridCore::HybridCore(Policy &p, Logic &l, CoreConfig c) : policy(p), logic(l), config(std::move(c)) {
  newEpoch();
}

void HybridCore::newEpoch() {
  policy.reset();
  logic.reset();
  std::random_device rd;
  epoch = rd();
  if (epoch == 0) epoch = 1;
  mode = Mode::WaitLink;
  fault = 0;
  reason.clear();
  link = false;
  rx_seq.reset();
  last_stamp.reset(); last_rx.reset(); last_t.reset(); start_stamp.reset();
  last_infer_stamp.reset(); next_infer_stamp.reset();
  last_ack_rx.reset(); last_ack.reset(); first_target_seq.reset();
  sent_targets.clear(); sent_target_at.clear(); ack_latency_ms.clear();
  support = false;
  phase = "Idle";
  action = {};
  velocity = {};
  target = {};
  handoff_us.reset();
  inference_max_ms = 0;
  consecutive_misses = 0;
  offset_us.reset();
  mcu_diagnostics.reset(); mcu_diagnostics_rx.reset(); mcu_diagnostics_offset_us.reset();
  last_telemetry_flags.reset();
  last_command_error.reset();
  encoder_source.reset();
  encoder_reset = message("idle", "Chưa chụp HOME trong phiên này");
  imu_calibration = message("idle", "Chưa yêu cầu calib IMU trong phiên này");
}

hcr::Frame HybridCore::frame(std::uint8_t kind, std::vector<std::uint8_t> payload) {
  hcr::Frame f{kind, tx_seq, monotonicUs(), std::move(payload)};
  ++tx_seq;
  return f;
}

hcr::Frame HybridCore::hello() { return frame(hcr::kHello, hcr::helloData(epoch)); }

hcr::Frame HybridCore::status() {
  return frame(hcr::kStatus, hcr::statusData(epoch, static_cast<std::uint32_t>(mode), fault,
                                             static_cast<std::uint32_t>(counters.rejected)));
}

// Four absolute hip/crank angles + two wheel angular rates; ff is hipL/hipR/kneeL/kneeR
// rate feedforward. Neither contains a torque command.
hcr::Frame HybridCore::output(std::uint32_t seq, std::uint16_t profile, std::uint16_t flags,
                              const std::array<double, 6> &tgt, const std::array<double, 4> &ff,
                              std::optional<double> sent_at) {
  hcr::Frame f = frame(hcr::kTarget, hcr::targetData(epoch, seq, profile, flags, tgt, ff));
  if (profile && !first_target_seq) first_target_seq = f.seq;
  if (profile) {
    if (sent_targets.size() == 32) {
      sent_target_at.erase(sent_targets.front());
      sent_targets.pop_front();
    }
    sent_targets.push_back(f.seq);
    sent_target_at[f.seq] = sent_at ? *sent_at : monotonicS();
  }
  return f;
}

hcr::Frame HybridCore::fail(const std::string &name, const std::string &detail) {
  if (name != "invalid_telemetry" && encoder_reset.state == "pending")
    encoder_reset = message("error", "Reset encoder bị hủy vì runtime có lỗi; không tự gửi lại.");
  // A driver fault/E-stop takes precedence over missing startup calibration: that
  // startup fault permits a disarmed HOME capture.
  if (mode != Mode::Fault || name == "driver_fault") {
    fault = faultCode(name);
    reason = detail.empty() ? name : detail;
  }
  mode = Mode::Fault;
  return output(rx_seq.value_or(0));
}

std::vector<hcr::Frame> HybridCore::failWithStatus(const std::string &name, const std::string &detail) {
  hcr::Frame f = fail(name, detail);
  return {std::move(f), status()};
}

// Bounded ACK window from recent applied latency and phase.
double HybridCore::targetAckWindowMs(double now) const {
  const double floor_ms = config.target_ack_timeout_min_ms;
  const double ceiling_ms = config.target_ack_timeout_ms;
  const double margin_ms = config.target_ack_latency_margin_ms;
  std::vector<double> samples(ack_latency_ms.begin(), ack_latency_ms.end());
  std::sort(samples.begin(), samples.end());
  double observed_ms = 0, estimate_ms = floor_ms;
  if (!samples.empty()) {
    observed_ms = samples.size() < 8
                      ? samples.back()
                      : samples[static_cast<std::size_t>(std::ceil(0.9 * static_cast<double>(samples.size()))) - 1];
    estimate_ms = observed_ms + margin_ms;
  }
  if (phase == "Catching") {
    const double age_ms = mcu_diagnostics_rx ? std::max(0.0, 1000 * (now - *mcu_diagnostics_rx)) : kInf;
    double cycle_ms = config.target_ack_unknown_cycle_ms;
    if (mcu_diagnostics && age_ms <= config.target_ack_diag_max_age_ms) {
      const auto &d = *mcu_diagnostics;
      cycle_ms = std::max({d[hcr::kDiagCycleL], d[hcr::kDiagCycleR], d[hcr::kDiagCycleLMax],
                           d[hcr::kDiagCycleRMax]}) / 1000.0;
    }
    estimate_ms = std::max(estimate_ms, config.target_ack_wheel_settle_ms + std::max(cycle_ms, observed_ms) + margin_ms);
  }
  return std::max(floor_ms, std::min(ceiling_ms, estimate_ms));
}

// Track applied progress and fault only after the adaptive window. MCU TARGET TTL and
// telemetry freshness remain independent hard limits.
std::optional<std::string> HybridCore::targetAckTimeoutDetail(std::uint32_t applied_seq, double now) {
  if (!first_target_seq) return std::nullopt;
  if (std::find(sent_targets.begin(), sent_targets.end(), applied_seq) != sent_targets.end() &&
      hcr::newer(applied_seq, last_ack)) {
    auto at = sent_target_at.find(applied_seq);
    if (at != sent_target_at.end() && now >= at->second) {
      if (ack_latency_ms.size() == 32) ack_latency_ms.pop_front();
      ack_latency_ms.push_back(1000 * (now - at->second));
    }
    last_ack = applied_seq;
    last_ack_rx = now;
  }
  const double age = last_ack_rx ? now - *last_ack_rx : kInf;
  const double timeout_ms = targetAckWindowMs(now);
  if (age <= timeout_ms / 1000) return std::nullopt;
  return "no applied ACK for " + fixed1(1000 * age) + " ms (adaptive window " + fixed1(timeout_ms) +
         " ms); telemetry applied_seq=" + std::to_string(applied_seq) + ", last_ack_seq=" +
         optionalSeq(last_ack) + ", newest_sent_seq=" +
         (sent_targets.empty() ? std::string("None") : std::to_string(sent_targets.back()));
}

std::vector<hcr::Frame> HybridCore::watchdog(double now) {
  if (encoder_reset.state == "pending" && now - *encoder_reset.sent_at > 1.0)
    encoder_reset = message("error", "ESP chưa xác nhận Reset encoder; không tự gửi lại. Kiểm tra firmware/link.");
  if (imu_calibration.state == "pending" && now - *imu_calibration.sent_at > 1.0)
    imu_calibration = message("error", "ESP chưa xác nhận yêu cầu calib IMU; không tự gửi lại.");
  if (imu_calibration.state == "running" && now - *imu_calibration.started_at > 5.0)
    imu_calibration = message("error", "IMU chưa hợp lệ sau 5 s; giữ robot đứng yên rồi thử calib lại.");
  if (controlActive() && (!last_rx || now - *last_rx > config.telemetry_timeout_ms / 1000))
    return failWithStatus("telemetry_timeout");
  return {};
}

std::vector<hcr::Frame> HybridCore::receive(const hcr::Frame &f, double now) {
  // The MCU can reboot while LF retains its old sequence/timestamp baseline.
  if (f.kind == hcr::kHello) {
    if (f.payload == hcr::helloData(0)) {
      hcr::Frame disabled = output(f.seq);
      newEpoch();
      return {std::move(disabled), hello(), status()};
    }
    ++counters.rejected;
    return {};
  }
  if (f.kind != hcr::kHelloAck && f.kind != hcr::kTelemetry && f.kind != hcr::kEncoderHomeAck &&
      f.kind != hcr::kCalImuAck && f.kind != hcr::kExecutionDiag) {
    ++counters.rejected;
    return {};
  }
  if (f.payload.size() < 4 || hcr_u32(f.payload.data()) != epoch || !hcr::newer(f.seq, rx_seq)) {
    ++counters.rejected;
    return {};
  }
  if (f.kind == hcr::kExecutionDiag) return receiveDiagnostics(f, now);
  rx_seq = f.seq;
  try {
    if (f.kind == hcr::kEncoderHomeAck) return receiveEncoderHomeAck(f);
    if (f.kind == hcr::kCalImuAck) return receiveImuAck(f, now);
    if (f.kind == hcr::kHelloAck) {
      const auto h = hcr::decodeHello(f.payload);
      if (h.epoch != epoch || h.telemetry_hz != 200 || h.policy_hz != 100 || h.schema != hcr::kSchema)
        throw std::invalid_argument("interface mismatch");
      const double offset = now * 1e6 - static_cast<double>(f.stamp);
      mcu_diagnostics_offset_us = mcu_diagnostics_offset_us ? std::min(offset, *mcu_diagnostics_offset_us) : offset;
      if (!link) {
        link = true;
        mode = Mode::Disarmed;
      }
      return {status()};
    }
    if (!link) {
      ++counters.rejected;
      return {hello()};
    }
    ++counters.telemetry_received;
    return telemetry(f, now);
  } catch (const std::invalid_argument &error) {
    return failWithStatus("invalid_telemetry", error.what());
  } catch (const std::length_error &error) {
    return failWithStatus("invalid_telemetry", error.what());
  }
}

// Optional measurements share sequence ordering, but cannot renew telemetry/ACK
// watchdogs, seed encoder source, or emit TARGET.
std::vector<hcr::Frame> HybridCore::receiveDiagnostics(const hcr::Frame &f, double now) {
  Diagnostics data;
  try {
    data.words = hcr::decodeDiagnostics(f.payload);
  } catch (const std::invalid_argument &) {
    ++counters.rejected;
    return {};
  }
  if (!link) {
    ++counters.rejected;
    return {};
  }
  const double offset = now * 1e6 - static_cast<double>(f.stamp);
  double baseline = offset;
  if (mcu_diagnostics_offset_us) baseline = std::min(baseline, *mcu_diagnostics_offset_us);
  if (offset_us) baseline = std::min(baseline, *offset_us);
  mcu_diagnostics_offset_us = baseline;
  data.transport_age_s = std::max(0.0, (offset - baseline) / 1e6);
  mcu_diagnostics = data;
  mcu_diagnostics_rx = now;
  rx_seq = f.seq;
  return {};
}

std::vector<hcr::Frame> HybridCore::receiveEncoderHomeAck(const hcr::Frame &f) {
  const auto ack = hcr::decodeAck(f.payload);
  if (encoder_reset.state != "pending" || ack.request_seq != encoder_reset.request_seq) {
    ++counters.rejected;
    return {};
  }
  if (ack.result == hcr::kHomeApplied) {
    // This button explicitly resets the software reference and LF session. MCU local
    // fault latch/HOME remain untouched by epoch.
    PendingRequest imu = imu_calibration;
    hcr::Frame disabled = output(rx_seq.value_or(0));
    newEpoch();
    encoder_reset = message("done", "ESP đã chụp HOME. Giữ Disable; ARM là thao tác riêng.");
    imu_calibration = imu;
    return {std::move(disabled), hello(), status()};
  }
  const char *text = "Kết quả Reset encoder không hợp lệ";
  if (ack.result == hcr::kHomeNotOff) text = "ESP chưa xác nhận đủ 6 motor OFF; bấm Disable và đợi.";
  else if (ack.result == hcr::kHomeFeedback) text = "Feedback encoder cũ hoặc thiếu; chưa chụp HOME.";
  else if (ack.result == hcr::kHomeRefused) text = "ESP từ chối: encoder còn chuyển động, bus/lỗi hoặc chưa OFF.";
  encoder_reset = message("error", text);
  return {};
}

std::vector<hcr::Frame> HybridCore::receiveImuAck(const hcr::Frame &f, double now) {
  const auto ack = hcr::decodeAck(f.payload);
  if (imu_calibration.state != "pending" || ack.request_seq != imu_calibration.request_seq) {
    ++counters.rejected;
    return {};
  }
  if (ack.result == hcr::kCalAccepted) {
    imu_calibration = message("running", "ESP đang lấy mẫu IMU; giữ robot đứng yên.");
    imu_calibration.started_at = now;
    return {};
  }
  const char *text = "Kết quả calib IMU không hợp lệ.";
  if (ack.result == hcr::kCalNotOff) text = "Calib IMU bị từ chối: chưa xác nhận motor OFF.";
  else if (ack.result == hcr::kCalFeedback) text = "Calib IMU bị từ chối: telemetry đã cũ.";
  else if (ack.result == hcr::kCalBusy) text = "ESP đang xử lý một yêu cầu calib khác.";
  else if (ack.result == hcr::kCalRefused) text = "ESP từ chối calib IMU ở trạng thái hiện tại.";
  imu_calibration = message("error", text);
  return {};
}

std::vector<hcr::Frame> HybridCore::command(const std::string &op, double now, double vx, double wz) {
  last_command_error.reset();
  if (!std::isfinite(vx) || !std::isfinite(wz)) throw std::invalid_argument("nonfinite command");
  if (op != "ARM" && op != "DISARM" && op != "RESET" && op != "VELOCITY" && op != "RESET_ENCODERS" &&
      op != "CAL_IMU") {
    ++counters.rejected;
    return {status()};
  }
  const std::uint32_t flags = last_telemetry_flags.value_or(0);
  // Reset encoder and IMU calibration need a fresh, healthy, disarmed link.
  const bool disarmed_link =
      link && (mode == Mode::Disarmed || mode == Mode::Fault) &&
      !(mode == Mode::Fault && fault != kInvalidTelemetry) && encoder_reset.state != "pending" &&
      imu_calibration.state != "pending" && imu_calibration.state != "running" && encoder_source &&
      now - encoder_source->second >= 0 && now - encoder_source->second <= .03;
  if (op == "RESET_ENCODERS") {
    if (!disarmed_link || !(flags & hcr::kEncoderOk) || (flags & (hcr::kEstop | hcr::kDriverFault))) {
      ++counters.rejected;
      last_command_error = "Reset encoder: Disable trước, cần feedback encoder mới và không có lỗi driver/E-stop.";
      return {status()};
    }
    hcr::Frame request = frame(hcr::kEncoderHome, hcr::pair(epoch, encoder_source->first));
    encoder_reset = message("pending", "Đang chờ ESP xác nhận chụp HOME; không tự bật motor.");
    encoder_reset.request_seq = request.seq;
    encoder_reset.sent_at = now;
    return {std::move(request)};
  }
  if (op == "CAL_IMU") {
    if (!disarmed_link || !(flags & hcr::kEncoderOk) || (flags & (hcr::kEstop | hcr::kDriverFault))) {
      ++counters.rejected;
      last_command_error = "Calib IMU: cần link/telemetry mới, Disable, encoder hợp lệ và không có E-stop/lỗi driver.";
      return {status()};
    }
    hcr::Frame request = frame(hcr::kCalImu, hcr::pair(epoch, encoder_source->first));
    imu_calibration = message("pending", "Đang chờ ESP nhận yêu cầu calib IMU.");
    imu_calibration.request_seq = request.seq;
    imu_calibration.sent_at = now;
    return {std::move(request)};
  }
  if (op == "RESET") {
    hcr::Frame disabled = output(rx_seq.value_or(0));
    newEpoch();
    return {std::move(disabled), hello(), status()};
  }
  if (op == "DISARM") {
    // Faults remain latched until an explicit RESET.
    if (mode != Mode::Fault) {
      mode = Mode::Disarmed;
      policy.reset();
      logic.reset();
    }
    velocity = {};
    return {output(rx_seq.value_or(0)), status()};
  }
  if (op == "ARM" && (encoder_reset.state == "pending" || imu_calibration.state == "pending" ||
                      imu_calibration.state == "running")) {
    ++counters.rejected;
    last_command_error = "ARM: đang chờ ESP hoàn tất calib IMU hoặc chụp HOME";
    return {status()};
  }
  if (op == "ARM" && mode == Mode::Disarmed) {
    if (!last_rx || now - *last_rx > config.telemetry_timeout_ms / 1000) {
      ++counters.rejected;
      last_command_error = "ARM: chưa có telemetry hợp lệ đủ mới (giới hạn 30 ms)";
      return {status()};
    }
    for (unsigned i = 0; i < 4; ++i)
      if (std::fabs(last_t->q[i] - config.home_joints[i]) > config.home_tolerance_rad) {
        ++counters.rejected;
        last_command_error = "ARM: góc khớp lệch HOME quá giới hạn 0,15 rad";
        return {status()};
      }
    policy.reset();
    start_stamp.reset(); last_infer_stamp.reset(); next_infer_stamp.reset();
    first_target_seq.reset(); last_ack.reset(); last_ack_rx.reset();
    sent_targets.clear(); sent_target_at.clear(); ack_latency_ms.clear();
    action = {};
    velocity = {};
    support = false;
    phase = "Idle";
    handoff_us.reset();
    mode = Mode::Home;
    return {status()};
  }
  if (op == "VELOCITY" && controlActive()) {
    if (std::fabs(vx) > 2 || std::fabs(wz) > kPi) throw std::invalid_argument("command outside trained range");
    velocity = {vx, wz};
    return {status()};
  }
  ++counters.rejected;
  last_command_error = op + ": trạng thái " + modeName(mode) + ", lỗi " + std::to_string(fault) + ": " +
                       (reason.empty() ? "lệnh không phù hợp trạng thái hiện tại" : reason);
  return {status()};
}

std::vector<hcr::Frame> HybridCore::estop() { return failWithStatus("driver_fault", "operator E-stop"); }

std::vector<hcr::Frame> HybridCore::telemetry(const hcr::Frame &f, double now) {
  const hcr::Telemetry t = hcr::Telemetry::decode(f.payload);
  const std::int64_t stamp = static_cast<std::int64_t>(f.stamp);
  last_telemetry_flags = t.flags;
  encoder_source = std::make_pair(f.seq, now);
  if (imu_calibration.state == "running" && (t.flags & hcr::kImuOk))
    imu_calibration = message("done", "IMU đã hiệu chuẩn và đang hợp lệ.");
  if (t.flags & (hcr::kEstop | hcr::kDriverFault)) return failWithStatus("driver_fault");
  if ((t.flags & 7) != 7) throw std::invalid_argument("IMU/encoder/calibration validity missing");
  const double norm = std::sqrt(t.gravity[0] * t.gravity[0] + t.gravity[1] * t.gravity[1] + t.gravity[2] * t.gravity[2]);
  if (!(0.9 <= norm && norm <= 1.1)) throw std::invalid_argument("gravity must be a body-frame unit vector");
  const double timeout_us = config.telemetry_timeout_ms * 1000;
  if (last_stamp) {
    const std::int64_t dt = stamp - *last_stamp;
    if (dt <= 0 || static_cast<double>(dt) > timeout_us) return failWithStatus("clock_gap");
  }
  // Clock offset estimated without assuming synchronized MCU/Linux clocks.
  const double offset = now * 1e6 - static_cast<double>(stamp);
  offset_us = offset_us ? std::min(offset, *offset_us) : offset;
  if (offset - *offset_us > timeout_us) return failWithStatus("clock_gap", "queued telemetry exceeds age limit");
  last_stamp = stamp;
  last_rx = now;
  last_t = t;
  ++counters.telemetry;
  if (mode == Mode::Disarmed || mode == Mode::Fault || mode == Mode::WaitLink) return {output(f.seq)};
  if (!start_stamp) {
    start_stamp = stamp;
    next_infer_stamp = stamp + std::llround(config.warmup_start_s * 1e6);
    last_ack_rx = now;
  }
  const double elapsed = static_cast<double>(stamp - *start_stamp) / 1e6;
  if (first_target_seq) {
    if (auto detail = targetAckTimeoutDetail(t.applied_seq, now)) return failWithStatus("ack_timeout", *detail);
  }
  if (elapsed >= config.warmup_start_s && !(t.flags & hcr::kTargetValid)) return failWithStatus("missing_applied_target");
  // The simulator stops on a fall; keep that behaviour instead of automatic rearming.
  if (mode == Mode::Model && -t.gravity[2] / norm < std::cos(70.0 * kPi / 180.0)) return failWithStatus("fall");
  const bool due = stamp >= *next_infer_stamp;
  if (elapsed >= config.warmup_start_s && due) {
    if (mode != Mode::Model) action = encodeTargets(t.applied_target);
    const bool handoff_ready = elapsed >= config.handoff_earliest_s && support &&
                               policy.steps() >= config.warmup_steps && mode != Mode::Model;
    Action prediction;
    try {
      prediction = policy.step(observation(t, action, velocity));
    } catch (const std::exception &error) {
      return failWithStatus("policy_fault", error.what());
    }
    ++counters.inferences;
    inference_max_ms = std::max(inference_max_ms, policy.lastMs());
    last_infer_stamp = stamp;
    // Keep an absolute 100 Hz schedule despite telemetry timestamp jitter.
    *next_infer_stamp += 10000;
    while (*next_infer_stamp <= stamp) *next_infer_stamp += 10000;
    if (policy.lastMs() > config.inference_deadline_ms) {
      ++counters.deadline_misses;
      ++consecutive_misses;
      if (policy.lastMs() > config.inference_hard_deadline_ms ||
          consecutive_misses >= config.max_consecutive_deadline_misses) {
        const double native = policy.lastNativeMs();
        return failWithStatus("policy_deadline", "inference " + fixed(policy.lastMs(), 3) + " ms; native " +
                                                     (native < 0 ? std::string("None") : fixed(native, 3)) + " ms");
      }
      // Discard a late prediction, retain the last actually commanded action. The GRU has
      // observed this sample; do not roll its memory backwards.
    } else {
      consecutive_misses = 0;
      if (handoff_ready) {
        mode = Mode::Model;
        handoff_us = stamp;
      }
      if (mode == Mode::Model) {
        action = prediction;
        target = decodeActions(prediction);
      }
    }
  }
  if (elapsed < config.home_hold_s) {
    mode = Mode::Home;
    const auto &h = config.home_joints;
    target = {h[0], h[1], h[0] + h[2], h[1] + h[3], 0.0, 0.0};
    return {output(f.seq, hcr::kHomePd, 3, target, {}, now)};
  }
  if (mode == Mode::Model) return {output(f.seq, hcr::kPolicyPd, 3, target, {}, now)};
  try {
    const LogicResult r = logic.step(elapsed, t);
    support = r.support;
    target = r.target;
    phase = r.phase >= 0 && r.phase < 6 ? kPhases[r.phase] : "Unknown";
    mode = elapsed >= config.warmup_start_s ? Mode::Warmup : Mode::Standup;
    return {output(f.seq, hcr::kRecovery, static_cast<std::uint16_t>(r.flags), target, r.ff, now)};
  } catch (const std::exception &error) {
    return failWithStatus("logic_fault", error.what());
  }
}

Json HybridCore::snapshot(std::optional<double> now_in) {
  const double now = now_in ? *now_in : monotonicS();
  Json j = Json::object();
  if (mcu_diagnostics) {
    Json d = Json::object();
    for (unsigned i = 0; i < hcr::kDiagWords; ++i) d.set(hcr::kDiagFields[i], mcu_diagnostics->words[i]);
    d.set("transport_age_s", mcu_diagnostics->transport_age_s);
    d.set("age_s", std::max(0.0, now - *mcu_diagnostics_rx));
    j.set("mcu_diagnostics", d);
  } else {
    j.set("mcu_diagnostics", Json());
  }
  Json ack = Json::object();
  ack.set("window_ms", targetAckWindowMs(now));
  ack.set("age_ms", last_ack_rx ? Json(std::max(0.0, 1000 * (now - *last_ack_rx))) : Json());
  ack.set("latency_samples", static_cast<std::uint64_t>(ack_latency_ms.size()));
  ack.set("latency_ms_recent_max",
          ack_latency_ms.empty() ? 0.0 : *std::max_element(ack_latency_ms.begin(), ack_latency_ms.end()));
  j.set("ack_diagnostics", ack);
  j.set("encoder_reset", encoder_reset.toJson());
  j.set("imu_calibration", imu_calibration.toJson());
  j.set("epoch", epoch);
  j.set("mode", modeName(mode));
  j.set("phase", phase);
  j.set("fault_code", fault);
  j.set("reason", reason);
  j.set("policy_steps", policy.steps());
  j.set("handoff_us", handoff_us ? Json(*handoff_us) : Json());
  j.set("inference_max_ms", inference_max_ms);
  j.set("consecutive_deadline_misses", consecutive_misses);
  Json c = Json::object();
  c.set("telemetry", counters.telemetry);
  c.set("telemetry_received", counters.telemetry_received);
  c.set("rejected", counters.rejected);
  c.set("inferences", counters.inferences);
  c.set("deadline_misses", counters.deadline_misses);
  j.set("counters", c);
  j.set("link", link);
  j.set("last_telemetry_flags", last_telemetry_flags ? Json(*last_telemetry_flags) : Json());
  return j;
}

}  // namespace rb::runtime
