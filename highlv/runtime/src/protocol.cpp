#include "rb/runtime/protocol.hpp"

#include <cmath>
#include <stdexcept>

namespace rb::runtime::hcr {

const char *const kDiagFields[kDiagWords] = {
    "epoch", "version", "local_fault", "motor_error", "link_rx_age_us", "target_rx_age_us",
    "pending_age_us", "pending_seq", "superseded", "cycle_l_us", "cycle_r_us", "io_l_us",
    "io_r_us", "cycle_l_max_us", "cycle_r_max_us", "io_l_max_us", "io_r_max_us", "error_l",
    "error_r", "stage_l", "stage_r", "flags", "rx_queue_max_us", "applied_latency_max_us"};

std::vector<std::uint8_t> Frame::encode() const {
  if (payload.size() > HCR_MAX_PAYLOAD) throw std::length_error("payload too large");
  std::vector<std::uint8_t> out(payload.size() + 24);
  std::size_t n = hcr_encode(out.data(), out.size(), kind, seq, stamp, payload.data(),
                             static_cast<std::uint16_t>(payload.size()));
  if (n != out.size()) throw std::length_error("frame encode failed");
  return out;
}

std::vector<Frame> Parser::feed(const std::uint8_t *data, std::size_t size) {
  std::vector<Frame> frames;
  HcrFrame raw;
  for (std::size_t i = 0; i < size; ++i)
    if (hcr_feed(&state_, data[i], &raw))
      frames.push_back({raw.type, raw.seq, raw.stamp_us, {raw.payload, raw.payload + raw.size}});
  return frames;
}

namespace {
void putU32(std::vector<std::uint8_t> &v, std::size_t at, std::uint32_t x) { hcr_put32(v.data() + at, x); }
void checkSize(const std::vector<std::uint8_t> &p, std::size_t n, const char *what) {
  if (p.size() != n) throw std::invalid_argument(std::string(what) + ": wrong payload size");
}
}  // namespace

Telemetry Telemetry::decode(const std::vector<std::uint8_t> &p) {
  checkSize(p, kTelemetrySize, "telemetry");
  Telemetry t;
  t.epoch = hcr_u32(p.data());
  t.flags = hcr_u32(p.data() + 4);
  t.applied_seq = hcr_u32(p.data() + 8);
  double f[29];
  for (unsigned i = 0; i < 29; ++i) {
    f[i] = hcr_float(p.data() + 12 + 4 * i);
    if (!std::isfinite(f[i])) throw std::invalid_argument("nonfinite telemetry");
  }
  for (unsigned i = 0; i < 3; ++i) {
    t.gyro[i] = f[HCR_GYRO + i];
    t.gravity[i] = f[HCR_GRAVITY + i];
    t.accel[i] = f[HCR_ACCEL + i];
  }
  for (unsigned i = 0; i < 6; ++i) {
    t.q[i] = f[HCR_Q + i];
    t.qd[i] = f[HCR_QD + i];
    t.applied_target[i] = f[HCR_APPLIED_TARGET + i];
  }
  t.hip_torque = {f[HCR_HIP_TORQUE], f[HCR_HIP_TORQUE + 1]};
  return t;
}

std::vector<std::uint8_t> Telemetry::encode() const {
  std::vector<std::uint8_t> p(kTelemetrySize);
  putU32(p, 0, epoch);
  putU32(p, 4, flags);
  putU32(p, 8, applied_seq);
  float f[29];
  for (unsigned i = 0; i < 3; ++i) {
    f[HCR_GYRO + i] = static_cast<float>(gyro[i]);
    f[HCR_GRAVITY + i] = static_cast<float>(gravity[i]);
    f[HCR_ACCEL + i] = static_cast<float>(accel[i]);
  }
  for (unsigned i = 0; i < 6; ++i) {
    f[HCR_Q + i] = static_cast<float>(q[i]);
    f[HCR_QD + i] = static_cast<float>(qd[i]);
    f[HCR_APPLIED_TARGET + i] = static_cast<float>(applied_target[i]);
  }
  f[HCR_HIP_TORQUE] = static_cast<float>(hip_torque[0]);
  f[HCR_HIP_TORQUE + 1] = static_cast<float>(hip_torque[1]);
  for (unsigned i = 0; i < 29; ++i) hcr_putfloat(p.data() + 12 + 4 * i, f[i]);
  return p;
}

std::vector<std::uint8_t> helloData(std::uint32_t epoch) {
  std::vector<std::uint8_t> p(kHelloSize);
  putU32(p, 0, epoch);
  hcr_put16(p.data() + 4, 200);
  hcr_put16(p.data() + 6, 100);
  putU32(p, 8, kSchema);
  return p;
}

std::vector<std::uint8_t> statusData(std::uint32_t epoch, std::uint32_t mode, std::uint32_t fault,
                                     std::uint32_t rejected) {
  std::vector<std::uint8_t> p(16);
  putU32(p, 0, epoch);
  putU32(p, 4, mode);
  putU32(p, 8, fault);
  putU32(p, 12, rejected);
  return p;
}

std::vector<std::uint8_t> targetData(std::uint32_t epoch, std::uint32_t seq, std::uint16_t profile,
                                     std::uint16_t flags, const std::array<double, 6> &target,
                                     const std::array<double, 4> &ff) {
  std::vector<std::uint8_t> p(kTargetSize);
  putU32(p, 0, epoch);
  putU32(p, 4, seq);
  hcr_put16(p.data() + 8, profile);
  hcr_put16(p.data() + 10, flags);
  for (unsigned i = 0; i < 6; ++i) hcr_putfloat(p.data() + 12 + 4 * i, static_cast<float>(target[i]));
  for (unsigned i = 0; i < 4; ++i) hcr_putfloat(p.data() + 36 + 4 * i, static_cast<float>(ff[i]));
  return p;
}

std::vector<std::uint8_t> pair(std::uint32_t epoch, std::uint32_t source_seq) {
  std::vector<std::uint8_t> p(8);
  putU32(p, 0, epoch);
  putU32(p, 4, source_seq);
  return p;
}

std::vector<std::uint8_t> ack(std::uint32_t epoch, std::uint32_t request_seq, std::uint32_t result) {
  std::vector<std::uint8_t> p(12);
  putU32(p, 0, epoch);
  putU32(p, 4, request_seq);
  putU32(p, 8, result);
  return p;
}

HelloData decodeHello(const std::vector<std::uint8_t> &p) {
  checkSize(p, kHelloSize, "hello");
  return {hcr_u32(p.data()), hcr_u16(p.data() + 4), hcr_u16(p.data() + 6), hcr_u32(p.data() + 8)};
}

AckData decodeAck(const std::vector<std::uint8_t> &p) {
  checkSize(p, 12, "ack");
  return {hcr_u32(p.data()), hcr_u32(p.data() + 4), hcr_u32(p.data() + 8)};
}

std::array<std::uint32_t, kDiagWords> decodeDiagnostics(const std::vector<std::uint8_t> &p) {
  checkSize(p, kDiagWords * 4, "execution diagnostics");
  std::array<std::uint32_t, kDiagWords> v{};
  for (unsigned i = 0; i < kDiagWords; ++i) v[i] = hcr_u32(p.data() + 4 * i);
  if (v[HCR_DIAG_VERSION] != HCR_EXECUTION_DIAG_VERSION)
    throw std::invalid_argument("unsupported execution diagnostics version");
  return v;
}

}  // namespace rb::runtime::hcr
