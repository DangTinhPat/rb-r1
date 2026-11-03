// Adapted from lumi-quantum 7b86053c89620484a3c99e869cce3c23dd39d37f; see PROVENANCE.md.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

// Wire protocol of the motor drivers: one-wire half-duplex UART, 1 Mbaud, Dynamixel-like packets. Pure and
// allocation-free; the same code is used by the firmware, the host tests and the simulator's fake drivers.
//
//   request: FF FF ID LEN INST PARAMS... CHK        LEN = params + 2, CHK = ~(ID + LEN + INST + PARAMS)
//   joint reply: FF FF ID LEN STATUS POS_L POS_H PRE_LEN PRE_INST PRE_P1 DATA... CHK
//   wheel reply: FF FF ID LEN STATUS PRE_LEN PRE_INST PRE_P1 DATA... CHK
//   (PRE_* echo LEN, INST and the first parameter of the request.)
//
// Two device families share a bus:
//   - joint driver (hip, knee): MIT control, position reply, 16384 counts per output revolution;
//   - Kpower BL4012 wheel: speed control and a 10-bit present-position reply.
// See examples/check_motor/docs for the vendor protocol and its errata (the MIT example checksum there is 0x69).
namespace gateway::hal::motor_frame {

constexpr float kTwoPi = 6.28318530717958647692F;

constexpr std::uint8_t kInsPing = 0x01;
constexpr std::uint8_t kInsRead = 0x02;
constexpr std::uint8_t kInsWrite = 0x03;
constexpr std::uint8_t kBroadcastId = 0xFE;

// Registers shared by both families.
constexpr std::uint8_t kRegTorqueEnable = 0x18;

// Joint driver registers.
constexpr std::uint8_t kRegResponseLevel = 0x10;  // must be 2: reply to every command
constexpr std::uint8_t kRegMitBlock = 0x1A;       // 10 bytes: Kp, Kd, position, speed, torque
constexpr std::uint8_t kRegControlMode = 0x2D;
constexpr std::uint8_t kRegGainLimits = 0x32;     // Kp_max (u16), Kd_max (u16)
constexpr std::uint8_t kModeMit = 3;
constexpr std::uint8_t kMitBlockLen = 10;
constexpr float kCountsPerRev = 16384.0F;

// Reply STATUS bits that mean the driver cut torque by itself.
constexpr std::uint8_t kStatusOverheat = 0x04;
constexpr std::uint8_t kStatusOverload = 0x20;

// Wheel registers.
constexpr std::uint8_t kWheelSpeedReg = 0x20;    // int16 RPM, signed
constexpr std::uint8_t kWheelPresentReg = 0x24;  // position u16 (0..1023 per turn), speed s16, load s16
constexpr std::uint8_t kWheelPresentLen = 6;
// raw = RPM. 1000 rpm is above the BL4012 no-load speed of 950 rpm at 12 V; the rated 400 rpm holds only at the full
// 0.13 N.m.
constexpr std::int16_t kWheelRawSpeedLimit = 1000;

enum class Family { Joint, Wheel };

// ---------------------------------------------------------------------------------------------------------------
// Unit conversions
// ---------------------------------------------------------------------------------------------------------------
constexpr float countsToRad(float counts) { return counts * kTwoPi / kCountsPerRev; }
constexpr float radToCounts(float rad) { return rad * kCountsPerRev / kTwoPi; }

inline std::int16_t saturate16(float v) {
  if (!std::isfinite(v)) return 0;
  return static_cast<std::int16_t>(std::lround(std::clamp(v, -32768.0F, 32767.0F)));
}

// Gains travel as 0.001 units in a u16 and are clamped to the limits the driver reports (kRegGainLimits).
inline std::uint16_t gainRaw(float gain, std::uint16_t raw_max) {
  if (!std::isfinite(gain)) return 0;
  const long raw = std::lround(std::max(gain, 0.0F) * 1000.0F);
  return static_cast<std::uint16_t>(std::min<long>(raw, raw_max));
}

inline std::int16_t wheelRadPerSecToRawSpeed(float rad_s) {
  if (!std::isfinite(rad_s)) return 0;
  float rpm = rad_s * 60.0F / kTwoPi;
  rpm = std::clamp(rpm, static_cast<float>(-kWheelRawSpeedLimit), static_cast<float>(kWheelRawSpeedLimit));
  return static_cast<std::int16_t>(rpm + (rpm >= 0.0F ? 0.5F : -0.5F));
}
inline float wheelRawSpeedToRadPerSec(std::int16_t raw) { return static_cast<float>(raw) * kTwoPi / 60.0F; }
inline std::uint16_t wheelRadToRawPosition(float rad) {
  if (!std::isfinite(rad)) return 0;
  float wrapped = std::fmod(rad, kTwoPi);
  if (wrapped < 0.0F) wrapped += kTwoPi;
  return static_cast<std::uint16_t>(wrapped * 1024.0F / kTwoPi);
}
// Single-turn position (0..1023 = 0..360 deg), NOT unwrapped: callers doing odometry unwrap it statefully.
inline float wheelRawPositionToRad(std::uint16_t raw) { return static_cast<float>(raw & 0x03FFU) * kTwoPi / 1024.0F; }

// ---------------------------------------------------------------------------------------------------------------
// Frames
// ---------------------------------------------------------------------------------------------------------------
using DxlBytes = std::array<std::uint8_t, 24>;  // largest frame: MIT write, 17 bytes

struct DxlFrame {
  DxlBytes bytes{};
  std::uint8_t len = 0;  // bytes actually used, starting at bytes[0]
  std::uint64_t stamp_us = 0;  // replies: when the bus received it (0 = unknown, the reader uses its own clock)
};

namespace detail {
// CHK = ~(sum of the bytes from ID up to the last parameter).
inline std::uint8_t checksum(const std::uint8_t* from_id, std::size_t n) {
  std::uint32_t sum = 0;
  for (std::size_t i = 0; i < n; ++i) sum += from_id[i];
  return static_cast<std::uint8_t>(~sum);
}
inline std::uint8_t dxlChecksum(std::uint8_t id, std::uint8_t len, std::uint8_t ins, const std::uint8_t* params,
                                std::uint8_t n) {
  std::uint32_t sum = static_cast<std::uint32_t>(id) + len + ins;
  for (std::uint8_t i = 0; i < n; ++i) sum += params[i];
  return static_cast<std::uint8_t>(~sum);
}
inline DxlFrame buildFrame(std::uint8_t id, std::uint8_t ins, const std::uint8_t* params, std::uint8_t n) {
  DxlFrame f;
  if (n > f.bytes.size() - 6U || (n && !params)) return f;
  const std::uint8_t len = static_cast<std::uint8_t>(n + 2);  // ins + params + chk
  f.bytes[0] = 0xFF;
  f.bytes[1] = 0xFF;
  f.bytes[2] = id;
  f.bytes[3] = len;
  f.bytes[4] = ins;
  for (std::uint8_t i = 0; i < n; ++i) f.bytes[5 + i] = params[i];
  f.bytes[5 + n] = dxlChecksum(id, len, ins, params, n);
  f.len = static_cast<std::uint8_t>(6 + n);
  return f;
}
}  // namespace detail

inline DxlFrame buildPing(std::uint8_t id) { return detail::buildFrame(id, kInsPing, nullptr, 0); }

inline DxlFrame buildRead(std::uint8_t id, std::uint8_t address, std::uint8_t length) {
  const std::uint8_t params[2] = {address, length};
  return detail::buildFrame(id, kInsRead, params, 2);
}

// Writes `length` (<= 16) bytes starting at `address`.
inline DxlFrame buildWrite(std::uint8_t id, std::uint8_t address, const std::uint8_t* data, std::uint8_t length) {
  if (length > 16U || (length && !data)) return {};
  std::array<std::uint8_t, 17> params{};
  params[0] = address;
  for (std::uint8_t i = 0; i < length && i < 16; ++i) params[1 + i] = data[i];
  return detail::buildFrame(id, kInsWrite, params.data(), static_cast<std::uint8_t>(1 + std::min<std::uint8_t>(length, 16)));
}

inline DxlFrame buildTorqueEnable(std::uint8_t id, bool enable) {
  const std::uint8_t value = enable ? 1 : 0;
  return buildWrite(id, kRegTorqueEnable, &value, 1);
}

// --- Joint driver ---------------------------------------------------------------------------------------------
inline DxlFrame buildControlMode(std::uint8_t id, std::uint8_t mode) { return buildWrite(id, kRegControlMode, &mode, 1); }

// The block the driver stores at kRegMitBlock. Position, speed and torque are the set-points of the driver's own
// loop: torque = Kp * (pos - measured) + Kd * (speed - measured speed) + tau.
struct MitBlock {
  std::uint16_t kp = 0;   // 0.001 N.m/rad
  std::uint16_t kd = 0;   // 0.001 N.m.s/rad
  std::int16_t pos = 0;   // counts, output shaft
  std::int16_t rpm = 0;   // output RPM
  std::int16_t tau = 0;   // 0.001 N.m
};

inline DxlFrame buildMitWrite(std::uint8_t id, const MitBlock& b) {
  const auto u = [](std::int16_t v) { return static_cast<std::uint16_t>(v); };
  const std::uint8_t block[kMitBlockLen] = {
      static_cast<std::uint8_t>(b.kp & 0xFFU),    static_cast<std::uint8_t>(b.kp >> 8),
      static_cast<std::uint8_t>(b.kd & 0xFFU),    static_cast<std::uint8_t>(b.kd >> 8),
      static_cast<std::uint8_t>(u(b.pos) & 0xFFU), static_cast<std::uint8_t>(u(b.pos) >> 8),
      static_cast<std::uint8_t>(u(b.rpm) & 0xFFU), static_cast<std::uint8_t>(u(b.rpm) >> 8),
      static_cast<std::uint8_t>(u(b.tau) & 0xFFU), static_cast<std::uint8_t>(u(b.tau) >> 8)};
  return buildWrite(id, kRegMitBlock, block, kMitBlockLen);
}

// --- Wheel ----------------------------------------------------------------------------------------------------
inline DxlFrame buildMovingSpeed(std::uint8_t id, float rad_s) {
  const auto u = static_cast<std::uint16_t>(wheelRadPerSecToRawSpeed(rad_s));  // 2's complement little endian
  const std::uint8_t data[2] = {static_cast<std::uint8_t>(u & 0xFFU), static_cast<std::uint8_t>(u >> 8)};
  return buildWrite(id, kWheelSpeedReg, data, 2);
}
inline DxlFrame buildReadPresent(std::uint8_t id) { return buildRead(id, kWheelPresentReg, kWheelPresentLen); }

// ---------------------------------------------------------------------------------------------------------------
// Decoding of requests (used by the fake drivers) and of replies
// ---------------------------------------------------------------------------------------------------------------
struct DxlWrite {
  bool ok = false;
  std::uint8_t id = 0;
  std::uint8_t address = 0;
  std::array<std::uint8_t, 16> data{};
  std::uint8_t data_len = 0;
};

struct DxlRead {
  bool ok = false;
  std::uint8_t id = 0;
  std::uint8_t address = 0;
  std::uint8_t length = 0;
};

inline DxlWrite parseDxlWrite(const DxlFrame& frame) {
  DxlWrite out;
  if (frame.len < 7 || frame.len > frame.bytes.size() || frame.bytes[0] != 0xFF || frame.bytes[1] != 0xFF ||
      frame.bytes[4] != kInsWrite || static_cast<std::size_t>(frame.bytes[3]) + 4U != frame.len) {
    return out;
  }
  const std::uint8_t param_n = static_cast<std::uint8_t>(frame.bytes[3] - 2U);
  if (param_n < 1 || param_n - 1U > out.data.size() || static_cast<std::size_t>(6U + param_n) != frame.len) return out;
  const std::uint8_t expect =
      detail::dxlChecksum(frame.bytes[2], frame.bytes[3], frame.bytes[4], frame.bytes.data() + 5, param_n);
  if (frame.bytes[frame.len - 1U] != expect) return out;
  out.ok = true;
  out.id = frame.bytes[2];
  out.address = frame.bytes[5];
  out.data_len = static_cast<std::uint8_t>(param_n - 1U);
  for (std::uint8_t i = 0; i < out.data_len; ++i) out.data[i] = frame.bytes[6U + i];
  return out;
}

inline DxlRead parseDxlRead(const DxlFrame& frame) {
  DxlRead out;
  if (frame.len != 8 || frame.bytes[0] != 0xFF || frame.bytes[1] != 0xFF || frame.bytes[4] != kInsRead ||
      frame.bytes[3] != 4) {
    return out;
  }
  const std::uint8_t params[2] = {frame.bytes[5], frame.bytes[6]};
  if (frame.bytes[7] != detail::dxlChecksum(frame.bytes[2], frame.bytes[3], frame.bytes[4], params, 2)) return out;
  out.ok = true;
  out.id = frame.bytes[2];
  out.address = frame.bytes[5];
  out.length = frame.bytes[6];
  return out;
}

// The MIT block of a write to kRegMitBlock; ok is false for any other frame.
struct MitWrite {
  bool ok = false;
  std::uint8_t id = 0;
  MitBlock block{};
};
inline MitWrite parseMitWrite(const DxlFrame& frame) {
  MitWrite out;
  const DxlWrite w = parseDxlWrite(frame);
  if (!w.ok || w.address != kRegMitBlock || w.data_len != kMitBlockLen) return out;
  const auto u16 = [&](std::size_t i) {
    return static_cast<std::uint16_t>(w.data[i] | (static_cast<std::uint16_t>(w.data[i + 1]) << 8));
  };
  out.ok = true;
  out.id = w.id;
  out.block.kp = u16(0);
  out.block.kd = u16(2);
  out.block.pos = static_cast<std::int16_t>(u16(4));
  out.block.rpm = static_cast<std::int16_t>(u16(6));
  out.block.tau = static_cast<std::int16_t>(u16(8));
  return out;
}

// Number of bytes a reply to `request` has on the wire, 0 when none is expected (broadcast).
inline std::size_t expectedReplyLength(const DxlFrame& request, Family family) {
  if (request.len < 6 || request.bytes[2] == kBroadcastId) return 0;
  const std::size_t data_len = request.bytes[4] == kInsRead && request.len >= 8 ? request.bytes[6] : 0;
  const std::size_t echo_at = family == Family::Joint ? 7 : 5;  // index of PRE_LEN
  return data_len + echo_at + 4;
}

struct JointReply {
  bool ok = false;
  std::uint8_t id = 0;
  std::uint8_t status = 0;
  std::int16_t pos = 0;  // output shaft, kCountsPerRev per turn; wraps every 2 turns
  std::uint8_t echo_len = 0, echo_ins = 0, echo_p1 = 0;  // header of the request this reply answers
  std::array<std::uint8_t, 8> data{};
  std::uint8_t data_len = 0;
};

// Validates sync, LEN and checksum of a joint reply and returns its fields.
inline JointReply parseJointReply(const std::uint8_t* bytes, std::size_t n) {
  JointReply out;
  constexpr std::size_t kEchoAt = 7;
  if (!bytes || n < kEchoAt + 4 || bytes[0] != 0xFF || bytes[1] != 0xFF || bytes[3] + 4U != n) return out;
  if (bytes[n - 1] != detail::checksum(bytes + 2, n - 3)) return out;
  const std::size_t data_len = n - (kEchoAt + 3) - 1;
  if (data_len > out.data.size()) return out;
  out.ok = true;
  out.id = bytes[2];
  out.status = bytes[4];
  out.pos = static_cast<std::int16_t>(bytes[5] | (static_cast<std::uint16_t>(bytes[6]) << 8));
  out.echo_len = bytes[kEchoAt];
  out.echo_ins = bytes[kEchoAt + 1];
  out.echo_p1 = bytes[kEchoAt + 2];
  out.data_len = static_cast<std::uint8_t>(data_len);
  for (std::size_t i = 0; i < data_len; ++i) out.data[i] = bytes[kEchoAt + 3 + i];
  return out;
}

// True when `reply` answers `request`: same driver and the echoed header of the request.
inline bool replyMatches(const JointReply& reply, const DxlFrame& request) {
  const bool has_param = request.bytes[4] == kInsRead || request.bytes[4] == kInsWrite;
  return reply.ok && reply.id == request.bytes[2] && reply.echo_len == request.bytes[3] &&
         reply.echo_ins == request.bytes[4] && reply.echo_p1 == (has_param ? request.bytes[5] : 0U);
}

// Reply a fake joint driver sends to `request`: status, current position and the data of a read.
inline DxlFrame buildJointReply(const DxlFrame& request, std::uint8_t status, std::int16_t pos,
                                const std::uint8_t* data = nullptr, std::uint8_t data_len = 0) {
  DxlFrame f;
  if (data_len > f.bytes.size() - 11U || (data_len && !data)) return f;
  const bool has_param = request.bytes[4] == kInsRead || request.bytes[4] == kInsWrite;
  const auto p = static_cast<std::uint16_t>(pos);
  std::size_t i = 0;
  f.bytes[i++] = 0xFF;
  f.bytes[i++] = 0xFF;
  f.bytes[i++] = request.bytes[2];
  f.bytes[i++] = static_cast<std::uint8_t>(data_len + 7);  // LEN = total - 4, filled in below
  f.bytes[i++] = status;
  f.bytes[i++] = static_cast<std::uint8_t>(p & 0xFFU);
  f.bytes[i++] = static_cast<std::uint8_t>(p >> 8);
  f.bytes[i++] = request.bytes[3];
  f.bytes[i++] = request.bytes[4];
  f.bytes[i++] = has_param ? request.bytes[5] : 0;
  for (std::uint8_t k = 0; k < data_len; ++k) f.bytes[i++] = data[k];
  f.bytes[3] = static_cast<std::uint8_t>(i + 1 - 4);
  f.bytes[i] = detail::checksum(f.bytes.data() + 2, i - 2);
  f.len = static_cast<std::uint8_t>(i + 1);
  return f;
}

// --- Wheel replies --------------------------------------------------------------------------------------------
struct DxlParsed {
  bool ok = false;
  std::uint8_t id = 0;
  std::uint8_t err = 0;
  std::array<std::uint8_t, kWheelPresentLen> payload{};
  std::uint8_t payload_len = 0;
};

// Parses a wheel reply, INCLUDING the driver's quirk of echoing [LEN, INS, ADDR] of the request before the data.
inline DxlParsed parseDxlResponse(const std::uint8_t* bytes, std::size_t n) {
  DxlParsed out;
  if (!bytes || n < 9 || bytes[0] != 0xFF || bytes[1] != 0xFF) return out;
  const std::uint8_t id = bytes[2];
  const std::uint8_t len = bytes[3];
  const std::uint8_t err = bytes[4];
  if (len < 5 || static_cast<std::size_t>(len) + 4 != n) return out;
  const std::uint8_t* params = bytes + 5;
  const std::uint8_t param_n = static_cast<std::uint8_t>(len - 2);
  const std::uint8_t chk_index = static_cast<std::uint8_t>(4 + len - 1);
  if (bytes[chk_index] != detail::dxlChecksum(id, len, err, params, param_n)) return out;
  if (param_n < 3) return out;
  const std::uint8_t real_n = static_cast<std::uint8_t>(param_n - 3);
  if (real_n > kWheelPresentLen) return out;
  out.ok = true;
  out.id = id;
  out.err = err;
  out.payload_len = real_n;
  for (std::uint8_t i = 0; i < real_n; ++i) out.payload[i] = params[3 + i];
  return out;
}

struct PresentData {
  bool valid = false;
  std::uint16_t position_raw = 0;
  std::int16_t speed_raw = 0;
  std::int16_t load_raw = 0;
};

// Position (u16 LE), Speed (s16 LE), Load (s16 LE), in that order.
inline PresentData parsePresentPayload(const DxlParsed& parsed) {
  PresentData out;
  if (!parsed.ok || parsed.payload_len != kWheelPresentLen) return out;
  const auto& p = parsed.payload;
  out.position_raw = static_cast<std::uint16_t>(p[0] | (static_cast<std::uint16_t>(p[1]) << 8));
  out.speed_raw = static_cast<std::int16_t>(p[2] | (static_cast<std::uint16_t>(p[3]) << 8));
  out.load_raw = static_cast<std::int16_t>(p[4] | (static_cast<std::uint16_t>(p[5]) << 8));
  out.valid = true;
  return out;
}

// Reply of a fake wheel to a present-data read, including the echo quirk.
inline DxlFrame buildPresentResponse(std::uint8_t id, std::uint16_t position_raw, std::int16_t speed_raw,
                                     std::int16_t load_raw) {
  const auto speed_u = static_cast<std::uint16_t>(speed_raw);
  const auto load_u = static_cast<std::uint16_t>(load_raw);
  const std::uint8_t params[9] = {kWheelPresentLen, kInsRead, kWheelPresentReg,
                                  static_cast<std::uint8_t>(position_raw & 0xFFU), static_cast<std::uint8_t>(position_raw >> 8),
                                  static_cast<std::uint8_t>(speed_u & 0xFFU), static_cast<std::uint8_t>(speed_u >> 8),
                                  static_cast<std::uint8_t>(load_u & 0xFFU), static_cast<std::uint8_t>(load_u >> 8)};
  return detail::buildFrame(id, 0, params, 9);
}

// Reply of a fake wheel to a write: same echo shape, no data.
inline DxlFrame buildWriteAck(std::uint8_t id, std::uint8_t written_len, std::uint8_t addr) {
  const std::uint8_t params[3] = {written_len, kInsWrite, addr};
  return detail::buildFrame(id, 0, params, 3);
}

}  // namespace gateway::hal::motor_frame
