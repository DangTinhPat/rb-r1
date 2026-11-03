// Source golden tests; see include/gateway/motor/hal/PROVENANCE.md.
#include <cmath>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <iostream>

#include "gateway/motor/hal/MotorFrameCodec.hpp"

using namespace gateway::hal::motor_frame;

namespace {
int checks = 0;
int failures = 0;
void check(bool ok, const char* msg) {
  ++checks;
  if (!ok) {
    ++failures;
    std::cerr << "FAIL " << msg << '\n';
  }
}
bool eqFrame(const DxlFrame& f, std::initializer_list<int> expect) {
  if (f.len != expect.size()) return false;
  std::size_t i = 0;
  for (int v : expect) {
    if (f.bytes[i++] != static_cast<std::uint8_t>(v)) return false;
  }
  return true;
}
}  // namespace

int main() {
  // --- Joint driver, MIT block. The vendor document prints checksum 0x64 for this frame, but the formula gives 0x69
  // (examples/check_motor/docs/PROTOCOL_ERRATA.md); 0x69 is what the driver accepts.
  {
    MitBlock block;
    block.kp = 10000;  // 10 N.m/rad
    block.kd = 36;     // 0.036 N.m.s/rad
    block.pos = 4096;
    const DxlFrame mit = buildMitWrite(1, block);
    check(eqFrame(mit, {0xFF, 0xFF, 0x01, 0x0D, 0x03, 0x1A, 0x10, 0x27, 0x24, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x69}),
          "MIT write matches the vendor example (checksum 0x69)");
    const auto back = parseMitWrite(mit);
    check(back.ok && back.id == 1 && back.block.kp == 10000 && back.block.kd == 36 && back.block.pos == 4096 &&
              back.block.rpm == 0 && back.block.tau == 0,
          "MIT write parses back at the wire boundary");
    MitBlock signed_block;
    signed_block.pos = -1234;
    signed_block.rpm = -300;
    signed_block.tau = -50;
    const auto signed_back = parseMitWrite(buildMitWrite(2, signed_block));
    check(signed_back.ok && signed_back.block.pos == -1234 && signed_back.block.rpm == -300 &&
              signed_back.block.tau == -50,
          "negative position, speed and torque survive the round trip");
    DxlFrame bad = mit;
    bad.bytes[bad.len - 1] ^= 0x01U;
    check(!parseMitWrite(bad).ok, "corrupted MIT write is rejected");
  }

  // --- Generic requests.
  check(eqFrame(buildPing(2), {0xFF, 0xFF, 0x02, 0x02, 0x01, 0xFA}), "ping frame");
  check(eqFrame(buildTorqueEnable(1, false), {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x18, 0x00, 0xDF}), "torque disable frame");
  check(eqFrame(buildTorqueEnable(2, true), {0xFF, 0xFF, 0x02, 0x04, 0x03, 0x18, 0x01, 0xDD}), "torque enable frame");
  check(eqFrame(buildControlMode(3, kModeMit), {0xFF, 0xFF, 0x03, 0x04, 0x03, 0x2D, 0x03, 0xC5}), "control mode MIT frame");
  check(eqFrame(buildRead(2, kRegGainLimits, 4), {0xFF, 0xFF, 0x02, 0x04, 0x02, 0x32, 0x04, 0xC1}), "gain limit read frame");

  // --- Conversions.
  check(std::abs(countsToRad(16384.0F) - kTwoPi) < 1e-5F, "one revolution is 16384 counts");
  check(std::abs(radToCounts(countsToRad(1234.0F)) - 1234.0F) < 1e-2F, "counts round-trip through radians");
  check(saturate16(40000.0F) == 32767 && saturate16(-40000.0F) == -32768 && saturate16(-12.4F) == -12,
        "position saturates at the int16 range");
  check(gainRaw(10.0F, 20000) == 10000 && gainRaw(0.036F, 120) == 36 && gainRaw(0.5F, 120) == 120 && gainRaw(-1.0F, 120) == 0,
        "gains scale by 1000 and clamp to the driver limit");

  // --- Joint replies: the request header is echoed and the position follows the status.
  {
    const DxlFrame request = buildMitWrite(2, MitBlock{});
    check(expectedReplyLength(request, Family::Joint) == 11, "joint write reply is 11 bytes");
    check(expectedReplyLength(buildRead(2, kRegGainLimits, 4), Family::Joint) == 15, "joint read adds the data bytes");
    check(expectedReplyLength(buildPing(2), Family::Joint) == 11, "joint ping reply is 11 bytes");
    check(expectedReplyLength(buildMovingSpeed(1, 0.0F), Family::Wheel) == 9, "wheel write reply is 9 bytes");
    check(expectedReplyLength(buildReadPresent(1), Family::Wheel) == 15, "wheel present read reply is 15 bytes");
    check(expectedReplyLength(buildTorqueEnable(kBroadcastId, false), Family::Joint) == 0, "broadcast has no reply");

    const DxlFrame reply = buildJointReply(request, 0x00, -2345);
    check(reply.len == 11, "joint reply frame length");
    const auto parsed = parseJointReply(reply.bytes.data(), reply.len);
    check(parsed.ok && parsed.id == 2 && parsed.status == 0 && parsed.pos == -2345 && parsed.data_len == 0,
          "joint reply decodes status and position");
    check(replyMatches(parsed, request), "reply matches its request");
    check(!replyMatches(parsed, buildMitWrite(3, MitBlock{})), "reply of another driver does not match");
    check(!replyMatches(parsed, buildPing(2)), "reply of another instruction does not match");

    const std::uint8_t limits[4] = {0x20, 0x4E, 0x78, 0x00};  // Kp_max 20.000, Kd_max 0.120
    const DxlFrame read = buildRead(2, kRegGainLimits, 4);
    const DxlFrame read_reply = buildJointReply(read, kStatusOverload, 100, limits, 4);
    const auto limits_parsed = parseJointReply(read_reply.bytes.data(), read_reply.len);
    check(limits_parsed.ok && limits_parsed.data_len == 4 && limits_parsed.data[0] == 0x20 &&
              limits_parsed.data[3] == 0x00 && limits_parsed.status == kStatusOverload && replyMatches(limits_parsed, read),
          "read reply carries the data and the status flags");

    // Replies of the vendor document and of the robot (golden vectors of examples/check_motor/tests): the checksum is
    // not produced by this codec, so a wrong parser checksum cannot hide behind a wrong builder.
    {
      const std::uint8_t mit_rx[] = {0xFF, 0xFF, 0x01, 0x07, 0x00, 0x00, 0x10, 0x0D, 0x03, 0x1A, 0xBD};
      const auto mit = parseJointReply(mit_rx, sizeof mit_rx);
      check(mit.ok && mit.id == 1 && mit.status == 0 && mit.pos == 4096 && mit.echo_len == 0x0D && mit.echo_ins == 0x03 &&
                mit.echo_p1 == 0x1A,
            "vendor MIT reply decodes");
      const std::uint8_t pos_rx[] = {0xFF, 0xFF, 0x01, 0x07, 0x00, 0x40, 0x1F, 0x05, 0x03, 0x1E, 0x72};
      const auto goal = parseJointReply(pos_rx, sizeof pos_rx);
      check(goal.ok && goal.pos == 8000 && goal.echo_p1 == 0x1E, "vendor position-write reply decodes");
      std::uint8_t bad[sizeof mit_rx];
      std::memcpy(bad, mit_rx, sizeof bad);
      bad[sizeof bad - 1U] ^= 0x01U;
      check(!parseJointReply(bad, sizeof bad).ok, "vendor reply with a wrong checksum is rejected");
      // The reply the codec builds for the same request is byte-identical to the vendor one.
      MitBlock zero{};
      DxlFrame built = buildJointReply(buildMitWrite(1, zero), 0x00, 4096);
      check(built.len == sizeof mit_rx && std::memcmp(built.bytes.data(), mit_rx, sizeof mit_rx) == 0,
            "built reply equals the vendor reply byte for byte");
    }

    DxlFrame corrupted = reply;
    corrupted.bytes[corrupted.len - 1] ^= 0x10U;
    check(!parseJointReply(corrupted.bytes.data(), corrupted.len).ok, "corrupted joint reply is rejected");
    check(!parseJointReply(reply.bytes.data(), reply.len - 1U).ok, "truncated joint reply is rejected");
  }

  // --- Wheel frames: examples of the Kpower BL4012 protocol document, checksums verified.
  const float rpm200 = 200.0F * kTwoPi / 60.0F;
  check(eqFrame(buildMovingSpeed(1, rpm200), {0xFF, 0xFF, 0x01, 0x05, 0x03, 0x20, 0xC8, 0x00, 0x0E}),
        "Moving Speed +200 matches the document frame");
  check(eqFrame(buildMovingSpeed(1, -rpm200), {0xFF, 0xFF, 0x01, 0x05, 0x03, 0x20, 0x38, 0xFF, 0x9F}),
        "Moving Speed -200 matches the document frame (signed little endian)");
  check(eqFrame(buildMovingSpeed(1, 0.0F), {0xFF, 0xFF, 0x01, 0x05, 0x03, 0x20, 0x00, 0x00, 0xD6}),
        "Moving Speed 0 matches the document frame");
  check(eqFrame(buildReadPresent(1), {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x24, 0x06, 0xCE}), "Read Present matches the document frame");

  check(wheelRadPerSecToRawSpeed(rpm200) == 200, "200 rpm converts to raw 200");
  check(wheelRadPerSecToRawSpeed(-rpm200) == -200, "-200 rpm converts to raw -200");
  check(wheelRadPerSecToRawSpeed(1.0e5F) == kWheelRawSpeedLimit, "overspeed clamps high");
  check(wheelRadPerSecToRawSpeed(-1.0e5F) == -kWheelRawSpeedLimit, "overspeed clamps low");
  check(std::abs(wheelRawSpeedToRadPerSec(wheelRadPerSecToRawSpeed(rpm200)) - rpm200) < 0.02F, "wheel speed round-trips");
  check(std::abs(wheelRawPositionToRad(0)) < 1e-6F, "wheel position raw 0 = 0 rad");
  check(std::abs(wheelRawPositionToRad(1023) - (kTwoPi * 1023.0F / 1024.0F)) < 1e-3F, "wheel position raw 1023 approaches 2*pi");
  check(std::abs(wheelRawPositionToRad(1024)) < 1e-6F, "wheel position wraps at 1024 (single turn)");
  check(wheelRadToRawPosition(0.0F) == 0 && wheelRadToRawPosition(kTwoPi) == 0, "wheel position encodes modulo one turn");

  {
    const DxlFrame present = buildPresentResponse(1, 512, -150, 7);
    const auto parsed = parseDxlResponse(present.bytes.data(), present.len);
    check(parsed.ok && parsed.id == 1 && parsed.err == 0 && parsed.payload_len == kWheelPresentLen,
          "present response parses and strips the 3-byte echo");
    const auto data = parsePresentPayload(parsed);
    check(data.valid && data.position_raw == 512 && data.speed_raw == -150 && data.load_raw == 7,
          "present position, speed and load round-trip");
    const DxlFrame ack = buildWriteAck(1, 3, kWheelSpeedReg);
    check(parseDxlResponse(ack.bytes.data(), ack.len).ok && parseDxlResponse(ack.bytes.data(), ack.len).payload_len == 0,
          "write ack parses with no payload");
    DxlFrame corrupted = buildPresentResponse(1, 1, 1, 1);
    corrupted.bytes[corrupted.len - 1] ^= 0xFFU;
    check(!parseDxlResponse(corrupted.bytes.data(), corrupted.len).ok, "corrupted wheel reply is rejected");
  }
  {
    const auto write = parseDxlWrite(buildMovingSpeed(1, rpm200));
    check(write.ok && write.id == 1 && write.address == kWheelSpeedReg && write.data_len == 2 && write.data[0] == 0xC8 &&
              write.data[1] == 0x00,
          "Moving Speed command parses back at the wire boundary");
    const auto read = parseDxlRead(buildReadPresent(1));
    check(read.ok && read.id == 1 && read.address == kWheelPresentReg && read.length == kWheelPresentLen,
          "present read parses back");
  }

  { // Local hardening: malformed lengths must never underflow or read outside the frame.
    check(!parseDxlResponse(nullptr, 15).ok && !parseJointReply(nullptr, 11).ok, "null reply rejected");
    std::array<std::uint8_t,32> bytes{};
    bytes[0]=bytes[1]=0xff;bytes[2]=1;
    for (unsigned len=0;len<256;++len) for (unsigned size=0;size<=bytes.size();++size) {
      bytes[3]=len;
      (void)parseDxlResponse(bytes.data(),size);
      (void)parseJointReply(bytes.data(),size);
    }
    const auto valid=buildWriteAck(1,5,kWheelSpeedReg);
    std::copy_n(valid.bytes.begin(),valid.len,bytes.begin());
    check(!parseDxlResponse(bytes.data(),valid.len+1).ok,"trailing byte rejected");
    bytes[3]=0;check(!parseDxlResponse(bytes.data(),valid.len).ok,"LEN underflow rejected");
    check(buildWrite(1,0x18,nullptr,1).len==0,"null write rejected");
    check(buildWrite(1,0x18,bytes.data(),17).len==0,"oversize write rejected without truncation");
    check(detail::buildFrame(1,3,bytes.data(),19).len==0,"oversize raw frame rejected");
    check(buildJointReply(buildPing(2),0,0,nullptr,1).len==0,"null fake reply data rejected");
    check(buildJointReply(buildPing(2),0,0,bytes.data(),255).len==0,"oversize fake reply rejected");
  }

  std::cout << checks - failures << '/' << checks << " motor frame codec checks passed\n";
  return failures ? 1 : 0;
}
