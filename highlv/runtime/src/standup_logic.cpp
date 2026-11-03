#include "rb/runtime/logic.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>

#include "rb/recovery/standup_adapter.hpp"
#include "rb/runtime/json.hpp"

namespace rb::runtime {

namespace {
constexpr const char *kFailures[] = {"None", "InvalidInput", "PitchLimit", "Timeout", "HipTracking", "FloorConstraint"};
constexpr const char *kRequestFields[18] = {"t", "pitch", "gyroY", "gyroZ", "accelX", "accelZ",
    "hipL", "hipR", "kneeL", "kneeR", "wheelPosL", "wheelPosR", "hipRateL", "hipRateR",
    "kneeRateL", "kneeRateR", "wheelRateL", "wheelRateR"};
constexpr const char *kReplyFields[13] = {"support", "phase", "enables", "hipL_ref", "hipR_ref",
    "crankL_ref", "crankR_ref", "wheelL_cmd", "wheelR_cmd", "ffHipL", "ffHipR", "ffKneeL", "ffKneeR"};
constexpr std::size_t kTraceSamples = 400;

std::string round4(double v) {
  char buf[40];
  auto r = std::to_chars(buf, buf + sizeof buf, std::round(v * 1e4) / 1e4);
  return std::string(buf, r.ptr);
}
std::string list(const std::array<double, 24> &v, int from, int count) {
  std::string out = "[";
  for (int i = 0; i < count; ++i) out += (i ? ", " : "") + round4(v[from + i]);
  return out + "]";
}
}  // namespace

std::string faultMessage(double code, const std::array<double, 24> &v) {
  const bool named = std::isfinite(code) && code >= 0 && code < 6 && code == std::floor(code);
  const char *name = named ? kFailures[static_cast<int>(code)] : "unknown";
  char head[64];
  std::snprintf(head, sizeof head, "%g", code);
  return std::string("stand-up fault ") + name + " (code " + head + ") t=" + round4(v[0]) +
         " pitch=" + round4(v[1]) + " gyroY=" + round4(v[2]) + " gyroZ=" + round4(v[3]) +
         " accel=[" + round4(v[4]) + "," + round4(v[5]) + "] hip=" + list(v, 6, 2) +
         " knee=" + list(v, 8, 2) + " wheel_pos=" + list(v, 10, 2) + " hip_rate=" + list(v, 12, 2) +
         " knee_rate=" + list(v, 14, 2) + " wheel_rate=" + list(v, 16, 2);
}

struct StandupLogic::Impl {
  std::string trace_file;
  std::unique_ptr<StandupAdapter> adapter;
  struct Sample { std::array<double, 18> in; std::array<double, 13> out; };
  std::deque<Sample> trace;

  void dump(double code) {
    try {
      Json samples = Json::array();
      for (const auto &s : trace) {
        Json in = Json::object(), out = Json::object();
        for (int i = 0; i < 18; ++i) in.set(kRequestFields[i], s.in[i]);
        for (int i = 0; i < 13; ++i) out.set(kReplyFields[i], s.out[i]);
        Json row = Json::object();
        row.set("in", in);
        row.set("out", out);
        samples.push(row);
      }
      Json doc = Json::object();
      doc.set("failure_code", code);
      doc.set("samples", samples);
      if (FILE *f = std::fopen(trace_file.c_str(), "w")) {
        const std::string text = doc.dump();
        std::fwrite(text.data(), 1, text.size(), f);
        std::fclose(f);
      }
    } catch (...) {
      // post-mortem data must never change the fault path
    }
  }
};

StandupLogic::StandupLogic(std::string trace_file) : impl_(new Impl) {
  impl_->trace_file = std::move(trace_file);
  // Gain-table solves are boot work, before UART/ARM is served.
  impl_->adapter = std::make_unique<StandupAdapter>();
}
StandupLogic::~StandupLogic() { delete impl_; }

void StandupLogic::reset() { impl_->adapter = std::make_unique<StandupAdapter>(); }

LogicResult StandupLogic::step(double elapsed, const hcr::Telemetry &t) {
  const double pitch = std::atan2(t.gravity[0], std::hypot(t.gravity[1], t.gravity[2]));
  std::array<double, 24> s{};
  s[0] = elapsed;
  s[1] = pitch;
  s[2] = t.gyro[1];
  s[3] = t.gyro[2];
  s[4] = t.accel[0];
  s[5] = t.accel[2];
  for (int i = 0; i < 6; ++i) {
    s[6 + i] = t.q[i];
    s[12 + i] = t.qd[i];
  }
  s[18] = t.hip_torque[0];
  s[19] = t.hip_torque[1];
  s[20] = 1.0;
  s[21] = 1.0;
  s[22] = (t.flags & hcr::kTorqueValid) ? 1.0 : 0.0;
  s[23] = 0.0;

  std::array<double, 14> r{};
  try {
    bool valid = true;
    for (double v : s) valid = valid && std::isfinite(v);
    if (!valid) r[0] = -1;
    else impl_->adapter->step(s, r);
  } catch (...) {
    r[0] = -1;
  }
  Impl::Sample sample;
  std::copy(s.begin(), s.begin() + 18, sample.in.begin());
  std::copy(r.begin(), r.begin() + 13, sample.out.begin());
  impl_->trace.push_back(sample);
  if (impl_->trace.size() > kTraceSamples) impl_->trace.pop_front();
  bool finite = true;
  for (double v : r) finite = finite && std::isfinite(v);
  if (!finite || r[0] < 0) {
    impl_->dump(r[1]);
    throw std::runtime_error(faultMessage(r[1], s));
  }
  LogicResult out;
  out.support = r[0] != 0;
  out.phase = static_cast<int>(r[1]);
  out.flags = static_cast<int>(r[2]);
  for (int i = 0; i < 6; ++i) out.target[i] = r[3 + i];
  for (int i = 0; i < 4; ++i) out.ff[i] = r[9 + i];
  return out;
}

}  // namespace rb::runtime
