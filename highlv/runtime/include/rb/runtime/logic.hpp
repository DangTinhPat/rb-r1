// Stand-up / recovery logic used by the hybrid core before the policy takes over.
#pragma once
#include <array>
#include <deque>
#include <string>

#include "rb/runtime/protocol.hpp"

namespace rb::runtime {

struct LogicResult {
  bool support = false;
  int phase = 0;  // StandUp::Phase
  int flags = 0;  // HCR LEGS_ENABLE | WHEELS_ENABLE
  std::array<double, 6> target{};
  std::array<double, 4> ff{};  // hipL, hipR, kneeL, kneeR rate feedforward
};

class Logic {
 public:
  virtual ~Logic() = default;
  virtual void reset() = 0;
  // Throws std::exception on a recovery fault; the core latches logic_fault.
  virtual LogicResult step(double elapsed_s, const hcr::Telemetry &t) = 0;
};

// Failure naming for the stand-up controller, in StandUp::Failure order.
std::string faultMessage(double code, const std::array<double, 24> &values);

// Runs rb::recovery in-process (no helper pipe). On a fault the last 400 samples
// (2 s at 200 Hz) are written to `trace_file` for post-mortem.
class StandupLogic final : public Logic {
 public:
  static constexpr const char *kTraceFile = "/tmp/rb-standup-trace.json";
  explicit StandupLogic(std::string trace_file = kTraceFile);
  ~StandupLogic() override;
  void reset() override;
  LogicResult step(double elapsed_s, const hcr::Telemetry &t) override;

 private:
  struct Impl;
  Impl *impl_;
};

}  // namespace rb::runtime
