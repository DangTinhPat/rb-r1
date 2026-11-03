// Policy observation/action mapping and the recurrent-policy interface.
#pragma once
#include <array>
#include <memory>
#include <string>

#include "rb/runtime/protocol.hpp"

namespace rb::runtime {

using Action = std::array<double, 6>;
using Observation = std::array<double, 32>;

// Servo references -> raw previous action (hip/crank rad, wheel rad/s).
Action encodeTargets(const std::array<double, 6> &target);
// Raw actor action -> [hipL,hipR,phiL,phiR,omegaL,omegaR]. No torque; phi is hip+knee.
std::array<double, 6> decodeActions(const Action &action);
Observation observation(const hcr::Telemetry &t, const Action &previous,
                        const std::array<double, 2> &velocity);

// One recurrent inference per 100 Hz tick. step() throws std::exception on a bad
// observation or result; the core turns that into policy_fault.
class Policy {
 public:
  virtual ~Policy() = default;
  virtual void reset() = 0;
  virtual Action step(const Observation &obs) = 0;
  virtual int steps() const = 0;
  virtual double lastMs() const = 0;
  virtual double lastNativeMs() const { return -1; }  // < 0: not measured
};

// RKNN GRU512 on the RV1106 NPU. Verifies the artifact hashes in models/manifest.json
// before loading anything. Throws std::runtime_error when the NPU runtime is missing.
std::unique_ptr<Policy> makeRknnPolicy(const std::string &root);

}  // namespace rb::runtime
