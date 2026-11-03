// Operator commands travel over a local socket, separate from HCR1.
#pragma once
#include <string>

namespace rb::runtime {

struct Command {
  std::string op;  // ARM, DISARM, RESET, ESTOP, RESET_ENCODERS, CAL_IMU, VELOCITY
  double vx = 0, wz = 0;
};

// Throws std::invalid_argument for anything that is not one well-formed command.
Command parseCommand(const std::string &line);

}  // namespace rb::runtime
