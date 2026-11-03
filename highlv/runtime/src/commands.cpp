#include "rb/runtime/commands.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <vector>

namespace rb::runtime {

namespace {
double parseFloat(const std::string &token) {
  char *end = nullptr;
  const double v = std::strtod(token.c_str(), &end);
  if (token.empty() || *end != 0) throw std::invalid_argument("could not convert string to float: '" + token + "'");
  return v;
}
}  // namespace

Command parseCommand(const std::string &line) {
  std::vector<std::string> tokens;
  std::string current;
  for (unsigned char c : line) {
    if (c >= 0x80) throw std::invalid_argument("command must be ASCII");
    if (std::isspace(c)) {
      if (!current.empty()) tokens.push_back(std::move(current)), current.clear();
    } else {
      current += static_cast<char>(c);
    }
  }
  if (!current.empty()) tokens.push_back(current);
  if (tokens.empty()) throw std::invalid_argument("empty command");
  std::string op = tokens[0];
  for (char &c : op) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  if (op == "VELOCITY" && tokens.size() == 3) {
    const double vx = parseFloat(tokens[1]), wz = parseFloat(tokens[2]);
    if (!std::isfinite(vx) || !std::isfinite(wz)) throw std::invalid_argument("nonfinite velocity");
    return {op, vx, wz};
  }
  if ((op == "ARM" || op == "DISARM" || op == "RESET" || op == "ESTOP" || op == "RESET_ENCODERS" ||
       op == "CAL_IMU") && tokens.size() == 1)
    return {op, 0.0, 0.0};
  throw std::invalid_argument("use ARM, DISARM, RESET, ESTOP, RESET_ENCODERS, CAL_IMU, or VELOCITY vx wz");
}

}  // namespace rb::runtime
