#pragma once
#include "rb/control/lqr_balancer.hpp"
namespace rb::control {
// Base LQR parameters shared by the recovery stages.
[[nodiscard]] LqrBalancer::Config tunedLqrConfig();
} // namespace rb::control
