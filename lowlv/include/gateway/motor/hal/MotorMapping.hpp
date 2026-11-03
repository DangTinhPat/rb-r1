// Adapted from lumi-quantum 7b86053c89620484a3c99e869cce3c23dd39d37f; see PROVENANCE.md.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace gateway::hal::motor_mapping {

// Two one-wire UART buses, one per leg. Bus index handed to the transport: 0 = left leg (UART2, GPIO15), 1 = right
// leg (UART1, GPIO17). Every bus carries the same three drivers.
constexpr std::uint8_t kWheelId = 1;  // Kpower BL4012, speed control
constexpr std::uint8_t kHipId = 2;    // joint driver, MIT
constexpr std::uint8_t kKneeId = 3;   // joint driver, MIT, on the crank of the knee parallelogram
constexpr std::size_t kBusCount = 2;

// Joint index contract from rb1::core::JointId without taking a dependency on the control core:
// [LH, LK, LW, RH, RK, RW].
struct JointBinding {
  std::size_t joint_index = 0;
  std::uint8_t bus = 0;
  std::uint8_t motor_id = 1;
  // Core coordinate = driver_sign * driver coordinate + offset. Applies to position, velocity and the set-point.
  float driver_sign = 1.0F;
};

// The four leg joints, in LegLock order: left hip, left knee, right hip, right knee.
//
// What the knee driver measures. 1: the crank of the parallelogram, phi = hip + knee, so the core knee angle is
// phi - hip (MotorFeedback) and the knee set-point is hip_ref + knee_ref (LegLock). 0: the knee joint itself, whatever
// the hip does. Measured on the robot (2026-11-01, hip moved 3.15 rad with the folded leg resting on the floor): the knee
// driver reading stayed within 0.25 rad, so it is 0. The two legs are mirror images, so
// their driver signs are opposite (left +1, right -1).
inline constexpr std::array<JointBinding, 4> kLegBindings{{
    {0U, 0U, kHipId, 1.0F},    // left hip
    {1U, 0U, kKneeId, 1.0F},   // left knee (crank)
    {3U, 1U, kHipId, -1.0F},   // right hip
    {4U, 1U, kKneeId, -1.0F},  // right knee (crank)
}};

// The two wheels. The left wheel follows the core direction, the right one is reversed.
inline constexpr std::array<JointBinding, 2> kWheelBindings{{
    {2U, 0U, kWheelId, 1.0F},   // left wheel (joint LW)
    {5U, 1U, kWheelId, -1.0F},  // right wheel (joint RW)
}};
inline constexpr bool kWheelDirectionsVerified = true;

// Signs of the leg drivers. The bench (examples/check_motor) checked on the robot that its `dir` makes a positive angle
// a flexion (gap): hip left +1, right -1; knee left -1, right +1. The core hip grows with flexion (the homing pose is the
// hip at its +135 deg stop), so the hip sign equals `dir`; the core knee grows with extension (0 = folded), so the knee
// sign is -`dir`: left +1, right -1, as in kLegBindings. The crank coupling phi = hip + knee is that of the model
// (model/actuators.yaml). The controller refuses to engage the legs while this is false (docs/firmware.md, "Xác minh chân").
inline constexpr bool kLegDirectionsVerified = true;
inline constexpr float kKneeCrankCoupling = 0.0F;

// Present Speed readback scale. The 1/0.585 correction overstated wheel speed ~1.7x, so the readback is used unscaled.
inline constexpr float kWheelPresentSpeedScale = 1.0F;
// Protocol documentation infers raw speed = RPM; retain an enable lock until this conversion has been measured on the
// assembled wheel drive.
inline constexpr bool kWheelSpeedScaleVerified = true;

}  // namespace gateway::hal::motor_mapping
