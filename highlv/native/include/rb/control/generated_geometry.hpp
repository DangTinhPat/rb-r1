#pragma once
#include <array>
namespace rb::control::geometry {
struct Part {
  float mass, x, z, iyy;
};
struct Origin {
  float x, y, z;
};
inline constexpr std::array<Part, 7> parts{{
    {1.955F, 0.01234025F, 0.0110474F, 0.007118036F},    // body (body_link + head)
    {0.085F, -0.0477985F, 0.008336F, 0.0002281138F},    // left thigh
    {0.167F, 0.08313712F, -0.03849651F, 0.0001397159F}, // left shin + BL4012 stator
    {0.018F, 0.0F, 0.0F, 1.1025e-05F},                  // left wheel + tyre
    {0.085F, -0.0477985F, 0.008336F, 0.0002281138F},    // right thigh
    {0.167F, 0.08313712F, -0.03849651F, 0.0001397159F}, // right shin + BL4012 stator
    {0.018F, 0.0F, 0.0F, 1.1025e-05F}                   // right wheel + tyre
}};
// Joint origins: hip in the body frame, knee relative to the hip, wheel axle relative to the knee.
inline constexpr std::array<Origin, 6> origins{{{0.0F, 0.0812F, 0.0F},
                                                {-0.09999935F, 0.0212F, -0.0002569F},
                                                {0.09073855F, 0.0176999F, -0.0420287F},
                                                {0.0F, -0.0812F, 0.0F},
                                                {-0.09999935F, -0.0212F, -0.0002569F},
                                                {0.09073855F, -0.0176999F, -0.0420287F}}};
inline constexpr float total_mass_kg = 2.495F;
inline constexpr float wheel_radius_m = 0.035F;
inline constexpr float axle_spacing_m = 0.2401998F;
inline constexpr float neutral_hip_rad = 0.0F;
inline constexpr float neutral_knee_rad = 0.0F; // CAD pose: knee fully folded.
inline constexpr float knee_zero_rebase_rad = 0.0F;
inline constexpr float neutral_spawn_z_m = 0.0772856F;
inline constexpr float native_hip_min_rad = -1.53589F;
inline constexpr float native_hip_max_rad = 2.321288F;
inline constexpr float native_knee_min_rad = 0.0F;
inline constexpr float native_knee_max_rad = 1.919862F;
inline constexpr float native_knee_travel_rad = native_knee_max_rad;
// Encoder-zero poses in controller coordinates (hip measured from hang_hip_cad). The real robot is
// homed in home_*: knee folded to its stop, hip at its positive mechanical stop. straight_* is the
// extended pose (knee at the soft limit, axle straight below the hip, 184.8 mm).
inline constexpr float hang_hip_cad_rad = -0.215602F;
inline constexpr float home_hip_rad = 2.572669F;
inline constexpr float home_knee_rad = 0.0F;
inline constexpr float straight_hip_rad = -0.959927F;
inline constexpr float straight_knee_rad = 1.919862F;
inline constexpr float leg_hip_inertia_kgm2 = 0.00075F; // inherited, not measured for RB
inline constexpr float leg_knee_inertia_kgm2 = 0.0025F; // inherited, not measured for RB
} // namespace rb::control::geometry
