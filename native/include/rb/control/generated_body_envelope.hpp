#pragma once
#include <array>
namespace rb::control::geometry {
struct BodyBox {
  std::array<float, 3> lo, hi;
};
inline constexpr std::array<BodyBox, 3> body_envelope{{
    {{-0.081748F, -0.0805F, -0.0426F}, {0.111748F, 0.0805F, 0.0501F}},       // Body 1 v1.3.2-1
    {{-0.081748F, -0.071748F, -0.0501F}, {0.111748F, 0.071748F, -0.0004F}},  // Body 2 v1.3.2
    {{-0.038102F, -0.048102F, 0.047898F}, {0.098002F, 0.048102F, 0.117402F}} // Robot Head POC
}};
} // namespace rb::control::geometry
