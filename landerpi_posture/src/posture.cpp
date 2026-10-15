// Copyright 2026 ldavalle
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "landerpi_posture/posture.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace landerpi_posture
{

std::vector<std::array<double, 2>> footprintCorners(const Posture & p)
{
  return {{{p.front, p.half_width}}, {{p.back, p.half_width}}, {{p.back, -p.half_width}},
    {{p.front, -p.half_width}}};
}

std::string footprintString(const Posture & p)
{
  std::ostringstream s;
  s << "[";
  const auto corners = footprintCorners(p);
  for (std::size_t i = 0; i < corners.size(); ++i) {
    s << (i ? ", " : "") << "[" << corners[i][0] << ", " << corners[i][1] << "]";
  }
  s << "]";
  return s.str();
}

bool calibrationPlausible(
  double height, double pitch, double roll, const Posture & expected,
  const CalibrationLimits & limits)
{
  if (!std::isfinite(height) || !std::isfinite(pitch) || !std::isfinite(roll)) {
    return false;
  }
  return std::abs(height - expected.cam_height) <= limits.max_height_error &&
         std::abs(pitch - expected.cam_pitch) <= limits.max_angle_error &&
         std::abs(roll - expected.cam_roll) <= limits.max_angle_error;
}

std::array<int, 5> clampPulses(const std::array<int, 5> & pulses)
{
  std::array<int, 5> out{};
  for (std::size_t i = 0; i < pulses.size(); ++i) {
    out[i] = std::clamp(pulses[i], 0, 1000);
  }
  return out;
}

}  // namespace landerpi_posture
