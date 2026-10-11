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

#include "landerpi_web_nav/scan_recenter.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace landerpi_web_nav
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

// Wraps an angle to [-pi, pi).
double wrapAngle(double a)
{
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) {
    a += 2.0 * kPi;
  }
  return a - kPi;
}

}  // namespace

RecenterPlan planRecenter(const ScanLayout & layout)
{
  if (layout.count < 2) {
    throw std::invalid_argument("scan needs at least 2 beams");
  }
  if (!std::isfinite(layout.angle_increment) || layout.angle_increment <= 0.0 ||
    !std::isfinite(layout.angle_min))
  {
    throw std::invalid_argument("scan needs a finite angle_min and a positive angle_increment");
  }
  const double span = static_cast<double>(layout.count) * layout.angle_increment;
  if (std::abs(span - 2.0 * kPi) > 2.0 * layout.angle_increment) {
    throw std::invalid_argument(
            "only full-circle scans can be recentered: count * increment = " +
            std::to_string(span) + " rad");
  }

  // The new first beam is the one whose wrapped angle is the smallest, i.e.
  // the closest to -pi from above. Beams are evenly spaced, so after the
  // rotation the wrapped angles increase monotonically again.
  RecenterPlan plan;
  double smallest = kPi;
  for (std::size_t i = 0; i < layout.count; ++i) {
    const double a = wrapAngle(layout.angle_min + static_cast<double>(i) * layout.angle_increment);
    if (a < smallest) {
      smallest = a;
      plan.shift = i;
    }
  }
  plan.angle_min = smallest;
  plan.angle_max = smallest + static_cast<double>(layout.count - 1) * layout.angle_increment;
  return plan;
}

}  // namespace landerpi_web_nav
