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

#include "landerpi_web_nav/scan_projector.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace landerpi_web_nav
{

std::vector<Point2> projectScan(const ScanView & scan, const Pose2D & sensor, int stride)
{
  if (scan.ranges == nullptr || stride < 1) {
    throw std::invalid_argument("projectScan needs ranges and stride >= 1");
  }
  const double c = std::cos(sensor.yaw);
  const double s = std::sin(sensor.yaw);
  std::vector<Point2> out;
  out.reserve(scan.ranges->size() / static_cast<std::size_t>(stride) + 1);
  for (std::size_t i = 0; i < scan.ranges->size(); i += static_cast<std::size_t>(stride)) {
    const double r = (*scan.ranges)[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
      continue;
    }
    const double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    // Beam end in the lidar frame, then rotated and translated into map.
    const double lx = r * std::cos(a);
    const double ly = r * std::sin(a);
    out.push_back({sensor.x + c * lx - s * ly, sensor.y + s * lx + c * ly});
  }
  return out;
}

}  // namespace landerpi_web_nav
