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

#include "landerpi_nav2_depth_layer/height_classifier.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace landerpi_nav2_depth_layer
{

Classified classify(const std::vector<Eigen::Vector3d> & points_base, const RobotEnvelope & robot)
{
  if (!(robot.height > 0.0) || !(robot.max_range > 0.0) || !(robot.height_margin >= 0.0) ||
    !(robot.floor_band >= 0.0 && robot.floor_band < robot.height))
  {
    throw std::invalid_argument("bad robot envelope");
  }
  Classified out;
  for (const auto & p : points_base) {
    if (!p.allFinite()) {
      continue;
    }
    if (std::hypot(p.x(), p.y()) > robot.max_range) {
      ++out.too_far;
      continue;
    }
    if (p.x() >= robot.body_x_min && p.x() <= robot.body_x_max &&
      std::abs(p.y()) <= robot.body_half_width)
    {
      ++out.own_body;
      continue;
    }
    if (p.z() < robot.floor_band) {
      out.floor.emplace_back(p.x(), p.y());
    } else if (p.z() <= robot.height + robot.height_margin) {
      out.obstacles.emplace_back(p.x(), p.y());
    } else {
      ++out.above;
    }
  }
  return out;
}

}  // namespace landerpi_nav2_depth_layer
