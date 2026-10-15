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


#include "landerpi_nav2_depth_layer/observation_cloud.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace landerpi_nav2_depth_layer
{

std::vector<Eigen::Vector3d> toCloudPoints(const Classified & c, double floor_cell)
{
  if (!(floor_cell > 0.0)) {
    throw std::invalid_argument("floor_cell must be positive");
  }
  std::vector<Eigen::Vector3d> out;
  out.reserve(c.obstacles.size() + c.crouchable.size() + c.floor.size() / 8);
  for (const auto & o : c.obstacles) {
    out.emplace_back(o.x(), o.y(), kObstacleZ);
  }
  for (const auto & o : c.crouchable) {
    out.emplace_back(o.x(), o.y(), kCrouchableZ);
  }
  // One floor point per cell: the key packs the two cell indices (the
  // camera sees a few metres, far inside 32 bits each).
  std::unordered_set<std::int64_t> seen;
  for (const auto & f : c.floor) {
    const auto i = static_cast<std::int64_t>(std::floor(f.x() / floor_cell));
    const auto j = static_cast<std::int64_t>(std::floor(f.y() / floor_cell));
    if (seen.insert((i << 32) ^ (j & 0xffffffff)).second) {
      out.emplace_back(f.x(), f.y(), kFloorZ);
    }
  }
  return out;
}

Classified fromCloudPoints(const std::vector<Eigen::Vector3d> & points)
{
  Classified c;
  for (const auto & p : points) {
    if (!p.allFinite()) {
      continue;
    }
    if (p.z() > (kObstacleZ + kCrouchableZ) / 2) {
      c.crouchable.emplace_back(p.x(), p.y());
    } else if (p.z() > (kFloorZ + kObstacleZ) / 2) {
      c.obstacles.emplace_back(p.x(), p.y());
    } else {
      c.floor.emplace_back(p.x(), p.y());
    }
  }
  return c;
}

}  // namespace landerpi_nav2_depth_layer
