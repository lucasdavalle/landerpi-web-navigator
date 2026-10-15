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


#include "landerpi_nav2_depth_layer/clearance_check.hpp"

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace landerpi_nav2_depth_layer
{

ClearanceResult checkClearance(
  const std::vector<std::uint16_t> & depth_mm, int width, int height,
  const ClearanceConfig & config)
{
  if (width <= 0 || height <= 0 ||
    depth_mm.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height))
  {
    throw std::invalid_argument("depth image size mismatch");
  }
  if (config.stride < 1 || !(config.near_distance > 0.0) ||
    !(config.min_valid_fraction > 0.0 && config.min_valid_fraction <= 1.0) ||
    !(config.max_near_fraction >= 0.0 && config.max_near_fraction < 1.0))
  {
    throw std::invalid_argument("bad clearance config");
  }
  const double near_mm = config.near_distance * 1000.0;
  std::size_t valid = 0;
  std::size_t near = 0;
  ClearanceResult r;
  for (int v = 0; v < height; v += config.stride) {
    for (int u = 0; u < width; u += config.stride) {
      ++r.sampled;
      const std::uint16_t d = depth_mm[static_cast<std::size_t>(v) * width + u];
      if (d == 0) {
        continue;
      }
      ++valid;
      if (d < near_mm) {
        ++near;
      }
    }
  }
  r.valid_fraction = static_cast<double>(valid) / static_cast<double>(r.sampled);
  r.near_fraction = valid > 0 ? static_cast<double>(near) / static_cast<double>(valid) : 0.0;
  if (r.valid_fraction < config.min_valid_fraction) {
    r.verdict = Clearance::kUnknown;
  } else if (r.near_fraction > config.max_near_fraction) {
    r.verdict = Clearance::kBlocked;
  } else {
    r.verdict = Clearance::kClear;
  }
  return r;
}

}  // namespace landerpi_nav2_depth_layer
