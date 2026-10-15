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

#include "landerpi_nav2_depth_layer/depth_projector.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace landerpi_nav2_depth_layer
{

std::vector<Eigen::Vector3d> projectDepth(
  const std::vector<uint16_t> & depth_mm, int width, int height, const Intrinsics & k,
  const ProjectionConfig & config)
{
  if (width <= 0 || height <= 0 ||
    depth_mm.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height))
  {
    throw std::invalid_argument("depth image size does not match width * height");
  }
  if (!(k.fx > 0.0) || !(k.fy > 0.0) || config.stride < 1 ||
    !(config.min_range >= 0.0) || !(config.max_range > config.min_range) ||
    !(config.min_row_fraction >= 0.0 && config.min_row_fraction < 1.0))
  {
    throw std::invalid_argument("bad intrinsics or projection config");
  }

  const int first_row = static_cast<int>(std::ceil(config.min_row_fraction * height));
  std::vector<Eigen::Vector3d> points;
  points.reserve(
    static_cast<std::size_t>((height - first_row) / config.stride + 1) *
    static_cast<std::size_t>(width / config.stride + 1));
  for (int v = first_row; v < height; v += config.stride) {
    const uint16_t * row = depth_mm.data() + static_cast<std::size_t>(v) * width;
    for (int u = 0; u < width; u += config.stride) {
      const double z = row[u] * 0.001;  // mm -> m
      if (z < config.min_range || z > config.max_range) {
        continue;  // 0 = no reading, also lands here
      }
      // Inverse pinhole: pixel (u, v) at depth z.
      points.emplace_back((u - k.cx) * z / k.fx, (v - k.cy) * z / k.fy, z);
    }
  }
  return points;
}

}  // namespace landerpi_nav2_depth_layer
