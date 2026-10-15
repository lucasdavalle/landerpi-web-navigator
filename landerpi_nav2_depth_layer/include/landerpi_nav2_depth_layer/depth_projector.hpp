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

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "Eigen/Core"

namespace landerpi_nav2_depth_layer
{

// Pinhole intrinsics from sensor_msgs/CameraInfo (k = [fx 0 cx; 0 fy cy; 0 0 1]).
struct Intrinsics
{
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
};

struct ProjectionConfig
{
  int stride = 4;            // use every n-th pixel in both directions
  double min_range = 0.25;   // m: the Aurora930 measures nothing closer (~0.22-0.35)
  double max_range = 3.0;    // m: noisier beyond, and the costmap is only 3 x 3 m
  // Only rows from this fraction of the image height down (0 = all rows).
  // The floor calibration uses the lower half, where the floor is.
  double min_row_fraction = 0.0;
};

// Depth image (16-bit millimetres, row-major, as the Aurora930 publishes
// it: mono16) -> 3D points in the camera's OPTICAL frame (x right, y down,
// z forward). Pixels with no depth (0) or out of range are skipped. Throws
// std::invalid_argument on inconsistent sizes or a bad config.
std::vector<Eigen::Vector3d> projectDepth(
  const std::vector<uint16_t> & depth_mm, int width, int height, const Intrinsics & k,
  const ProjectionConfig & config);

}  // namespace landerpi_nav2_depth_layer
