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

#include <vector>

#include "Eigen/Core"

#include "landerpi_nav2_depth_layer/height_classifier.hpp"

namespace landerpi_nav2_depth_layer
{

// One classified camera frame as a point cloud in base_footprint, so the
// image is processed once (by the local costmap's layer) and the global
// costmap's layer reuses it. The class is encoded in z:
//   floor at kFloorZ, obstacles at kObstacleZ, crouchable at kCrouchableZ.
// The floor is thinned to one point per `floor_cell` (a costmap cell) so
// the cloud stays small: thousands of floor points become a few hundred.
constexpr double kFloorZ = 0.0;
constexpr double kObstacleZ = 0.1;
constexpr double kCrouchableZ = 0.3;

std::vector<Eigen::Vector3d> toCloudPoints(const Classified & c, double floor_cell);

// The inverse, splitting halfway between the encoded heights. Only the
// obstacle, crouchable and floor lists are filled.
Classified fromCloudPoints(const std::vector<Eigen::Vector3d> & points);

}  // namespace landerpi_nav2_depth_layer
