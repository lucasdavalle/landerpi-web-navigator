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
// image is processed once (by the local costmap's layer) and both the
// global costmap's layer and the Collision Monitor reuse it. The class is
// encoded in z:
//   obstacles at kObstacleZ: inside any height band the monitor checks;
//   floor at kFloorZ: below the monitor's min_height, so it ignores it.
// The floor is thinned to one point per `floor_cell` (a costmap cell) so
// the cloud stays small: thousands of floor points become a few hundred.
constexpr double kObstacleZ = 0.1;
constexpr double kFloorZ = 0.0;

std::vector<Eigen::Vector3d> toCloudPoints(const Classified & c, double floor_cell);

// The inverse: z above half of kObstacleZ is an obstacle, the rest floor.
// Only the obstacle and floor lists are filled.
Classified fromCloudPoints(const std::vector<Eigen::Vector3d> & points);

}  // namespace landerpi_nav2_depth_layer
