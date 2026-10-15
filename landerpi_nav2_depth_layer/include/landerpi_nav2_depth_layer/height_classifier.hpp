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
#include <vector>

#include "Eigen/Core"

namespace landerpi_nav2_depth_layer
{

// What the robot is, for deciding what a 3D point means to it.
struct RobotEnvelope
{
  // m: below this it is floor. Measured in the room: the calibrated floor
  // stays within 0.5 cm up to 1.5 m, and a 3 cm box reads 1.5-2.6 cm.
  double floor_band = 0.015;
  double height = 0.31;         // m: current height (posture: 0.31 high, 0.22 low)
  // m: safety margin over the height. The camera is the highest point, so
  // anything at its height (a bed edge) is seen at the very top of the image,
  // noisy, and must still count as an obstacle.
  double height_margin = 0.05;
  double max_range = 2.5;    // m (horizontal from the robot): ignore farther points
  // The robot's own body in base_footprint (chassis + what sticks out, plus
  // a margin): points inside are the robot itself (arm, gripper), not
  // obstacles.
  double body_x_min = -0.13;
  double body_x_max = 0.13;
  double body_half_width = 0.11;
};

// Points (in base_footprint) sorted by what they mean for navigation. Only
// x, y are kept: the costmap is 2D, height is what was used to decide.
struct Classified
{
  std::vector<Eigen::Vector2d> obstacles;  // floor_band <= z <= height + margin: would hit it
  std::vector<Eigen::Vector2d> floor;      // z < floor_band: free floor, clears cells
  std::size_t above = 0;     // z > height + margin: the robot passes under it
  std::size_t own_body = 0;  // inside the body box
  std::size_t too_far = 0;
};

// Throws std::invalid_argument on a non-positive height or range, a
// negative margin, or a floor band not below the robot's height.
Classified classify(const std::vector<Eigen::Vector3d> & points_base, const RobotEnvelope & robot);

}  // namespace landerpi_nav2_depth_layer
