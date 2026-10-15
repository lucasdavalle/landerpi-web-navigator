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
#include <optional>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace landerpi_nav2_depth_layer
{

// The floor as seen by the camera, in its optical frame.
struct FloorFit
{
  Eigen::Vector3d normal;    // unit, pointing up (away from the floor)
  Eigen::Vector3d centroid;  // of the inliers
  std::size_t inliers = 0;
  double rms = 0.0;          // m, of the inliers' distance to the plane
};

struct FloorFitConfig
{
  double inlier_distance = 0.01;  // m
  int iterations = 300;
  std::size_t min_inliers = 500;  // fewer: no reliable floor in view
  uint32_t seed = 0;              // RANSAC is random; fixed seed = repeatable
};

// RANSAC plane + least-squares refinement on (mostly) floor points. Returns
// nothing if no plane gathers enough inliers.
std::optional<FloorFit> fitFloor(
  const std::vector<Eigen::Vector3d> & points, const FloorFitConfig & config);

// Where the camera sits relative to the robot, in base_footprint
// (x forward, y left, z up, z = 0 on the floor).
struct CameraMount
{
  Eigen::Isometry3d base_from_optical = Eigen::Isometry3d::Identity();
  double height = 0.0;  // m above the floor (from the floor plane)
  double pitch = 0.0;   // rad the optical axis looks down (from the floor plane)
  double roll = 0.0;    // rad (from the floor plane)
};

// Why calibrate against the floor: the arm's kinematic model is off by ~10
// deg in camera pitch (stage 6b finding), which would tilt the whole floor.
// The floor plane gives height, pitch and roll exactly; x, y and yaw (which
// the floor cannot observe) come from the URDF.
CameraMount mountFromFloor(const FloorFit & floor, double x, double y, double yaw);

// The fixed rotation between a camera's body frame (x forward, y left,
// z up) and its optical frame (x right, y down, z forward).
Eigen::Matrix3d bodyFromOptical();

}  // namespace landerpi_nav2_depth_layer
