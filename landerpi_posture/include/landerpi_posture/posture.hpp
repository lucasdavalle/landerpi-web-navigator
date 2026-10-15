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

#include <array>
#include <string>
#include <vector>

namespace landerpi_posture
{

// One arm posture for navigation (stage 6): what to command and what the
// robot then is (measured on the real robot, 2026-10-15).
struct Posture
{
  std::string name;
  std::array<int, 5> pulses{500, 500, 500, 500, 500};  // servos 1..5 (gripper untouched)
  double height = 0.31;  // m, floor to the highest point
  // Footprint in base_footprint: [x_front, x_back, half_width] (x_back < 0).
  double front = 0.11;
  double back = -0.11;
  double half_width = 0.09;
  // Fallback camera calibration (from a previous floor fit), used when the
  // live one fails: height (m), pitch and roll (rad).
  double cam_height = 0.289;
  double cam_pitch = 0.389;
  double cam_roll = 0.049;
  // False until the stored camera values come from a real floor fit.
  bool calibrated = true;
};

// Nav2 footprint string "[[x, y], ...]" (counter-clockwise rectangle).
std::string footprintString(const Posture & p);

// The 4 corners of the footprint, for geometry_msgs/Polygon.
std::vector<std::array<double, 2>> footprintCorners(const Posture & p);

struct CalibrationLimits
{
  double max_height_error = 0.04;  // m
  double max_angle_error = 0.17;   // rad (~10 deg)
};

// A live floor calibration is trusted only if it is close to the stored
// one: a wrong plane (a wall, a table top) must not tilt the whole costmap.
bool calibrationPlausible(
  double height, double pitch, double roll, const Posture & expected,
  const CalibrationLimits & limits);

// Clamps pulses to the servos' range: nothing out of 0..1000 is sent.
std::array<int, 5> clampPulses(const std::array<int, 5> & pulses);

}  // namespace landerpi_posture
