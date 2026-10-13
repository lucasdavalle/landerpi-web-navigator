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

#include "landerpi_web_nav/line_extractor.hpp"
#include "landerpi_web_nav/message_codec.hpp"

namespace landerpi_web_nav
{

// The parts of a sensor_msgs/LaserScan needed here, without ROS types.
struct ScanView
{
  double angle_min = 0.0;
  double angle_increment = 0.0;
  double range_min = 0.0;
  double range_max = 0.0;
  const std::vector<float> * ranges = nullptr;
};

// Laser beams -> points in the map frame, given the lidar's pose in that
// frame (the node gets it from TF: map -> lidar_frame). Invalid beams
// (NaN, inf, outside [range_min, range_max]) are skipped. `stride` keeps
// every n-th beam, to thin the stream sent to the browser.
std::vector<Point2> projectScan(const ScanView & scan, const Pose2D & sensor, int stride = 1);

}  // namespace landerpi_web_nav
