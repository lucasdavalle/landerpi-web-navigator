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

#include <algorithm>
#include <cstddef>
#include <vector>

namespace landerpi_web_nav
{

// Why this exists: rf2o_laser_odometry ignores LaserScan::angle_min and
// assumes the first beam sits at -fov/2 (CLaserOdometry2D.cpp:441). The
// LanderPi lidar publishes 0..2pi, so rf2o sees every beam rotated by 180 deg
// and reports translation with the wrong sign (rotation stays right). This
// rotates a full-circle scan so it spans [-pi, pi), which is what rf2o
// assumes. See CLAUDE.md, "Hallazgo clave".

// Angular layout of a LaserScan (the parts that matter for recentering).
struct ScanLayout
{
  double angle_min = 0.0;
  double angle_increment = 0.0;
  std::size_t count = 0;
};

// Precomputed answer for one layout: rotate the arrays left by `shift`
// beams and use the new angle bounds. Computed once per layout, applied to
// every scan.
struct RecenterPlan
{
  std::size_t shift = 0;
  double angle_min = 0.0;
  double angle_max = 0.0;
};

// Throws std::invalid_argument unless the layout is a full circle
// (count * increment within two beams of 2*pi) with a positive increment:
// rotating a partial scan would not make it symmetric, so it is rejected
// instead of silently producing a scan rf2o would still misread.
RecenterPlan planRecenter(const ScanLayout & layout);

// Rotates one per-beam array (ranges or intensities) according to the plan.
// An empty array is left alone (intensities are optional in LaserScan).
template<typename T>
void applyRecenter(const RecenterPlan & plan, std::vector<T> & data)
{
  if (data.empty()) {
    return;
  }
  std::rotate(data.begin(), data.begin() + (plan.shift % data.size()), data.end());
}

}  // namespace landerpi_web_nav
