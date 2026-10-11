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

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "landerpi_web_nav/scan_recenter.hpp"

using landerpi_web_nav::RecenterPlan;
using landerpi_web_nav::ScanLayout;
using landerpi_web_nav::applyRecenter;
using landerpi_web_nav::planRecenter;

namespace
{

constexpr double kPi = 3.14159265358979323846;

// Same shape as the real LanderPi lidar (stage 0 bag): 0..2pi.
ScanLayout zeroToTwoPi(std::size_t count)
{
  return ScanLayout{0.0, 2.0 * kPi / static_cast<double>(count), count};
}

}  // namespace

TEST(ScanRecenter, ZeroToTwoPiStartsNearMinusPi)
{
  const auto layout = zeroToTwoPi(8);  // beams every 45 deg
  const RecenterPlan plan = planRecenter(layout);
  // Beam 4 is at 180 deg == -180 deg: it becomes the first one.
  EXPECT_EQ(plan.shift, 4u);
  EXPECT_NEAR(plan.angle_min, -kPi, 1e-9);
  EXPECT_NEAR(plan.angle_max, -kPi + 7 * kPi / 4, 1e-9);
}

TEST(ScanRecenter, BeamKeepsItsPhysicalDirection)
{
  // The beam pointing straight ahead (angle 0, index 0 in the raw scan) must
  // end up at the index whose new angle is 0.
  const auto layout = zeroToTwoPi(8);
  const auto plan = planRecenter(layout);
  std::vector<float> ranges{1, 2, 3, 4, 5, 6, 7, 8};  // value = 1 + raw index
  applyRecenter(plan, ranges);
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double new_angle = plan.angle_min + static_cast<double>(i) * layout.angle_increment;
    const auto raw_index = static_cast<std::size_t>(ranges[i]) - 1;
    const double raw_angle = layout.angle_min +
      static_cast<double>(raw_index) * layout.angle_increment;
    // Same direction: the angles differ by a multiple of 2*pi.
    EXPECT_NEAR(std::remainder(new_angle - raw_angle, 2.0 * kPi), 0.0, 1e-9) << "beam " << i;
  }
  EXPECT_FLOAT_EQ(ranges[4], 1.0f);  // straight ahead sits at -pi + 4*45deg = 0
}

TEST(ScanRecenter, RealLidarLayout)
{
  // 502 beams of 0.012541 rad, as recorded on the robot.
  const ScanLayout layout{0.0, 0.012541288509964943, 502};
  const auto plan = planRecenter(layout);
  EXPECT_GE(plan.angle_min, -kPi);
  EXPECT_LT(plan.angle_min, -kPi + layout.angle_increment);
  // Symmetric enough for rf2o's -fov/2 assumption: off by < one beam.
  const double fov = plan.angle_max - plan.angle_min;
  EXPECT_LT(std::abs(plan.angle_min + fov / 2.0), layout.angle_increment);
}

TEST(ScanRecenter, AlreadyCenteredScanIsUnchanged)
{
  const ScanLayout layout{-kPi, 2.0 * kPi / 8.0, 8};
  const auto plan = planRecenter(layout);
  EXPECT_EQ(plan.shift, 0u);
  EXPECT_NEAR(plan.angle_min, -kPi, 1e-9);
}

TEST(ScanRecenter, EmptyIntensitiesAreLeftAlone)
{
  const auto plan = planRecenter(zeroToTwoPi(8));
  std::vector<float> intensities;
  applyRecenter(plan, intensities);
  EXPECT_TRUE(intensities.empty());
}

TEST(ScanRecenter, RejectsPartialAndMalformedScans)
{
  // 180 deg scan: cannot be made symmetric by rotating.
  EXPECT_THROW(planRecenter(ScanLayout{0.0, kPi / 100.0, 100}), std::invalid_argument);
  EXPECT_THROW(planRecenter(ScanLayout{0.0, 0.0, 100}), std::invalid_argument);
  EXPECT_THROW(planRecenter(ScanLayout{0.0, -0.1, 63}), std::invalid_argument);
  EXPECT_THROW(planRecenter(ScanLayout{0.0, 0.1, 1}), std::invalid_argument);
  EXPECT_THROW(planRecenter(ScanLayout{std::nan(""), 0.1, 63}), std::invalid_argument);
}
