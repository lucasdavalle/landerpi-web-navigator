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
#include <limits>
#include <stdexcept>
#include <vector>

#include "landerpi_web_nav/scan_projector.hpp"

using landerpi_web_nav::Pose2D;
using landerpi_web_nav::ScanView;
using landerpi_web_nav::projectScan;

namespace
{

constexpr double kPi = 3.14159265358979323846;

}  // namespace

TEST(ScanProjector, BeamStraightAheadOfAnUnrotatedSensor)
{
  const std::vector<float> ranges{2.0f};
  const ScanView scan{0.0, 0.1, 0.05, 12.0, &ranges};
  const auto pts = projectScan(scan, Pose2D{1.0, 1.0, 0.0});
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_NEAR(pts[0].x, 3.0, 1e-9);
  EXPECT_NEAR(pts[0].y, 1.0, 1e-9);
}

TEST(ScanProjector, RotatesWithTheSensor)
{
  // Sensor facing +y: a beam straight ahead lands above it.
  const std::vector<float> ranges{2.0f};
  const ScanView scan{0.0, 0.1, 0.05, 12.0, &ranges};
  const auto pts = projectScan(scan, Pose2D{0.0, 0.0, kPi / 2});
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_NEAR(pts[0].x, 0.0, 1e-9);
  EXPECT_NEAR(pts[0].y, 2.0, 1e-9);
}

TEST(ScanProjector, UsesBeamAngles)
{
  // 4 beams every 90 deg starting at 0: front, left, back, right.
  const std::vector<float> ranges{1.0f, 1.0f, 1.0f, 1.0f};
  const ScanView scan{0.0, kPi / 2, 0.05, 12.0, &ranges};
  const auto pts = projectScan(scan, Pose2D{});
  ASSERT_EQ(pts.size(), 4u);
  EXPECT_NEAR(pts[1].y, 1.0, 1e-9);   // left
  EXPECT_NEAR(pts[2].x, -1.0, 1e-9);  // back
  EXPECT_NEAR(pts[3].y, -1.0, 1e-9);  // right
}

TEST(ScanProjector, SkipsInvalidBeamsAndThins)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const std::vector<float> ranges{1.0f, nan, inf, 0.01f, 30.0f, 2.0f};
  const ScanView scan{0.0, 0.1, 0.05, 12.0, &ranges};
  EXPECT_EQ(projectScan(scan, Pose2D{}).size(), 2u);    // 1.0 and 2.0
  EXPECT_EQ(projectScan(scan, Pose2D{}, 5).size(), 2u);  // beams 0 and 5
  EXPECT_THROW(projectScan(scan, Pose2D{}, 0), std::invalid_argument);
}
