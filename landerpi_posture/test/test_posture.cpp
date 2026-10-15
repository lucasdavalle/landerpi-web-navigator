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

#include <array>
#include <cmath>
#include <string>

#include "landerpi_posture/posture.hpp"

using landerpi_posture::CalibrationLimits;
using landerpi_posture::Posture;
using landerpi_posture::calibrationPlausible;
using landerpi_posture::clampPulses;
using landerpi_posture::footprintCorners;
using landerpi_posture::footprintString;

TEST(Posture, FootprintOfTheLowPostureReachesTheArm)
{
  Posture low;
  low.front = 0.34;  // chassis front 0.11 + arm 0.23
  const auto c = footprintCorners(low);
  ASSERT_EQ(c.size(), 4u);
  EXPECT_DOUBLE_EQ(c[0][0], 0.34);
  EXPECT_DOUBLE_EQ(c[2][0], -0.11);
  EXPECT_EQ(footprintString(low), "[[0.34, 0.09], [-0.11, 0.09], [-0.11, -0.09], [0.34, -0.09]]");
}

TEST(Posture, AcceptsACalibrationCloseToTheStoredOne)
{
  Posture high;  // stored: 0.289 m, 0.389 rad, 0.049 rad
  const CalibrationLimits lim;
  EXPECT_TRUE(calibrationPlausible(0.295, 0.40, 0.04, high, lim));
}

TEST(Posture, RejectsAWrongPlane)
{
  Posture high;
  const CalibrationLimits lim;
  EXPECT_FALSE(calibrationPlausible(0.60, 0.39, 0.05, high, lim));  // a table top, not the floor
  EXPECT_FALSE(calibrationPlausible(0.29, 1.20, 0.05, high, lim));  // a wall
  EXPECT_FALSE(calibrationPlausible(0.29, 0.39, std::nan(""), high, lim));
}

TEST(Posture, ClampsPulses)
{
  const std::array<int, 5> out = clampPulses({-20, 500, 1200, 10, 999});
  EXPECT_EQ(out[0], 0);
  EXPECT_EQ(out[2], 1000);
  EXPECT_EQ(out[3], 10);
}
