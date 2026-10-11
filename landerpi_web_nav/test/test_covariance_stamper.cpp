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

#include "landerpi_web_nav/covariance_stamper.hpp"

using landerpi_web_nav::CovarianceStamper;
using landerpi_web_nav::Dof3;
using landerpi_web_nav::Dof6;
using landerpi_web_nav::ImuNoiseModel;
using landerpi_web_nav::OdomNoiseModel;
using landerpi_web_nav::diagonalCovariance;
using landerpi_web_nav::imuCovariances;

namespace
{

// Indices of the diagonal of a row-major 6x6 matrix.
constexpr int kX = 0;
constexpr int kY = 7;
constexpr int kYaw = 35;

OdomNoiseModel slipModel()
{
  OdomNoiseModel model;
  model.pose_stddev = {0.05, 0.05, 1.0, 1.0, 1.0, 0.1};
  model.twist_stddev_base = {0.01, 0.01, 1.0, 1.0, 1.0, 0.02};
  model.twist_stddev_per_mps = {0.3, 0.5, 0.0, 0.0, 0.0, 0.2};
  model.twist_stddev_per_radps = {0.0, 0.0, 0.0, 0.0, 0.0, 0.4};
  model.max_plausible_speed = 1.0;
  model.max_plausible_yaw_rate = 3.0;
  return model;
}

}  // namespace

TEST(DiagonalCovariance, SquaresStddevsOnTheDiagonalOnly)
{
  const auto cov = diagonalCovariance(Dof6{0.1, 0.2, 0.3, 0.4, 0.5, 0.6});
  EXPECT_DOUBLE_EQ(cov[kX], 0.01);
  EXPECT_DOUBLE_EQ(cov[kY], 0.04);
  EXPECT_DOUBLE_EQ(cov[kYaw], 0.36);
  int non_zero = 0;
  for (double v : cov) {
    non_zero += (v != 0.0);
  }
  EXPECT_EQ(non_zero, 6);
}

TEST(DiagonalCovariance, Works3x3)
{
  const auto cov = diagonalCovariance(Dof3{1.0, 2.0, 3.0});
  EXPECT_DOUBLE_EQ(cov[0], 1.0);
  EXPECT_DOUBLE_EQ(cov[4], 4.0);
  EXPECT_DOUBLE_EQ(cov[8], 9.0);
  EXPECT_DOUBLE_EQ(cov[1], 0.0);
}

TEST(CovarianceStamper, AtRestUsesBaseTwistNoise)
{
  const CovarianceStamper stamper{slipModel()};
  const auto out = stamper.stamp(0.0, 0.0, 0.0);
  EXPECT_FALSE(out.rejected);
  EXPECT_DOUBLE_EQ(out.twist[kX], 0.01 * 0.01);
  EXPECT_DOUBLE_EQ(out.twist[kYaw], 0.02 * 0.02);
  EXPECT_DOUBLE_EQ(out.pose[kX], 0.05 * 0.05);
}

TEST(CovarianceStamper, TwistNoiseGrowsWithPlanarSpeed)
{
  const CovarianceStamper stamper{slipModel()};
  // 0.3 m/s forward + 0.4 m/s sideways = 0.5 m/s planar speed.
  const auto out = stamper.stamp(0.3, 0.4, 0.0);
  EXPECT_FALSE(out.rejected);
  const double sx = 0.01 + 0.3 * 0.5;
  const double sy = 0.01 + 0.5 * 0.5;
  EXPECT_NEAR(out.twist[kX], sx * sx, 1e-12);
  EXPECT_NEAR(out.twist[kY], sy * sy, 1e-12);
  // Pose noise does not depend on speed.
  EXPECT_DOUBLE_EQ(out.pose[kX], 0.05 * 0.05);
}

TEST(CovarianceStamper, YawNoiseGrowsWhenTurningInPlace)
{
  const CovarianceStamper stamper{slipModel()};
  // Pure rotation: planar speed is zero, only the per-rad/s term applies,
  // and the sign of the rotation does not matter.
  const auto left = stamper.stamp(0.0, 0.0, 1.0);
  const auto right = stamper.stamp(0.0, 0.0, -1.0);
  const double syaw = 0.02 + 0.4 * 1.0;
  EXPECT_NEAR(left.twist[kYaw], syaw * syaw, 1e-12);
  EXPECT_DOUBLE_EQ(left.twist[kYaw], right.twist[kYaw]);
  EXPECT_DOUBLE_EQ(left.twist[kX], 0.01 * 0.01);
}

TEST(CovarianceStamper, RejectsImplausibleSpeed)
{
  const CovarianceStamper stamper{slipModel()};
  const auto out = stamper.stamp(1.5, 0.0, 0.0);
  EXPECT_TRUE(out.rejected);
  EXPECT_DOUBLE_EQ(out.twist[kX], CovarianceStamper::kRejectedVariance);
  EXPECT_DOUBLE_EQ(out.pose[kYaw], CovarianceStamper::kRejectedVariance);
}

TEST(CovarianceStamper, RejectsImplausibleYawRateInBothDirections)
{
  const CovarianceStamper stamper{slipModel()};
  EXPECT_TRUE(stamper.stamp(0.0, 0.0, 3.5).rejected);
  EXPECT_TRUE(stamper.stamp(0.0, 0.0, -3.5).rejected);
  EXPECT_FALSE(stamper.stamp(0.0, 0.0, -2.9).rejected);
}

TEST(CovarianceStamper, RejectsNonFiniteMeasurements)
{
  const CovarianceStamper stamper{slipModel()};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(stamper.stamp(nan, 0.0, 0.0).rejected);
  EXPECT_TRUE(stamper.stamp(0.0, 0.0, std::numeric_limits<double>::infinity()).rejected);
}

TEST(CovarianceStamper, RejectsInvalidModels)
{
  auto negative = slipModel();
  negative.pose_stddev[1] = -0.1;
  EXPECT_THROW(CovarianceStamper{negative}, std::invalid_argument);

  auto nan_stddev = slipModel();
  nan_stddev.twist_stddev_per_mps[0] = std::nan("");
  EXPECT_THROW(CovarianceStamper{nan_stddev}, std::invalid_argument);

  auto zero_limit = slipModel();
  zero_limit.max_plausible_speed = 0.0;
  EXPECT_THROW(CovarianceStamper{zero_limit}, std::invalid_argument);
}

TEST(ImuCovariances, BuildsTheThreeMatrices)
{
  ImuNoiseModel model;
  model.orientation_stddev = {0.1, 0.1, 0.05};
  model.angular_velocity_stddev = {0.002, 0.002, 0.002};
  model.linear_acceleration_stddev = {0.01, 0.01, 0.01};
  const auto out = imuCovariances(model);
  EXPECT_DOUBLE_EQ(out.orientation[8], 0.05 * 0.05);
  EXPECT_DOUBLE_EQ(out.angular_velocity[8], 0.002 * 0.002);
  EXPECT_DOUBLE_EQ(out.linear_acceleration[0], 0.01 * 0.01);
}

TEST(ImuCovariances, RejectsNegativeStddev)
{
  ImuNoiseModel model;
  model.angular_velocity_stddev[2] = -1.0;
  EXPECT_THROW(imuCovariances(model), std::invalid_argument);
}
