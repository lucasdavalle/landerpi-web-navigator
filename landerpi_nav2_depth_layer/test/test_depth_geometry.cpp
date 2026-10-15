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
#include <cstdint>
#include <random>
#include <stdexcept>
#include <vector>

#include "Eigen/Geometry"

#include "landerpi_nav2_depth_layer/depth_projector.hpp"
#include "landerpi_nav2_depth_layer/floor_calibrator.hpp"
#include "landerpi_nav2_depth_layer/height_classifier.hpp"
#include "landerpi_nav2_depth_layer/observation_cloud.hpp"

using landerpi_nav2_depth_layer::CameraMount;
using landerpi_nav2_depth_layer::Classified;
using landerpi_nav2_depth_layer::FloorFitConfig;
using landerpi_nav2_depth_layer::Intrinsics;
using landerpi_nav2_depth_layer::ProjectionConfig;
using landerpi_nav2_depth_layer::RobotEnvelope;
using landerpi_nav2_depth_layer::bodyFromOptical;
using landerpi_nav2_depth_layer::classify;
using landerpi_nav2_depth_layer::fitFloor;
using landerpi_nav2_depth_layer::fromCloudPoints;
using landerpi_nav2_depth_layer::mountFromFloor;
using landerpi_nav2_depth_layer::projectDepth;
using landerpi_nav2_depth_layer::toCloudPoints;

namespace
{

constexpr double kDeg = M_PI / 180.0;
constexpr int kW = 640;
constexpr int kH = 400;
// The real Aurora930 intrinsics (camera_info, 2026-10-15).
const Intrinsics kK{421.9, 422.8, 317.2, 200.5};

// A camera at (x, y, height) looking down `pitch` and rolled `roll`: the
// ground-truth pose the tests try to recover.
Eigen::Isometry3d truePose(double x, double height, double pitch, double roll)
{
  return Eigen::Translation3d(x, 0.0, height) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *   // nose down
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
         Eigen::Quaterniond(bodyFromOptical());
}

// Ray-casts every pixel onto the floor (z = 0) and returns the depth image
// (mm) the camera would publish. Pixels that miss the floor stay 0.
std::vector<uint16_t> renderFloor(const Eigen::Isometry3d & base_from_optical)
{
  std::vector<uint16_t> img(kW * kH, 0);
  const Eigen::Vector3d o = base_from_optical.translation();
  for (int v = 0; v < kH; ++v) {
    for (int u = 0; u < kW; ++u) {
      const Eigen::Vector3d ray((u - kK.cx) / kK.fx, (v - kK.cy) / kK.fy, 1.0);  // z_opt = 1
      const Eigen::Vector3d d = base_from_optical.linear() * ray;
      if (d.z() >= -1e-6) {
        continue;  // looks at or above the horizon
      }
      const double t = -o.z() / d.z();  // depth along optical z
      if (t < 4.0) {
        img[v * kW + u] = static_cast<uint16_t>(std::lround(t * 1000.0));
      }
    }
  }
  return img;
}

}  // namespace

TEST(DepthProjector, CentrePixelLooksStraightAhead)
{
  std::vector<uint16_t> img(kW * kH, 0);
  const int u = 317;
  const int v = 200;
  img[v * kW + u] = 1500;
  ProjectionConfig cfg;
  cfg.stride = 1;
  const auto pts = projectDepth(img, kW, kH, kK, cfg);
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_NEAR(pts[0].z(), 1.5, 1e-9);
  EXPECT_NEAR(pts[0].x(), (u - kK.cx) * 1.5 / kK.fx, 1e-9);
  EXPECT_NEAR(pts[0].y(), (v - kK.cy) * 1.5 / kK.fy, 1e-9);
}

TEST(DepthProjector, SkipsNoReadingOutOfRangeAndUpperRows)
{
  std::vector<uint16_t> img(kW * kH, 1000);
  img[0] = 0;         // no reading
  img[1] = 100;       // closer than min_range
  img[2] = 9000;      // farther than max_range
  ProjectionConfig cfg;
  cfg.stride = 1;
  EXPECT_EQ(projectDepth(img, kW, kH, kK, cfg).size(), static_cast<std::size_t>(kW * kH - 3));
  cfg.min_row_fraction = 0.5;
  EXPECT_EQ(projectDepth(img, kW, kH, kK, cfg).size(), static_cast<std::size_t>(kW * kH / 2));
  cfg.min_row_fraction = 0.0;
  cfg.stride = 4;
  EXPECT_EQ(projectDepth(img, kW, kH, kK, cfg).size(), static_cast<std::size_t>(160 * 100 - 1));
}

TEST(DepthProjector, RejectsBadInput)
{
  std::vector<uint16_t> img(10, 0);
  EXPECT_THROW(projectDepth(img, kW, kH, kK, ProjectionConfig{}), std::invalid_argument);
  std::vector<uint16_t> ok(kW * kH, 0);
  ProjectionConfig bad;
  bad.stride = 0;
  EXPECT_THROW(projectDepth(ok, kW, kH, kK, bad), std::invalid_argument);
}

TEST(FloorCalibrator, RecoversHeightPitchAndRoll)
{
  const auto truth = truePose(0.08, 0.30, 25.0 * kDeg, 3.0 * kDeg);
  const auto img = renderFloor(truth);
  ProjectionConfig pc;
  pc.min_row_fraction = 0.55;
  const auto pts = projectDepth(img, kW, kH, kK, pc);
  const auto fit = fitFloor(pts, FloorFitConfig{});
  ASSERT_TRUE(fit.has_value());
  const CameraMount m = mountFromFloor(*fit, 0.08, 0.0, 0.0);
  // Depth is rounded to whole millimetres: a few mm / tenths of a degree.
  EXPECT_NEAR(m.height, 0.30, 0.003);
  EXPECT_NEAR(m.pitch, 25.0 * kDeg, 0.3 * kDeg);
  EXPECT_NEAR(std::abs(m.roll), 3.0 * kDeg, 0.3 * kDeg);
}

TEST(FloorCalibrator, CalibratedFloorLiesAtZeroHeight)
{
  const auto truth = truePose(0.08, 0.289, 22.3 * kDeg, 2.8 * kDeg);  // the real high posture
  ProjectionConfig pc;
  pc.min_row_fraction = 0.55;
  const auto pts = projectDepth(renderFloor(truth), kW, kH, kK, pc);
  const auto fit = fitFloor(pts, FloorFitConfig{});
  ASSERT_TRUE(fit.has_value());
  const auto m = mountFromFloor(*fit, 0.08, 0.0, 0.0);
  double worst = 0.0;
  for (const auto & p : pts) {
    worst = std::max(worst, std::abs((m.base_from_optical * p).z()));
  }
  EXPECT_LT(worst, 0.005);  // every floor point within 5 mm of z = 0
}

TEST(FloorCalibrator, IgnoresAnObstacleInView)
{
  const auto truth = truePose(0.08, 0.30, 25.0 * kDeg, 0.0);
  ProjectionConfig pc;
  pc.min_row_fraction = 0.55;
  auto pts = projectDepth(renderFloor(truth), kW, kH, kK, pc);
  // A box face: 2000 points 15 cm above the floor, 0.6 m ahead.
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> uy(-0.15, 0.15);
  std::uniform_real_distribution<double> uz(0.02, 0.15);
  const Eigen::Isometry3d inv = truth.inverse();
  for (int i = 0; i < 2000; ++i) {
    pts.push_back(inv * Eigen::Vector3d(0.6, uy(rng), uz(rng)));
  }
  const auto fit = fitFloor(pts, FloorFitConfig{});
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(mountFromFloor(*fit, 0.08, 0.0, 0.0).height, 0.30, 0.003);
}

TEST(FloorCalibrator, NoFloorNoFit)
{
  std::vector<Eigen::Vector3d> few(100, Eigen::Vector3d(0, 0, 1));
  EXPECT_FALSE(fitFloor(few, FloorFitConfig{}).has_value());
}

TEST(HeightClassifier, SortsPointsByWhatTheyMeanForTheRobot)
{
  RobotEnvelope robot;  // 1.5 cm floor band, 31 cm tall, 5 cm margin
  const std::vector<Eigen::Vector3d> pts{
    {0.8, 0.0, 0.005},   // floor
    {0.8, 0.1, 0.05},    // a shoe: lower than the lidar (9 cm), the robot would hit it
    {0.9, 0.0, 0.28},    // a table edge just under the robot's height
    {0.9, 0.0, 0.40},    // a table top higher than robot + margin: passes under
    {0.10, 0.0, 0.20},   // the robot's own arm
    {3.5, 0.0, 0.10},    // too far
  };
  const auto c = classify(pts, robot);
  EXPECT_EQ(c.floor.size(), 1u);
  EXPECT_EQ(c.obstacles.size(), 2u);
  EXPECT_EQ(c.above, 1u);
  EXPECT_EQ(c.own_body, 1u);
  EXPECT_EQ(c.too_far, 1u);
}

// The box the robot dragged in the first 6b test: 3 cm tall, its points read
// 1.5-2.6 cm over the calibrated floor (the old 3 cm floor band hid it).
TEST(HeightClassifier, ThreeCentimetreBoxIsAnObstacle)
{
  RobotEnvelope robot;
  const std::vector<Eigen::Vector3d> box{{0.7, 0.0, 0.016}, {0.7, 0.02, 0.026}};
  EXPECT_EQ(classify(box, robot).obstacles.size(), 2u);
}

// The bed edge the camera ran into: 30 cm, seen at the top of the image
// with noise up to 33 cm, while the robot stands 31 cm tall.
TEST(HeightClassifier, NoisyBedEdgeAtCameraHeightIsAnObstacle)
{
  RobotEnvelope robot;
  const std::vector<Eigen::Vector3d> bed_edge{{0.9, 0.0, 0.30}, {0.9, 0.0, 0.33}};
  EXPECT_EQ(classify(bed_edge, robot).obstacles.size(), 2u);
}

TEST(HeightClassifier, LowerPostureLetsMoreThroughUnder)
{
  const std::vector<Eigen::Vector3d> bed_frame{{0.9, 0.0, 0.30}};  // the bed: 30 cm clearance
  RobotEnvelope high;
  high.height = 0.31;
  RobotEnvelope low;
  low.height = 0.22;  // + 5 cm margin = 27 cm, still under the bed
  EXPECT_EQ(classify(bed_frame, high).obstacles.size(), 1u);  // standing: would hit it
  EXPECT_EQ(classify(bed_frame, low).above, 1u);              // crouched: passes under
}

TEST(HeightClassifier, RejectsBadEnvelope)
{
  RobotEnvelope bad;
  bad.floor_band = 0.5;  // above the robot's height
  EXPECT_THROW(classify({}, bad), std::invalid_argument);
  RobotEnvelope negative_margin;
  negative_margin.height_margin = -0.01;
  EXPECT_THROW(classify({}, negative_margin), std::invalid_argument);
}

// The global costmap's layer rebuilds the local layer's classification from
// its cloud: obstacles survive one by one, the floor one point per cell.
TEST(ObservationCloud, RoundTripKeepsObstaclesAndThinsTheFloor)
{
  Classified c;
  c.obstacles = {{0.8, 0.0}, {0.82, 0.01}};
  for (int k = 0; k < 10; ++k) {
    c.floor.emplace_back(1.0 + 0.004 * k, 0.2);  // ten points inside one 5 cm cell
  }
  c.floor.emplace_back(1.5, -0.3);  // another cell
  const auto cloud = toCloudPoints(c, 0.05);
  ASSERT_EQ(cloud.size(), 4u);
  const Classified back = fromCloudPoints(cloud);
  ASSERT_EQ(back.obstacles.size(), 2u);
  EXPECT_NEAR(back.obstacles[1].x(), 0.82, 1e-9);
  EXPECT_EQ(back.floor.size(), 2u);
}

TEST(ObservationCloud, FloorCellsAreSeparatedOnBothSidesOfZero)
{
  Classified c;
  c.floor = {{0.01, -0.01}, {0.01, 0.01}, {-0.01, 0.01}};  // three cells around the origin
  EXPECT_EQ(toCloudPoints(c, 0.05).size(), 3u);
  EXPECT_THROW(toCloudPoints(c, 0.0), std::invalid_argument);
}
