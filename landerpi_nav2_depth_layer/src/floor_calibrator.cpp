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

#include "landerpi_nav2_depth_layer/floor_calibrator.hpp"

#include <cmath>
#include <random>
#include <vector>

#include "Eigen/Dense"

namespace landerpi_nav2_depth_layer
{

Eigen::Matrix3d bodyFromOptical()
{
  Eigen::Matrix3d r;
  // body x = optical z, body y = -optical x, body z = -optical y
  r << 0, 0, 1,
    -1, 0, 0,
    0, -1, 0;
  return r;
}

std::optional<FloorFit> fitFloor(
  const std::vector<Eigen::Vector3d> & points, const FloorFitConfig & config)
{
  if (points.size() < 3 || points.size() < config.min_inliers) {
    return std::nullopt;
  }
  std::mt19937 rng(config.seed);
  std::uniform_int_distribution<std::size_t> pick(0, points.size() - 1);

  // 1) RANSAC: the plane through 3 random points that most points agree with.
  std::size_t best_count = 0;
  Eigen::Vector3d best_n;
  Eigen::Vector3d best_p;
  for (int it = 0; it < config.iterations; ++it) {
    const auto & a = points[pick(rng)];
    const auto & b = points[pick(rng)];
    const auto & c = points[pick(rng)];
    Eigen::Vector3d n = (b - a).cross(c - a);
    if (n.norm() < 1e-9) {
      continue;  // degenerate (collinear or repeated) sample
    }
    n.normalize();
    std::size_t count = 0;
    for (const auto & p : points) {
      count += std::abs((p - a).dot(n)) < config.inlier_distance;
    }
    if (count > best_count) {
      best_count = count;
      best_n = n;
      best_p = a;
    }
  }
  if (best_count < config.min_inliers) {
    return std::nullopt;
  }

  // 2) Least squares on the inliers: the plane normal is the direction of
  //    least variance (smallest singular vector of the centred points).
  std::vector<Eigen::Vector3d> inl;
  inl.reserve(best_count);
  for (const auto & p : points) {
    if (std::abs((p - best_p).dot(best_n)) < config.inlier_distance) {
      inl.push_back(p);
    }
  }
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto & p : inl) {
    centroid += p;
  }
  centroid /= static_cast<double>(inl.size());
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto & p : inl) {
    const Eigen::Vector3d d = p - centroid;
    cov += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
  Eigen::Vector3d n = eig.eigenvectors().col(0);  // smallest eigenvalue first
  // Up in the optical frame is -y (y points down in the image).
  if (n.y() > 0.0) {
    n = -n;
  }

  FloorFit fit;
  fit.normal = n;
  fit.centroid = centroid;
  fit.inliers = inl.size();
  double sq = 0.0;
  for (const auto & p : inl) {
    const double d = (p - centroid).dot(n);
    sq += d * d;
  }
  fit.rms = std::sqrt(sq / static_cast<double>(inl.size()));
  return fit;
}

CameraMount mountFromFloor(const FloorFit & floor, double x, double y, double yaw)
{
  // Floor normal expressed in an un-tilted camera body frame.
  const Eigen::Vector3d b = bodyFromOptical() * floor.normal;
  // The smallest rotation that turns it into "up": that is exactly the
  // camera's tilt (pitch + roll), and it adds no yaw of its own.
  const Eigen::Quaterniond level = Eigen::Quaterniond::FromTwoVectors(b, Eigen::Vector3d::UnitZ());

  CameraMount m;
  m.height = std::abs(floor.normal.dot(floor.centroid));
  m.pitch = std::atan2(-floor.normal.z(), -floor.normal.y());
  m.roll = std::atan2(floor.normal.x(), -floor.normal.y());
  m.base_from_optical =
    Eigen::Translation3d(x, y, m.height) *
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    level *
    Eigen::Quaterniond(bodyFromOptical());
  return m;
}

}  // namespace landerpi_nav2_depth_layer
