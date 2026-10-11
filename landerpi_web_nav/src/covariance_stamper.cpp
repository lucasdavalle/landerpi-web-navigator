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

#include "landerpi_web_nav/covariance_stamper.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace landerpi_web_nav
{

namespace
{

template<std::size_t N>
void validateStddevs(const std::array<double, N> & stddev, const std::string & name)
{
  for (std::size_t i = 0; i < N; ++i) {
    if (!std::isfinite(stddev[i]) || stddev[i] < 0.0) {
      throw std::invalid_argument(
              name + "[" + std::to_string(i) + "] must be finite and >= 0, got " +
              std::to_string(stddev[i]));
    }
  }
}

void validatePositive(double value, const std::string & name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(name + " must be finite and > 0, got " + std::to_string(value));
  }
}

template<std::size_t N>
std::array<double, N * N> diagonal(const std::array<double, N> & stddev)
{
  std::array<double, N * N> covariance{};  // zero-initialized off-diagonal
  for (std::size_t i = 0; i < N; ++i) {
    covariance[i * N + i] = stddev[i] * stddev[i];
  }
  return covariance;
}

}  // namespace

Covariance6 diagonalCovariance(const Dof6 & stddev) {return diagonal(stddev);}

Covariance3 diagonalCovariance(const Dof3 & stddev) {return diagonal(stddev);}

CovarianceStamper::CovarianceStamper(const OdomNoiseModel & model)
: model_(model)
{
  validateStddevs(model_.pose_stddev, "pose_stddev");
  validateStddevs(model_.twist_stddev_base, "twist_stddev_base");
  validateStddevs(model_.twist_stddev_per_mps, "twist_stddev_per_mps");
  validateStddevs(model_.twist_stddev_per_radps, "twist_stddev_per_radps");
  validatePositive(model_.max_plausible_speed, "max_plausible_speed");
  validatePositive(model_.max_plausible_yaw_rate, "max_plausible_yaw_rate");
  pose_covariance_ = diagonalCovariance(model_.pose_stddev);
}

OdomCovariances CovarianceStamper::stamp(double vx, double vy, double wz) const
{
  OdomCovariances out;
  const double speed = std::hypot(vx, vy);

  // NaN compares false against everything, so check finiteness explicitly
  // instead of relying on the > comparisons below to catch it.
  const bool finite = std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz);
  if (!finite || speed > model_.max_plausible_speed ||
    std::abs(wz) > model_.max_plausible_yaw_rate)
  {
    out.rejected = true;
    for (std::size_t i = 0; i < 6; ++i) {
      out.pose[i * 6 + i] = kRejectedVariance;
      out.twist[i * 6 + i] = kRejectedVariance;
    }
    return out;
  }

  out.pose = pose_covariance_;
  const double yaw_rate = std::abs(wz);
  Dof6 twist_stddev{};
  for (std::size_t i = 0; i < 6; ++i) {
    twist_stddev[i] = model_.twist_stddev_base[i] +
      model_.twist_stddev_per_mps[i] * speed +
      model_.twist_stddev_per_radps[i] * yaw_rate;
  }
  out.twist = diagonalCovariance(twist_stddev);
  return out;
}

ImuCovariances imuCovariances(const ImuNoiseModel & model)
{
  validateStddevs(model.orientation_stddev, "orientation_stddev");
  validateStddevs(model.angular_velocity_stddev, "angular_velocity_stddev");
  validateStddevs(model.linear_acceleration_stddev, "linear_acceleration_stddev");
  ImuCovariances out;
  out.orientation = diagonalCovariance(model.orientation_stddev);
  out.angular_velocity = diagonalCovariance(model.angular_velocity_stddev);
  out.linear_acceleration = diagonalCovariance(model.linear_acceleration_stddev);
  return out;
}

}  // namespace landerpi_web_nav
