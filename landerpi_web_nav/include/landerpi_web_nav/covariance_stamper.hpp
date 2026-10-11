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

namespace landerpi_web_nav
{

// Standard deviations for the 6 degrees of freedom, in the order ROS uses for
// pose/twist covariances: x, y, z, roll, pitch, yaw (m, m/s, rad, rad/s).
using Dof6 = std::array<double, 6>;
using Dof3 = std::array<double, 3>;

// Row-major covariance matrices, same layout as the `covariance` fields of
// geometry_msgs (6x6) and sensor_msgs/Imu (3x3), so they can be copied
// straight into a message.
using Covariance6 = std::array<double, 36>;
using Covariance3 = std::array<double, 9>;

// Noise of an odometry source (nav_msgs/Odometry).
//
// None of the sources on this robot report real covariances (see CLAUDE.md,
// "Hallazgo etapa 1"): rf2o publishes all zeros and odom_raw ~zeros, which
// robot_localization reads as near-certainty. This model replaces them.
struct OdomNoiseModel
{
  // Fixed noise of the pose fields.
  Dof6 pose_stddev{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // Twist noise at rest...
  Dof6 twist_stddev_base{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // ...plus this much per m/s of planar speed: wheel slip on the mecanum
  // chassis grows with speed, and odom_raw (integrated from the commanded
  // cmd_vel) cannot see it.
  Dof6 twist_stddev_per_mps{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // ...plus this much per rad/s of yaw rate: turning in place has zero
  // planar speed but is where the mecanum wheels slip the most (stage 0:
  // odom_raw ended 78 deg off).
  Dof6 twist_stddev_per_radps{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // Zero-velocity update (ZUPT). When |vx|, |vy| and |wz| are all at or
  // below zero_velocity_threshold, the robot is known to be still and the
  // twist is stamped with twist_stddev_at_rest instead of the speed model.
  // Meant for odom_raw: it is the command itself, so an exact zero means the
  // motors are off and the robot cannot be moving (unless pushed); a tight
  // covariance there keeps the EKF from integrating rf2o's jitter at rest
  // (stage 1: 62 cm of fake motion in 70 s still). A negative threshold
  // disables it (the default).
  double zero_velocity_threshold = -1.0;
  Dof6 twist_stddev_at_rest{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // Physical plausibility gate: a measurement faster than this is treated as
  // a glitch (rf2o occasionally jumps) and stamped with kRejectedVariance so
  // the EKF effectively ignores it. The real chassis tops out well below.
  double max_plausible_speed = 1.0;      // m/s
  double max_plausible_yaw_rate = 3.0;   // rad/s
};

// Noise of an IMU (sensor_msgs/Imu): one value per axis.
struct ImuNoiseModel
{
  Dof3 orientation_stddev{0.0, 0.0, 0.0};          // rad
  Dof3 angular_velocity_stddev{0.0, 0.0, 0.0};     // rad/s
  Dof3 linear_acceleration_stddev{0.0, 0.0, 0.0};  // m/s^2
};

struct OdomCovariances
{
  Covariance6 pose{};
  Covariance6 twist{};
  // True when the measurement failed the plausibility gate.
  bool rejected = false;
  // True when the zero-velocity update applied.
  bool at_rest = false;
};

struct ImuCovariances
{
  Covariance3 orientation{};
  Covariance3 angular_velocity{};
  Covariance3 linear_acceleration{};
};

// Diagonal covariance from standard deviations (variance = stddev^2).
Covariance6 diagonalCovariance(const Dof6 & stddev);
Covariance3 diagonalCovariance(const Dof3 & stddev);

// Computes the covariances to stamp on each odometry message.
class CovarianceStamper
{
public:
  // Variance used for a rejected measurement: large enough that the EKF
  // gives it no weight, finite so it never produces NaNs downstream.
  static constexpr double kRejectedVariance = 1e6;

  // Throws std::invalid_argument if any stddev is negative or not finite, or
  // a plausibility limit is not strictly positive.
  explicit CovarianceStamper(const OdomNoiseModel & model);

  // vx, vy (m/s) and wz (rad/s) are the twist of the incoming message.
  OdomCovariances stamp(double vx, double vy, double wz) const;

  const OdomNoiseModel & model() const {return model_;}

private:
  OdomNoiseModel model_;
  Covariance6 pose_covariance_;  // constant, precomputed once
};

// Validates the model and builds the (constant) IMU covariances. Throws
// std::invalid_argument on a negative or non-finite stddev.
ImuCovariances imuCovariances(const ImuNoiseModel & model);

}  // namespace landerpi_web_nav
