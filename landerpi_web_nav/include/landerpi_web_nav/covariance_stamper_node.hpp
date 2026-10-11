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

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "landerpi_web_nav/covariance_stamper.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace landerpi_web_nav
{

// Republishes odometry/IMU topics with realistic covariances so the EKF
// (robot_localization) can weigh them. Plain rclcpp::Node, not lifecycle: it
// is a stateless pass-through that never commands an actuator.
//
// Parameters (see config/covariance_stamper.yaml):
//   odom_channels: names of the odometry channels, e.g. ["rf2o", "raw"]
//   <channel>.input_topic / <channel>.output_topic
//   <channel>.pose_stddev / twist_stddev_base (6 values)
//   <channel>.twist_stddev_per_mps / twist_stddev_per_radps (6 values)
//   <channel>.max_plausible_speed / max_plausible_yaw_rate
//   imu.enabled, imu.input_topic / imu.output_topic
//   imu.orientation_stddev / angular_velocity_stddev / linear_acceleration_stddev (3)
class CovarianceStamperNode : public rclcpp::Node
{
public:
  explicit CovarianceStamperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct OdomChannel
  {
    std::string name;
    std::unique_ptr<CovarianceStamper> stamper;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub;
    uint64_t received = 0;
    uint64_t rejected = 0;
  };

  void setUpOdomChannel(const std::string & name);
  void setUpImu();
  void onOdom(OdomChannel & channel, nav_msgs::msg::Odometry::UniquePtr msg);
  void onImu(sensor_msgs::msg::Imu::UniquePtr msg);

  Dof6 declareDof6(const std::string & name);
  Dof3 declareDof3(const std::string & name);

  // unique_ptr so each channel keeps a stable address for its callback.
  std::vector<std::unique_ptr<OdomChannel>> odom_channels_;

  std::optional<ImuCovariances> imu_covariances_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_finish_srv_;
};

}  // namespace landerpi_web_nav
