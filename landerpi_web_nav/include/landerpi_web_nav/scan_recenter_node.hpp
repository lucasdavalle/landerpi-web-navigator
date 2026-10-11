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
#include <optional>

#include "landerpi_web_nav/scan_recenter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace landerpi_web_nav
{

// Republishes a full-circle LaserScan rotated to [-pi, pi) so
// rf2o_laser_odometry reads it correctly (see scan_recenter.hpp). Plain
// rclcpp::Node: stateless pass-through, commands no actuator.
//
// Parameters: input_topic (/scan_raw), output_topic (/a7/scan_centered).
class ScanRecenterNode : public rclcpp::Node
{
public:
  explicit ScanRecenterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onScan(sensor_msgs::msg::LaserScan::UniquePtr msg);

  // The lidar alternates between 501 and 502 beams per scan, so the plan is
  // cached per layout and recomputed when it changes.
  std::optional<ScanLayout> layout_;
  RecenterPlan plan_;
  uint64_t dropped_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_finish_srv_;
};

}  // namespace landerpi_web_nav
