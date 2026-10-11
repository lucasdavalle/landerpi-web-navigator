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

#include "landerpi_web_nav/scan_recenter_node.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace landerpi_web_nav
{

ScanRecenterNode::ScanRecenterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("scan_recenter", options)
{
  const auto input = declare_parameter<std::string>("input_topic", "/scan_raw");
  const auto output = declare_parameter<std::string>("output_topic", "/a7/scan_centered");
  if (input.empty() || output.empty() || input == output) {
    throw std::invalid_argument("input_topic and output_topic must be distinct and non-empty");
  }

  // /scan_raw is subscribed BEST_EFFORT, as everywhere in the portfolio
  // (compatible with the driver's RELIABLE publisher). The output is
  // RELIABLE so both reliable (rf2o) and best-effort readers can match.
  sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    input, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {onScan(std::move(msg));});
  pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
    output, rclcpp::QoS(rclcpp::KeepLast(5)).reliable());

  init_finish_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/init_finish",
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = true;
    });

  RCLCPP_INFO(get_logger(), "scan_recenter: %s -> %s", input.c_str(), output.c_str());
}

void ScanRecenterNode::onScan(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  const ScanLayout layout{msg->angle_min, msg->angle_increment, msg->ranges.size()};
  const bool changed = !layout_ || layout_->angle_min != layout.angle_min ||
    layout_->angle_increment != layout.angle_increment || layout_->count != layout.count;
  if (changed) {
    try {
      plan_ = planRecenter(layout);
      layout_ = layout;
    } catch (const std::invalid_argument & e) {
      // Not publishing is safer than publishing a scan rf2o would misread.
      layout_.reset();
      ++dropped_;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "dropping scan (%lu so far): %s",
        static_cast<unsigned long>(dropped_), e.what());  // NOLINT(runtime/int)
      return;
    }
  }

  applyRecenter(plan_, msg->ranges);
  applyRecenter(plan_, msg->intensities);
  msg->angle_min = static_cast<float>(plan_.angle_min);
  msg->angle_max = static_cast<float>(plan_.angle_max);
  pub_->publish(std::move(msg));
}

}  // namespace landerpi_web_nav
