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

#include "landerpi_web_nav/covariance_stamper_node.hpp"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace landerpi_web_nav
{

namespace
{

// The inputs are all published RELIABLE by their drivers (odom_publisher,
// rf2o, imu_complementary_filter); depth 10 absorbs short bursts.
rclcpp::QoS inputQos() {return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();}
rclcpp::QoS outputQos() {return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();}

template<std::size_t N>
std::array<double, N> toArray(const std::vector<double> & values, const std::string & name)
{
  if (values.size() != N) {
    throw std::invalid_argument(
            name + " needs " + std::to_string(N) + " values, got " +
            std::to_string(values.size()));
  }
  std::array<double, N> out{};
  std::copy(values.begin(), values.end(), out.begin());
  return out;
}

}  // namespace

CovarianceStamperNode::CovarianceStamperNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("covariance_stamper", options)
{
  const auto channels = declare_parameter<std::vector<std::string>>(
    "odom_channels", std::vector<std::string>{"rf2o", "raw"});
  for (const auto & name : channels) {
    setUpOdomChannel(name);
  }
  setUpImu();

  init_finish_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/init_finish",
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = true;
    });

  RCLCPP_INFO(
    get_logger(), "covariance_stamper ready: %zu odom channel(s), imu %s",
    odom_channels_.size(), imu_covariances_ ? "on" : "off");
}

Dof6 CovarianceStamperNode::declareDof6(const std::string & name)
{
  return toArray<6>(
    declare_parameter<std::vector<double>>(name, std::vector<double>(6, 0.0)), name);
}

Dof3 CovarianceStamperNode::declareDof3(const std::string & name)
{
  return toArray<3>(
    declare_parameter<std::vector<double>>(name, std::vector<double>(3, 0.0)), name);
}

void CovarianceStamperNode::setUpOdomChannel(const std::string & name)
{
  const std::string p = name + ".";
  const auto input = declare_parameter<std::string>(p + "input_topic", "");
  const auto output = declare_parameter<std::string>(p + "output_topic", "");
  if (input.empty() || output.empty() || input == output) {
    throw std::invalid_argument(
            "channel '" + name + "' needs distinct, non-empty input_topic and output_topic");
  }

  OdomNoiseModel model;
  model.pose_stddev = declareDof6(p + "pose_stddev");
  model.twist_stddev_base = declareDof6(p + "twist_stddev_base");
  model.twist_stddev_per_mps = declareDof6(p + "twist_stddev_per_mps");
  model.twist_stddev_per_radps = declareDof6(p + "twist_stddev_per_radps");
  model.zero_velocity_threshold = declare_parameter<double>(p + "zero_velocity_threshold", -1.0);
  model.twist_stddev_at_rest = declareDof6(p + "twist_stddev_at_rest");
  model.max_plausible_speed = declare_parameter<double>(p + "max_plausible_speed", 1.0);
  model.max_plausible_yaw_rate = declare_parameter<double>(p + "max_plausible_yaw_rate", 3.0);

  auto channel = std::make_unique<OdomChannel>();
  channel->name = name;
  channel->stamper = std::make_unique<CovarianceStamper>(model);  // validates
  channel->pub = create_publisher<nav_msgs::msg::Odometry>(output, outputQos());
  OdomChannel & ref = *channel;
  channel->sub = create_subscription<nav_msgs::msg::Odometry>(
    input, inputQos(),
    [this, &ref](nav_msgs::msg::Odometry::UniquePtr msg) {onOdom(ref, std::move(msg));});
  odom_channels_.push_back(std::move(channel));

  RCLCPP_INFO(get_logger(), "channel '%s': %s -> %s", name.c_str(), input.c_str(), output.c_str());
}

void CovarianceStamperNode::setUpImu()
{
  if (!declare_parameter<bool>("imu.enabled", true)) {
    return;
  }
  const auto input = declare_parameter<std::string>("imu.input_topic", "/imu");
  const auto output = declare_parameter<std::string>("imu.output_topic", "/a7/imu");
  if (input == output) {
    throw std::invalid_argument("imu.input_topic and imu.output_topic must differ");
  }

  ImuNoiseModel model;
  model.orientation_stddev = declareDof3("imu.orientation_stddev");
  model.angular_velocity_stddev = declareDof3("imu.angular_velocity_stddev");
  model.linear_acceleration_stddev = declareDof3("imu.linear_acceleration_stddev");
  imu_covariances_ = imuCovariances(model);  // validates

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(output, outputQos());
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    input, inputQos(),
    [this](sensor_msgs::msg::Imu::UniquePtr msg) {onImu(std::move(msg));});

  RCLCPP_INFO(get_logger(), "imu: %s -> %s", input.c_str(), output.c_str());
}

void CovarianceStamperNode::onOdom(OdomChannel & channel, nav_msgs::msg::Odometry::UniquePtr msg)
{
  const auto & twist = msg->twist.twist;
  const auto cov = channel.stamper->stamp(twist.linear.x, twist.linear.y, twist.angular.z);
  ++channel.received;
  if (cov.rejected) {
    ++channel.rejected;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "channel '%s': implausible measurement (vx=%.2f vy=%.2f wz=%.2f), "
      "stamped as rejected (%lu of %lu so far)",
      channel.name.c_str(), twist.linear.x, twist.linear.y, twist.angular.z,
      static_cast<unsigned long>(channel.rejected),  // NOLINT(runtime/int)
      static_cast<unsigned long>(channel.received));  // NOLINT(runtime/int)
  }
  std::copy(cov.pose.begin(), cov.pose.end(), msg->pose.covariance.begin());
  std::copy(cov.twist.begin(), cov.twist.end(), msg->twist.covariance.begin());
  channel.pub->publish(std::move(msg));
}

void CovarianceStamperNode::onImu(sensor_msgs::msg::Imu::UniquePtr msg)
{
  const auto & cov = *imu_covariances_;
  std::copy(cov.orientation.begin(), cov.orientation.end(), msg->orientation_covariance.begin());
  std::copy(
    cov.angular_velocity.begin(), cov.angular_velocity.end(),
    msg->angular_velocity_covariance.begin());
  std::copy(
    cov.linear_acceleration.begin(), cov.linear_acceleration.end(),
    msg->linear_acceleration_covariance.begin());
  imu_pub_->publish(std::move(msg));
}

}  // namespace landerpi_web_nav
