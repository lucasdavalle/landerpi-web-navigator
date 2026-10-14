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

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "landerpi_web_nav/velocity_gate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace landerpi_web_nav
{

// The only node that publishes to the wheels (/controller/cmd_vel), at a
// fixed rate, through VelocityGate: teleop over Nav2, clamping, ramps and a
// deadman on /a7/heartbeat. Starts idle (publishing zero) until
// ~/set_running(true). Publishes zero before deactivating or shutting down:
// the stock driver has no watchdog of its own.
//
// Topics: in /a7/cmd_vel_teleop, /a7/cmd_vel_nav (Twist), /a7/heartbeat
// (Empty); out /controller/cmd_vel (Twist), /a7/gate_status (String: the
// GateReason, on change and at 1 Hz).
class VelocityGateNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit VelocityGateNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  // Disables the gate and publishes zero right now. Called from a
  // pre-shutdown callback (see main): in Humble, Ctrl+C does NOT run the
  // lifecycle on_shutdown by itself, so relying on it alone would leave the
  // wheels with the last command.
  void stopNow();

private:
  // Monotonic seconds: the deadman must not depend on ROS time behaving.
  static double secs();
  void onTimer();
  void publishZero();

  std::mutex mutex_;
  std::unique_ptr<VelocityGate> gate_;
  std::string last_reason_;
  double last_status_time_ = 0.0;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_running_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_finish_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace landerpi_web_nav
