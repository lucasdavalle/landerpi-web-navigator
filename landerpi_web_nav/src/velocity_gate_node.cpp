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

#include "landerpi_web_nav/velocity_gate_node.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"

namespace landerpi_web_nav
{

namespace
{

Velocity fromTwist(const geometry_msgs::msg::Twist & t)
{
  return {t.linear.x, t.linear.y, t.angular.z};
}

}  // namespace

double VelocityGateNode::secs()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

VelocityGateNode::VelocityGateNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("velocity_gate", options)
{
  declare_parameter<double>("max_vx", 0.25);
  declare_parameter<double>("max_vy", 0.25);
  declare_parameter<double>("max_wz", 1.0);
  declare_parameter<double>("max_linear_accel", 0.5);
  declare_parameter<double>("max_angular_accel", 2.0);
  declare_parameter<double>("heartbeat_timeout", 0.5);
  declare_parameter<double>("command_timeout", 0.3);
  declare_parameter<double>("rate_hz", 20.0);
  declare_parameter<std::string>("output_topic", "/controller/cmd_vel");
  // Stage 6c: zero unless posture_manager reports the arm settled ("high"
  // or "low"). Off for setups without posture_manager.
  declare_parameter<bool>("require_posture", false);

  init_finish_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/init_finish",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success =
      get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    });
}

VelocityGateNode::CallbackReturn VelocityGateNode::on_configure(const rclcpp_lifecycle::State &)
{
  VelocityGateConfig cfg;
  cfg.max_vx = get_parameter("max_vx").as_double();
  cfg.max_vy = get_parameter("max_vy").as_double();
  cfg.max_wz = get_parameter("max_wz").as_double();
  cfg.max_linear_accel = get_parameter("max_linear_accel").as_double();
  cfg.max_angular_accel = get_parameter("max_angular_accel").as_double();
  cfg.heartbeat_timeout = get_parameter("heartbeat_timeout").as_double();
  cfg.command_timeout = get_parameter("command_timeout").as_double();
  // Hard ceilings on top of the YAML: a typo must not make the robot fast.
  if (cfg.max_vx > 0.5 || cfg.max_vy > 0.5 || cfg.max_wz > 2.0) {
    RCLCPP_ERROR(get_logger(), "speed limits above the hard ceiling (0.5 m/s, 2.0 rad/s)");
    return CallbackReturn::FAILURE;
  }
  const double rate = get_parameter("rate_hz").as_double();
  if (!(rate >= 10.0 && rate <= 50.0)) {
    RCLCPP_ERROR(get_logger(), "rate_hz must be in [10, 50]");
    return CallbackReturn::FAILURE;
  }
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    gate_ = std::make_unique<VelocityGate>(cfg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "bad limits: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  const auto reliable = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    get_parameter("output_topic").as_string(), reliable);
  status_pub_ = create_publisher<std_msgs::msg::String>(
    "/a7/gate_status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/a7/cmd_vel_teleop", reliable, [this](geometry_msgs::msg::Twist::SharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);
      gate_->teleop(fromTwist(*m), secs());
    });
  nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/a7/cmd_vel_nav", reliable, [this](geometry_msgs::msg::Twist::SharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);
      gate_->nav(fromTwist(*m), secs());
    });
  if (get_parameter("require_posture").as_bool()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      gate_->setArmSettled(false);  // until posture_manager says otherwise
    }
    posture_sub_ = create_subscription<std_msgs::msg::String>(
      "/a7/posture", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        gate_->setArmSettled(m->data == "high" || m->data == "low");
      });
  }
  heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
    "/a7/heartbeat", reliable, [this](std_msgs::msg::Empty::SharedPtr) {
      std::lock_guard<std::mutex> lock(mutex_);
      gate_->heartbeat(secs());
    });

  set_running_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/set_running",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
      if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        res->success = false;
        res->message = "velocity_gate is not active";
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      gate_->setEnabled(req->data);
      res->success = true;
      res->message = req->data ? "enabled" : "disabled";
      RCLCPP_INFO(get_logger(), "motion %s", res->message.c_str());
    });

  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {onTimer();});
  timer_->cancel();  // runs only while active
  return CallbackReturn::SUCCESS;
}

VelocityGateNode::CallbackReturn VelocityGateNode::on_activate(const rclcpp_lifecycle::State &)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gate_->setEnabled(false);  // activating makes it ready, never moving
  }
  cmd_pub_->on_activate();
  status_pub_->on_activate();
  timer_->reset();
  RCLCPP_INFO(get_logger(), "active, idle until ~/set_running(true)");
  return CallbackReturn::SUCCESS;
}

void VelocityGateNode::publishZero()
{
  if (cmd_pub_ && cmd_pub_->is_activated()) {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }
}

VelocityGateNode::CallbackReturn VelocityGateNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_->cancel();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gate_->setEnabled(false);
  }
  publishZero();  // before the publisher goes quiet
  cmd_pub_->on_deactivate();
  status_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

VelocityGateNode::CallbackReturn VelocityGateNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  set_running_srv_.reset();
  teleop_sub_.reset();
  nav_sub_.reset();
  heartbeat_sub_.reset();
  posture_sub_.reset();
  cmd_pub_.reset();
  status_pub_.reset();
  gate_.reset();
  return CallbackReturn::SUCCESS;
}

VelocityGateNode::CallbackReturn VelocityGateNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  stopNow();
  return CallbackReturn::SUCCESS;
}

void VelocityGateNode::stopNow()
{
  if (timer_) {
    timer_->cancel();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (gate_) {
      gate_->setEnabled(false);
    }
  }
  publishZero();  // a kill mid-manoeuvre must not leave the wheels running
}

void VelocityGateNode::onTimer()
{
  GateOutput out;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    out = gate_->update(secs());
  }
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = out.cmd.vx;
  cmd.linear.y = out.cmd.vy;
  cmd.angular.z = out.cmd.wz;
  cmd_pub_->publish(cmd);

  const auto reason = toString(out.reason);
  const double t = secs();
  if (reason != last_reason_ || t - last_status_time_ >= 1.0) {
    std_msgs::msg::String s;
    s.data = reason;
    status_pub_->publish(s);
    if (reason != last_reason_) {
      RCLCPP_INFO(get_logger(), "gate: %s", reason.c_str());
    }
    last_reason_ = reason;
    last_status_time_ = t;
  }
}

}  // namespace landerpi_web_nav
