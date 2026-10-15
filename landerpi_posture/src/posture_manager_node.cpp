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

#include "landerpi_posture/posture_manager_node.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "landerpi_nav2_depth_layer/depth_projector.hpp"
#include "landerpi_nav2_depth_layer/floor_calibrator.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace landerpi_posture
{

namespace depth = landerpi_nav2_depth_layer;

PostureManagerNode::PostureManagerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("posture_manager", options)
{
  init_finish_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/init_finish",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      res->success =
      get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    });
}

Posture PostureManagerNode::declarePosture(const std::string & name)
{
  const std::string p = "postures." + name + ".";
  Posture out;
  out.name = name;
  const auto pulses = declare_parameter<std::vector<int64_t>>(
    p + "pulses", {500, 500, 500, 500,
      500});
  if (pulses.size() != 5) {
    throw std::invalid_argument(p + "pulses needs 5 values");
  }
  for (std::size_t i = 0; i < 5; ++i) {
    out.pulses[i] = static_cast<int>(pulses[i]);
  }
  out.pulses = clampPulses(out.pulses);
  out.height = declare_parameter<double>(p + "height", out.height);
  out.front = declare_parameter<double>(p + "front", out.front);
  out.back = declare_parameter<double>(p + "back", out.back);
  out.half_width = declare_parameter<double>(p + "half_width", out.half_width);
  out.cam_height = declare_parameter<double>(p + "cam_height", out.cam_height);
  out.cam_pitch = declare_parameter<double>(p + "cam_pitch", out.cam_pitch);
  out.cam_roll = declare_parameter<double>(p + "cam_roll", out.cam_roll);
  out.calibrated = declare_parameter<bool>(p + "calibrated", out.calibrated);
  if (!(out.height > 0.05 && out.height < 1.0) || !(out.front > out.back)) {
    throw std::invalid_argument("bad posture " + name);
  }
  return out;
}

PostureManagerNode::CallbackReturn PostureManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    for (const auto & name : {"high", "low"}) {
      postures_[name] = declarePosture(name);
    }
    move_duration_ = declare_parameter<double>("move_duration", 3.0);
    settle_ = declare_parameter<double>("settle", 0.7);
    calibration_timeout_ = declare_parameter<double>("calibration_timeout", 2.0);
    if (!(move_duration_ >= 1.0 && move_duration_ <= 10.0)) {
      throw std::invalid_argument("move_duration must be in [1, 10] s (slow, on purpose)");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "configure failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  servo_pub_ = create_publisher<servo_controller_msgs::msg::ServosPosition>(
    "servo_controller", rclcpp::QoS(10).reliable());
  height_pub_ = create_publisher<std_msgs::msg::Float32>("/a7/robot_height", latched);
  state_pub_ = create_publisher<std_msgs::msg::String>("/a7/posture", latched);
  local_fp_pub_ = create_publisher<geometry_msgs::msg::Polygon>(
    "/local_costmap/footprint", rclcpp::QoS(1).reliable());
  global_fp_pub_ = create_publisher<geometry_msgs::msg::Polygon>(
    "/global_costmap/footprint", rclcpp::QoS(1).reliable());

  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/ascamera/camera_publisher/depth0/image_raw", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Image::ConstSharedPtr m) {onDepth(std::move(m));});
  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/ascamera/camera_publisher/depth0/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);
      intrinsics_ = std::array<double, 4>{m->k[0], m->k[4], m->k[2], m->k[5]};
    });

  auto handler = [this](const std::string & name) {
      return [this, name](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
               res->success = request(name, res->message);
             };
    };
  go_high_srv_ = create_service<std_srvs::srv::Trigger>("~/go_high", handler("high"));
  go_low_srv_ = create_service<std_srvs::srv::Trigger>("~/go_low", handler("low"));
  return CallbackReturn::SUCCESS;
}

PostureManagerNode::CallbackReturn PostureManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
  // Ready, not moving: the arm moves only on ~/go_high or ~/go_low.
  publishState("none");
  timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {onTimer();});
  return CallbackReturn::SUCCESS;
}

PostureManagerNode::CallbackReturn PostureManagerNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  timer_.reset();
  return CallbackReturn::SUCCESS;
}

PostureManagerNode::CallbackReturn PostureManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  go_high_srv_.reset();
  go_low_srv_.reset();
  depth_sub_.reset();
  info_sub_.reset();
  servo_pub_.reset();
  tf_broadcaster_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  return CallbackReturn::SUCCESS;
}

void PostureManagerNode::publishState(const std::string & state)
{
  std_msgs::msg::String s;
  s.data = state;
  state_pub_->publish(s);
}

bool PostureManagerNode::request(const std::string & name, std::string & message)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    message = "posture_manager is not active";
    return false;
  }
  Posture p;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ == Phase::kMoving || phase_ == Phase::kCalibrating) {
      message = "busy: already moving to " + target_;
      return false;
    }
    if (phase_ == Phase::kReady && current_ == name) {
      message = "already in " + name;
      return true;
    }
    p = postures_.at(name);
    target_ = name;
    phase_ = Phase::kMoving;
    phase_until_ = steady_.now() + rclcpp::Duration::from_seconds(move_duration_ + settle_);
  }
  // While the arm moves, the robot is as tall as the taller of the two.
  std_msgs::msg::Float32 h;
  h.data = static_cast<float>(std::max(p.height, postures_.at("high").height));
  height_pub_->publish(h);
  publishState("moving");

  servo_controller_msgs::msg::ServosPosition cmd;
  cmd.duration = move_duration_;
  cmd.position_unit = "pulse";  // without it the message is silently ignored
  for (std::size_t i = 0; i < 5; ++i) {
    servo_controller_msgs::msg::ServoPosition s;
    s.id = static_cast<uint16_t>(i + 1);
    s.position = static_cast<double>(p.pulses[i]);
    cmd.position.push_back(s);
  }
  servo_pub_->publish(cmd);
  RCLCPP_INFO(get_logger(), "moving the arm to '%s'", name.c_str());
  message = "moving to " + name;
  return true;
}

void PostureManagerNode::onTimer()
{
  bool timed_out = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ == Phase::kMoving && steady_.now() >= phase_until_) {
      // Arm settled: onDepth calibrates on the next image.
      phase_ = Phase::kCalibrating;
      phase_until_ = steady_.now() + rclcpp::Duration::from_seconds(calibration_timeout_);
      publishState("calibrating");
    } else if (phase_ == Phase::kCalibrating && steady_.now() >= phase_until_) {
      timed_out = true;
    }
  }
  if (timed_out) {
    RCLCPP_WARN(get_logger(), "no depth image to calibrate: using the stored calibration");
    finishCalibration(std::nullopt);
  }

  // Keep broadcasting the camera pose (dynamic TF: it changes with posture).
  std::optional<Eigen::Isometry3d> mount;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mount = mount_;
  }
  if (mount) {
    auto t = tf2::eigenToTransform(*mount);
    t.header.stamp = now();
    t.header.frame_id = "base_footprint";
    t.child_frame_id = "a7_depth_optical";
    tf_broadcaster_->sendTransform(t);
  }
}

void PostureManagerNode::onDepth(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  std::array<double, 4> k{};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ != Phase::kCalibrating || !intrinsics_) {
      return;
    }
    k = *intrinsics_;
  }
  if ((msg->encoding != "mono16" && msg->encoding != "16UC1") || msg->step != msg->width * 2) {
    return;
  }
  std::vector<uint16_t> img(static_cast<std::size_t>(msg->width) * msg->height);
  std::memcpy(img.data(), msg->data.data(), img.size() * 2);

  depth::ProjectionConfig pc;
  pc.min_row_fraction = 0.55;  // lower part of the image: the floor
  const auto pts = depth::projectDepth(
    img, msg->width, msg->height, depth::Intrinsics{k[0], k[1], k[2], k[3]}, pc);
  const auto fit = depth::fitFloor(pts, depth::FloorFitConfig{});
  if (!fit) {
    finishCalibration(std::nullopt);
    return;
  }

  // x, y, yaw of the camera come from the URDF chain (the floor cannot
  // observe them); height, pitch and roll from the floor plane.
  double x = 0.084;
  double y = 0.0;
  double yaw = 0.0;
  try {
    const auto t = tf_buffer_->lookupTransform(
      "base_footprint", "depth_cam_link",
      tf2::TimePointZero);
    x = t.transform.translation.x;
    y = t.transform.translation.y;
    const auto & q = t.transform.rotation;
    yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  } catch (const tf2::TransformException &) {
    RCLCPP_WARN(get_logger(), "no depth_cam_link in TF: using x=0.084, y=0, yaw=0");
  }
  const auto m = depth::mountFromFloor(*fit, x, y, yaw);

  Posture p;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    p = postures_.at(target_);
  }
  // A posture never calibrated has only a rough stored estimate: accept any
  // physically sensible floor and log the values to store them.
  CalibrationLimits lim = limits_;
  if (!p.calibrated) {
    lim.max_height_error = 0.15;
    lim.max_angle_error = 0.7;
  }
  if (!calibrationPlausible(m.height, m.pitch, m.roll, p, lim)) {
    RCLCPP_WARN(
      get_logger(), "floor fit not plausible for '%s' (h %.3f, pitch %.1f, roll %.1f deg): "
      "using the stored calibration", p.name.c_str(), m.height, m.pitch * 180 / M_PI,
      m.roll * 180 / M_PI);
    finishCalibration(std::nullopt);
    return;
  }
  RCLCPP_INFO(
    get_logger(), "'%s' calibrated on the floor: camera at %.3f m, pitch %.1f, roll %.1f deg "
    "(%zu floor points, rms %.1f mm)%s", p.name.c_str(), m.height, m.pitch * 180 / M_PI,
    m.roll * 180 / M_PI, fit->inliers, fit->rms * 1000,
    p.calibrated ? "" : "  <- store as cam_height / cam_pitch / cam_roll (rad)");
  finishCalibration(m.base_from_optical);
}

Eigen::Isometry3d PostureManagerNode::storedMount(const Posture & p)
{
  double x = 0.084;
  double y = 0.0;
  double yaw = 0.0;
  try {
    const auto t = tf_buffer_->lookupTransform(
      "base_footprint", "depth_cam_link",
      tf2::TimePointZero);
    x = t.transform.translation.x;
    y = t.transform.translation.y;
    const auto & q = t.transform.rotation;
    yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  } catch (const tf2::TransformException &) {
  }
  // Same convention as mountFromFloor: nose down = positive rotation about y.
  return Eigen::Translation3d(x, y, p.cam_height) *
         Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(p.cam_pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(p.cam_roll, Eigen::Vector3d::UnitX()) *
         Eigen::Quaterniond(depth::bodyFromOptical());
}

void PostureManagerNode::finishCalibration(std::optional<Eigen::Isometry3d> live)
{
  Posture p;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ != Phase::kCalibrating) {
      return;  // already finished (a late image or the timeout)
    }
    p = postures_.at(target_);
  }
  const Eigen::Isometry3d mount = live ? *live : storedMount(p);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mount_ = mount;
    current_ = target_;
    phase_ = Phase::kReady;
  }
  std_msgs::msg::Float32 h;
  h.data = static_cast<float>(p.height);
  height_pub_->publish(h);
  geometry_msgs::msg::Polygon fp;
  for (const auto & c : footprintCorners(p)) {
    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(c[0]);
    pt.y = static_cast<float>(c[1]);
    fp.points.push_back(pt);
  }
  local_fp_pub_->publish(fp);
  global_fp_pub_->publish(fp);
  publishState(p.name);
  RCLCPP_INFO(
    get_logger(), "posture '%s' ready (%s calibration, height %.2f m)", p.name.c_str(),
    live ? "live" : "stored", p.height);
}

}  // namespace landerpi_posture
