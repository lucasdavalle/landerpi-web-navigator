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
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "Eigen/Geometry"

#include "geometry_msgs/msg/polygon.hpp"
#include "landerpi_nav2_depth_layer/clearance_check.hpp"
#include "landerpi_posture/posture.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "servo_controller_msgs/msg/servos_position.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace landerpi_posture
{

// Arm postures for navigation (stage 6). ~/go_high and ~/go_low move the
// arm (3 s, all joints at once so every servo interpolates from where it
// really is), wait for it to settle, calibrate the depth camera against the
// floor (falling back to the stored calibration if the live one is not
// plausible), then publish:
//   TF base_footprint -> a7_depth_optical  (the calibrated camera, 10 Hz)
//   /a7/robot_height   (Float32, transient local)
//   /a7/posture        (String: none, moving, calibrating, high, low,
//                       looking_up, checking)
//   /local_costmap/footprint, /global_costmap/footprint (Polygon)
// ~/try_stand (stage 6c) stands up only if there is room: from crouched it
// turns the camera up (the look-up pose, lower than the low posture), and
// on the next depth image checkClearance decides: clear -> high, blocked
// or unknown -> back to low. The verdict goes on /a7/stand_check (String,
// transient local).
// Activating the node moves nothing: the arm moves only when asked.
class PostureManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit PostureManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

private:
  enum class Phase {kNone, kMoving, kCalibrating, kReady, kLooking, kChecking};

  Posture declarePosture(const std::string & name);
  bool request(const std::string & name, std::string & message);
  bool tryStand(std::string & message);
  // Moves to a posture with no busy check (the caller owns the phase).
  void startMove(const std::string & name);
  void sendPulses(const std::array<int, 5> & pulses);
  void checkClearance(const sensor_msgs::msg::Image & msg);
  void finishCheck(landerpi_nav2_depth_layer::Clearance verdict, const std::string & detail);
  void onTimer();
  void onDepth(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void finishCalibration(std::optional<Eigen::Isometry3d> live);
  Eigen::Isometry3d storedMount(const Posture & p);
  void publishState(const std::string & state);

  std::mutex mutex_;
  std::map<std::string, Posture> postures_;
  Phase phase_ = Phase::kNone;
  std::string target_;
  std::string current_;
  rclcpp::Time phase_until_{0, 0, RCL_STEADY_TIME};
  rclcpp::Clock steady_{RCL_STEADY_TIME};
  std::optional<Eigen::Isometry3d> mount_;  // base_footprint <- a7_depth_optical
  std::optional<std::array<double, 4>> intrinsics_;
  double move_duration_ = 3.0;
  double settle_ = 0.7;
  double calibration_timeout_ = 2.0;
  CalibrationLimits limits_;
  std::array<int, 5> look_up_pulses_{500, 100, 260, 980, 500};
  landerpi_nav2_depth_layer::ClearanceConfig clearance_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<servo_controller_msgs::msg::ServosPosition>::SharedPtr servo_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr height_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr check_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr local_fp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr global_fp_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_high_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_low_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr try_stand_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_finish_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace landerpi_posture
