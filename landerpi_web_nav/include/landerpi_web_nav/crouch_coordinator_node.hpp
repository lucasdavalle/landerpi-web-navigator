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
#include <optional>
#include <string>
#include <vector>

#include "landerpi_web_nav/crouch_planner.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace landerpi_web_nav
{

// Crouches and stands the robot up during a navigation (stage 6c), with
// CrouchPlanner deciding at 2 Hz from:
//   /plan (Nav2's global plan, map), fresh only while navigating
//   /a7/crouch_cells (PointCloud2, map): from the global depth layer
//   /a7/posture, /a7/stand_check: from posture_manager
//   /local_costmap/costmap: obstacles near the robot (no arm moves there)
//   TF map -> base_footprint
// It acts through posture_manager (~/go_low, ~/try_stand); velocity_gate
// keeps the robot still while the arm moves. Publishes /a7/crouch_status
// (String, transient local) for the web. Without a navigation in progress
// it does nothing: activating it never moves the arm.
class CrouchCoordinatorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit CrouchCoordinatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

private:
  void onTimer();
  void onStandCheck(const std::string & verdict);
  void call(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client, const std::string & what);
  std::optional<Point2> robotInMap();
  // A lethal cell of the local costmap within arm_clearance of the robot.
  bool obstacleNear();
  void publishStatus(const std::string & status);

  std::mutex mutex_;
  std::unique_ptr<CrouchPlanner> planner_;
  std::vector<Point2> path_;
  rclcpp::Time path_received_{0, 0, RCL_STEADY_TIME};
  std::vector<Point2> crouch_cells_;
  nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap_;
  double arm_clearance_ = 0.45;
  std::string posture_ = "none";
  bool stand_pending_ = false;  // we asked try_stand and await its verdict
  std::string last_status_;
  rclcpp::Clock steady_{RCL_STEADY_TIME};
  double plan_timeout_ = 3.0;
  std::string map_frame_;
  std::string base_frame_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cells_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr posture_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr check_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr go_low_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr try_stand_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_finish_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace landerpi_web_nav
