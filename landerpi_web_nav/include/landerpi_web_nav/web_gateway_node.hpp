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

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <nlohmann/json.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "landerpi_web_nav/auth_manager.hpp"
#include "landerpi_web_nav/drive_arbiter.hpp"
#include "landerpi_web_nav/ip_filter.hpp"
#include "landerpi_web_nav/message_codec.hpp"
#include "landerpi_web_nav/web_server.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace landerpi_web_nav
{

// Web gateway: exposes the robot's live state to the browser over REST +
// WebSocket (see web_server.hpp), behind the home-only IP filter and
// password/token auth (stage 3), and lets one browser at a time drive it
// through velocity_gate (stage 4).
//
// ROS side: subscribes /map, /a7/walls (from wall_vectorizer), /scan_raw,
// /pose (slam_toolbox, covariance), /odom and the battery; looks up
// map -> base_footprint and map -> lidar frame in TF. Callbacks store the
// latest state in a mutex-protected snapshot; timers push pose, scan and
// status to the WebSocket clients.
//
// Driving (stage 4), WebSocket messages from the browser:
//   {"type":"drive","enable":true|false}  ask for / hand back control
//   {"type":"heartbeat"}                  driver only, ~5 Hz -> /a7/heartbeat
//   {"type":"teleop","vx":..,"vy":..,"wz":..}  driver only -> /a7/cmd_vel_teleop
//
// Navigation (stage 5), driver only:
//   {"type":"goal","x":..,"y":..}  -> Nav2 NavigateToPose (map frame)
//   {"type":"cancel"}              -> cancel the goal
// A non-zero teleop, handing back control or disconnecting also cancels it.
// The browser gets {"type":"nav",...} (state, goal, distance, eta) and
// {"type":"path","points":[[x,y],...]} (the plan, downsampled).
// The gateway never publishes to the wheels itself: velocity_gate does,
// and stops on its own if the heartbeats stop (tab closed, Wi-Fi lost).
class WebGatewayNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit WebGatewayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  // Latest state, shared between ROS callbacks and the network thread.
  struct Snapshot
  {
    std::optional<std::vector<uint8_t>> map_frame;  // encoded once per map
    std::optional<std::vector<uint8_t>> depth_frame;  // the camera's grid (6c)
    std::optional<MapMessage> map_info;              // without cells
    std::string walls_json = R"({"type":"walls","segments":[]})";
    std::optional<std::string> pose_json;
    std::optional<std::array<double, 3>> position_covariance;
    std::optional<double> yaw_variance;
    std::optional<int> battery_mv;
    std::optional<rclcpp::Time> last_scan, last_odom, last_map, last_pose;
    sensor_msgs::msg::LaserScan::SharedPtr scan;
  };

  HttpResponse handleApi(const HttpRequest & req);
  std::vector<WsFrame> initialFrames();
  StatusMessage status();

  void onMap(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onWalls(visualization_msgs::msg::MarkerArray::SharedPtr msg);
  void onScan(sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void publishPose();
  void publishScan();
  void publishStatus();
  void releaseAll();

  void onWsMessage(WsClientId client, const std::string & text);
  void handleWsMessage(WsClientId client, const nlohmann::json & msg);
  void onWsClosed(WsClientId client);
  void applyDriveAction(DriveArbiter::Action action);
  // Enabling first asks posture_manager to stand up if there is room
  // (stage 6c, try_stand): the gate is enabled once it settles, high or low.
  void enableAfterPosture();

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  void sendGoal(double x, double y);
  void cancelGoal(const char * why);
  void setNav(const std::string & state);  // updates nav_ and broadcasts
  std::string navJson();
  std::string navJsonLocked() const;  // caller holds mutex_
  void broadcastNav();
  void onPlan(nav_msgs::msg::Path::SharedPtr msg);
  void stopTeleop();
  std::string driveStateJson();
  std::string driveJsonLocked() const;  // caller holds mutex_
  void onDepthGrid(nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  std::mutex mutex_;
  Snapshot snap_;
  DriveArbiter arbiter_;        // guarded by mutex_
  std::string gate_reason_ = "unknown";  // guarded by mutex_
  std::string posture_ = "unknown";      // guarded by mutex_ (posture_manager state)
  // Stage 6c, guarded by mutex_: where the arm is going, the last look-up
  // verdict and what crouch_coordinator is doing (for the web's overlays).
  std::string arm_target_;
  std::string stand_check_;
  std::string crouch_status_;
  bool pending_enable_ = false;          // guarded by mutex_

  // Navigation state shown in the browser, guarded by mutex_.
  struct NavState
  {
    std::string state = "idle";  // idle, pending, active, succeeded, aborted, canceled, rejected
    std::optional<std::array<double, 2>> goal;
    std::optional<double> distance;
    std::optional<double> eta;
    int recoveries = 0;
  };
  NavState nav_;
  NavGoalHandle::SharedPtr goal_handle_;  // guarded by mutex_
  uint64_t goal_seq_ = 0;                  // ignores callbacks of replaced goals
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_feedback_broadcast_{0, 0, RCL_STEADY_TIME};

  std::unique_ptr<AuthManager> auth_;
  std::unique_ptr<IpFilter> ip_filter_;
  std::unique_ptr<WebServer> server_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string map_frame_;
  std::string base_frame_;
  int scan_stride_ = 2;
  double pose_rate_hz_ = 10.0;
  double scan_rate_hz_ = 5.0;
  double status_rate_hz_ = 1.0;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr walls_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gate_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr posture_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stand_check_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr crouch_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr depth_grid_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stand_client_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_running_client_;
  rclcpp::TimerBase::SharedPtr pose_timer_, scan_timer_, status_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_finish_srv_;
};

}  // namespace landerpi_web_nav
