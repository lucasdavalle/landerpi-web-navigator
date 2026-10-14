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
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
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
  void stopTeleop();
  std::string driveStateJson();

  std::mutex mutex_;
  Snapshot snap_;
  DriveArbiter arbiter_;        // guarded by mutex_
  std::string gate_reason_ = "unknown";  // guarded by mutex_

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
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_running_client_;
  rclcpp::TimerBase::SharedPtr pose_timer_, scan_timer_, status_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_finish_srv_;
};

}  // namespace landerpi_web_nav
