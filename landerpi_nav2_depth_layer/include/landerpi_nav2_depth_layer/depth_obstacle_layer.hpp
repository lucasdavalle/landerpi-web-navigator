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

#include "Eigen/Core"

#include "landerpi_nav2_depth_layer/crouch_grid.hpp"
#include "landerpi_nav2_depth_layer/depth_projector.hpp"
#include "landerpi_nav2_depth_layer/height_classifier.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

namespace landerpi_nav2_depth_layer
{

// Nav2 costmap layer from the depth camera (stages 6b, 6c). Each depth
// image (at most rate_hz) is projected to 3D, placed in base_footprint with
// the camera pose that posture_manager publishes as `sensor_frame`
// (calibrated against the floor) and classified against both postures:
// obstacle, crouchable (passable only crouched), floor. The layer's own
// grid remembers them (crouch_grid.hpp has the rules) and keeps them out of
// view: that memory covers the camera's blind zone and, under a bed, the
// bed itself.
//
// What reaches the master costmap depends on `crouchable_cost`:
// by_posture (the local costmap) blocks crouchable cells while the robot
// stands (/a7/robot_height) and frees them crouched; free (the global
// costmap) never blocks them, so the planner goes under and the robot
// crouches on the way. Combined with max: the layer only adds obstacles,
// never clears the lidar's.
//
// With `input: image` (the local costmap) it processes the camera and
// publishes:
// - the classified frame (`frame_topic`), which the global costmap's
//   instance reads with `input: cloud` instead of reprocessing the image;
// - the obstacles the robot would hit in its current posture
//   (`obstacle_cloud_topic`), for the Collision Monitor.
// With `publish_crouch_cells` (the global costmap) it publishes the
// crouchable cells in the map, for crouch_coordinator.
class DepthObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  DepthObstacleLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j) override;
  // Keeps what the camera saw: see the .cpp.
  void reset() override;
  bool isClearable() override {return false;}

private:
  void onImage(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  // Rate limit shared by both inputs: true if this frame should be used.
  bool takeFrame(const rclcpp::Time & stamp);
  // Places a classified frame (base_footprint, at `stamp`) in the global
  // frame and leaves it pending for updateBounds.
  void store(const Classified & c, const rclcpp::Time & stamp);
  bool standing();
  void publishCrouchCells();

  // Latest processed observation, in the costmap's global frame.
  struct Observation
  {
    std::vector<Eigen::Vector2d> obstacles;
    std::vector<Eigen::Vector2d> crouchable;
    std::vector<Eigen::Vector2d> floor;
  };

  std::mutex mutex_;
  std::optional<Observation> pending_;
  std::optional<Intrinsics> intrinsics_;
  double robot_height_ = 0.31;
  // Frames are used only with the arm settled in a calibrated posture:
  // while it moves, the camera turns but its calibrated pose does not, and
  // walls land in the wrong cells (seen on the robot, stage 6c step B).
  bool posture_ready_ = false;
  rclcpp::Time last_processed_{0, 0, RCL_ROS_TIME};
  // The posture the master costmap was last built with: when it changes,
  // every crouchable cell must be re-applied, not only the new ones.
  std::optional<bool> applied_standing_;
  rclcpp::Time last_crouch_publish_{0, 0, RCL_ROS_TIME};

  // Parameters.
  std::string sensor_frame_;
  std::string base_frame_;
  // Fixed frame for placing a frame in the world: the robot's motion is
  // taken from odom at the image time, the map correction (slam_toolbox,
  // which lags) at its latest. Asking map at the image time extrapolated.
  std::string odom_frame_;
  bool cloud_input_ = false;
  double floor_cell_ = 0.05;
  double rate_hz_ = 5.0;
  double transform_tolerance_ = 0.3;
  ProjectionConfig projection_;
  RobotEnvelope envelope_;
  CrouchableCost crouchable_cost_ = CrouchableCost::kByPosture;
  bool rolling_window_ = false;
  bool publish_obstacle_cloud_ = true;
  bool publish_crouch_cells_ = false;
  double crouch_cells_rate_ = 1.0;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr height_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr posture_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frame_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr crouch_pub_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("depth_obstacle_layer")};
};

}  // namespace landerpi_nav2_depth_layer
