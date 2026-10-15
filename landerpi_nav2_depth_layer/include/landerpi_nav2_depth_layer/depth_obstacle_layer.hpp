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

#include "landerpi_nav2_depth_layer/depth_projector.hpp"
#include "landerpi_nav2_depth_layer/height_classifier.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"

namespace landerpi_nav2_depth_layer
{

// Nav2 costmap layer from the depth camera (stage 6b). Each depth image
// (at most rate_hz) is projected to 3D, placed in base_footprint with the
// camera pose that posture_manager publishes as `sensor_frame` (calibrated
// against the floor), classified by height against the robot's current
// height (/a7/robot_height), and written into the layer's own grid:
// obstacles LETHAL, seen floor FREE. Cells out of view keep their value
// (memory: covers the camera's ~0.4 m blind zone in front of the robot).
// Combined with updateWithMax, so it can only add obstacles to the master
// costmap, never clear the lidar's.
//
// It also publishes the obstacle points (base_footprint) as a small
// PointCloud2 for the Collision Monitor.
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

  // Latest processed observation, in the costmap's global frame.
  struct Observation
  {
    std::vector<Eigen::Vector2d> obstacles;
    std::vector<Eigen::Vector2d> floor;
  };

  std::mutex mutex_;
  std::optional<Observation> pending_;
  std::optional<Intrinsics> intrinsics_;
  double robot_height_ = 0.31;
  rclcpp::Time last_processed_{0, 0, RCL_ROS_TIME};

  // Parameters.
  std::string sensor_frame_;
  std::string base_frame_;
  double rate_hz_ = 5.0;
  double transform_tolerance_ = 0.3;
  ProjectionConfig projection_;
  RobotEnvelope envelope_;
  bool rolling_window_ = false;
  // Only one layer instance (the local costmap's) should feed the Collision
  // Monitor; the global one processes fewer frames and publishes nothing.
  bool publish_cloud_ = true;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr height_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Logger logger_{rclcpp::get_logger("depth_obstacle_layer")};
};

}  // namespace landerpi_nav2_depth_layer
