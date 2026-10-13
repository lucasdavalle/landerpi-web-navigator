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
#include <optional>
#include <string>

#include "landerpi_web_nav/line_extractor.hpp"
#include "landerpi_web_nav/map_contours.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace landerpi_web_nav
{

// Turns the live SLAM map into wall segments (LineExtractor + MapContours)
// and publishes them as RViz markers. Plain rclcpp::Node: perception only,
// commands no actuator. The web gateway (stage 3) will reuse the same
// classes directly.
//
// Subscribes /map (transient local, as slam_toolbox publishes it), looks up
// map -> base_footprint for the free-space seed (the robot's position), and
// publishes /a7/walls (visualization_msgs/MarkerArray) at most rate_hz, only
// when the map changed. See config/wall_vectorizer.yaml for parameters.
class WallVectorizerNode : public rclcpp::Node
{
public:
  explicit WallVectorizerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onMap(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onTimer();
  std::optional<Point2> robotPosition(const std::string & map_frame);

  LineExtractorConfig extractor_config_;
  MapContoursConfig contours_config_;
  std::unique_ptr<LineExtractor> extractor_;
  std::string base_frame_;
  double line_width_ = 0.04;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  bool map_changed_ = false;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_finish_srv_;
};

}  // namespace landerpi_web_nav
