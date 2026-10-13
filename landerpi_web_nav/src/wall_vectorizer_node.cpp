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

#include "landerpi_web_nav/wall_vectorizer_node.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "tf2/exceptions.h"

namespace landerpi_web_nav
{

namespace
{

WallTracing parseMethod(const std::string & name)
{
  if (name == "free_space") {
    return WallTracing::kFreeSpace;
  }
  if (name == "skeleton") {
    return WallTracing::kSkeleton;
  }
  if (name == "contours") {
    return WallTracing::kContours;
  }
  throw std::invalid_argument(
          "method must be free_space, skeleton or contours, got '" + name + "'");
}

}  // namespace

WallVectorizerNode::WallVectorizerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("wall_vectorizer", options)
{
  contours_config_.method = parseMethod(declare_parameter<std::string>("method", "free_space"));
  contours_config_.occupied_threshold = declare_parameter<int>("occupied_threshold", 65);
  contours_config_.free_threshold = declare_parameter<int>("free_threshold", 25);
  contours_config_.wall_adjacency_cells = declare_parameter<int>("wall_adjacency_cells", 2);
  contours_config_.close_radius_cells = declare_parameter<int>("close_radius_cells", 1);

  extractor_config_.max_gap = declare_parameter<double>("max_gap", 0.15);
  extractor_config_.split_distance = declare_parameter<double>("split_distance", 0.08);
  extractor_config_.merge_angle = declare_parameter<double>("merge_angle", 0.14);
  extractor_config_.merge_distance = declare_parameter<double>("merge_distance", 0.12);
  extractor_config_.merge_gap = declare_parameter<double>("merge_gap", 0.25);
  extractor_config_.min_points =
    static_cast<std::size_t>(declare_parameter<int>("min_points", 5));
  extractor_config_.min_length = declare_parameter<double>("min_length", 0.30);
  extractor_config_.corner_min_angle = declare_parameter<double>("corner_min_angle", 0.52);
  extractor_config_.corner_max_extension =
    declare_parameter<double>("corner_max_extension", 0.35);
  extractor_ = std::make_unique<LineExtractor>(extractor_config_);  // validates

  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  line_width_ = declare_parameter<double>("line_width", 0.04);
  const double rate_hz = declare_parameter<double>("rate_hz", 1.0);
  if (!(rate_hz > 0.0 && rate_hz <= 10.0)) {
    throw std::invalid_argument("rate_hz must be in (0, 10]");
  }
  if (contours_config_.occupied_threshold < 1 || contours_config_.occupied_threshold > 100 ||
    contours_config_.free_threshold < 1 ||
    contours_config_.free_threshold > contours_config_.occupied_threshold ||
    contours_config_.wall_adjacency_cells < 0 || contours_config_.close_radius_cells < 0)
  {
    throw std::invalid_argument("invalid occupancy thresholds or cell radii");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // slam_toolbox publishes /map reliable + transient local: match it so a
  // late start still gets the current map.
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {onMap(std::move(msg));});
  walls_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/a7/walls", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_hz), [this]() {onTimer();});

  init_finish_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/init_finish",
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = true;
    });

  RCLCPP_INFO(get_logger(), "wall_vectorizer ready: /map -> /a7/walls at %.1f Hz", rate_hz);
}

void WallVectorizerNode::onMap(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_map_ = std::move(msg);
  map_changed_ = true;
}

std::optional<Point2> WallVectorizerNode::robotPosition(const std::string & map_frame)
{
  try {
    const auto tf = tf_buffer_->lookupTransform(map_frame, base_frame_, tf2::TimePointZero);
    return Point2{tf.transform.translation.x, tf.transform.translation.y};
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "no %s -> %s yet (%s); using the largest free region", map_frame.c_str(),
      base_frame_.c_str(), e.what());
    return std::nullopt;
  }
}

void WallVectorizerNode::onTimer()
{
  if (!map_changed_ || !latest_map_) {
    return;
  }
  map_changed_ = false;
  const auto & map = *latest_map_;
  // The map origin is assumed unrotated (slam_toolbox always publishes yaw 0).
  const GridView grid{
    static_cast<int>(map.info.width), static_cast<int>(map.info.height),
    map.info.resolution, map.info.origin.position.x, map.info.origin.position.y, &map.data};

  MapContoursConfig cfg = contours_config_;
  cfg.seed = robotPosition(map.header.frame_id);

  const auto t0 = std::chrono::steady_clock::now();
  std::vector<Segment> segments;
  try {
    segments = extractor_->extract(occupiedContours(grid, cfg));
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "bad map: %s", e.what());
    return;
  }
  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();

  visualization_msgs::msg::MarkerArray out;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  out.markers.push_back(clear);

  visualization_msgs::msg::Marker lines;
  lines.header.frame_id = map.header.frame_id;
  lines.header.stamp = now();
  lines.ns = "walls";
  lines.id = 0;
  lines.type = visualization_msgs::msg::Marker::LINE_LIST;
  lines.action = visualization_msgs::msg::Marker::ADD;
  lines.pose.orientation.w = 1.0;
  lines.scale.x = line_width_;
  lines.color.r = 1.0f;
  lines.color.g = 0.15f;
  lines.color.b = 0.1f;
  lines.color.a = 1.0f;
  for (const auto & s : segments) {
    geometry_msgs::msg::Point a;
    geometry_msgs::msg::Point b;
    a.x = s.start.x;
    a.y = s.start.y;
    a.z = 0.02;
    b.x = s.end.x;
    b.y = s.end.y;
    b.z = 0.02;
    lines.points.push_back(a);
    lines.points.push_back(b);
  }
  out.markers.push_back(lines);
  walls_pub_->publish(out);

  RCLCPP_DEBUG(
    get_logger(), "%zu walls from %ux%u map in %.1f ms", segments.size(), map.info.width,
    map.info.height, ms);
}

}  // namespace landerpi_web_nav
