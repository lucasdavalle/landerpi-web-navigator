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

#include "landerpi_nav2_depth_layer/depth_obstacle_layer.hpp"

#include <cmath>

#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Geometry"

#include "landerpi_nav2_depth_layer/observation_cloud.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"

namespace landerpi_nav2_depth_layer
{

void DepthObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("depth_obstacle_layer: node expired");
  }
  logger_ = node->get_logger();

  auto param = [&](const std::string & n, const rclcpp::ParameterValue & v) {
      declareParameter(n, v);
      return node->get_parameter(name_ + "." + n);
    };
  enabled_ = param("enabled", rclcpp::ParameterValue(true)).as_bool();
  const auto input = param("input", rclcpp::ParameterValue(std::string("image"))).as_string();
  if (input != "image" && input != "cloud") {
    throw std::invalid_argument("depth_obstacle_layer: input must be image or cloud");
  }
  cloud_input_ = input == "cloud";
  const std::string camera_ns = "/ascamera/camera_publisher/depth0";
  const auto depth_topic =
    param("depth_topic", rclcpp::ParameterValue(camera_ns + "/image_raw")).as_string();
  const auto info_topic =
    param("info_topic", rclcpp::ParameterValue(camera_ns + "/camera_info")).as_string();
  // The image's own frame (depth_camera_link) is not connected to the TF
  // tree; posture_manager publishes the floor-calibrated pose as this frame.
  sensor_frame_ = param("sensor_frame", rclcpp::ParameterValue(std::string("a7_depth_optical")))
    .as_string();
  base_frame_ = param("base_frame", rclcpp::ParameterValue(std::string("base_footprint")))
    .as_string();
  odom_frame_ = param("odom_frame", rclcpp::ParameterValue(std::string("odom"))).as_string();
  floor_cell_ = param("cloud_floor_cell", rclcpp::ParameterValue(0.05)).as_double();
  rate_hz_ = param("rate_hz", rclcpp::ParameterValue(5.0)).as_double();
  transform_tolerance_ = param("transform_tolerance", rclcpp::ParameterValue(0.3)).as_double();
  projection_.stride = param("stride", rclcpp::ParameterValue(4)).as_int();
  projection_.min_range = param("min_range", rclcpp::ParameterValue(0.25)).as_double();
  projection_.max_range = param("max_range", rclcpp::ParameterValue(3.0)).as_double();
  envelope_.floor_band = param("floor_band", rclcpp::ParameterValue(0.015)).as_double();
  envelope_.height_margin = param("height_margin", rclcpp::ParameterValue(0.05)).as_double();
  envelope_.max_range = param("obstacle_max_range", rclcpp::ParameterValue(2.5)).as_double();
  robot_height_ = param("default_robot_height", rclcpp::ParameterValue(0.31)).as_double();
  envelope_.body_x_min = param("body_x_min", rclcpp::ParameterValue(-0.13)).as_double();
  envelope_.body_x_max = param("body_x_max", rclcpp::ParameterValue(0.13)).as_double();
  envelope_.body_half_width = param("body_half_width", rclcpp::ParameterValue(0.11)).as_double();
  const auto height_topic = param(
    "height_topic", rclcpp::ParameterValue(std::string("/a7/robot_height"))).as_string();
  publish_cloud_ = param("publish_obstacle_cloud", rclcpp::ParameterValue(true)).as_bool();
  const auto cloud_topic = param(
    "obstacle_cloud_topic", rclcpp::ParameterValue(std::string("/a7/depth_obstacles")))
    .as_string();
  if (!(rate_hz_ > 0.0 && rate_hz_ <= 30.0) || projection_.stride < 1 || !(floor_cell_ > 0.0)) {
    throw std::invalid_argument("depth_obstacle_layer: bad rate_hz, stride or cloud_floor_cell");
  }

  rolling_window_ = layered_costmap_->isRolling();
  // Unknown is not this layer's business: start (and reset) to free, so
  // updateWithMax only ever adds what the camera actually saw.
  default_value_ = nav2_costmap_2d::FREE_SPACE;
  matchSize();
  current_ = true;

  if (cloud_input_) {
    // Another instance of this layer already classified the frame.
    cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::QoS(1).reliable(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {onCloud(std::move(m));});
    publish_cloud_ = false;
    RCLCPP_INFO(
      logger_, "depth_obstacle_layer: reading classified frames from %s, %.1f Hz",
      cloud_topic.c_str(), rate_hz_);
    return;
  }
  image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
    depth_topic, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Image::ConstSharedPtr m) {onImage(std::move(m));});
  info_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    info_topic, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);
      intrinsics_ = Intrinsics{m->k[0], m->k[4], m->k[2], m->k[5]};
    });
  // posture_manager publishes the current height (transient local: a late
  // start still gets it).
  height_sub_ = node->create_subscription<std_msgs::msg::Float32>(
    height_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::Float32::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);
      robot_height_ = m->data;
    });
  if (publish_cloud_) {
    obstacle_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::QoS(1).reliable());
  }

  RCLCPP_INFO(
    logger_, "depth_obstacle_layer: %s in frame %s, %.1f Hz, stride %d", depth_topic.c_str(),
    sensor_frame_.c_str(), rate_hz_, projection_.stride);
}

void DepthObstacleLayer::onImage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (!enabled_) {
    return;
  }
  auto node = node_.lock();
  if (!node) {
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
  if (!takeFrame(stamp)) {
    return;
  }

  Intrinsics k;
  RobotEnvelope envelope = envelope_;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!intrinsics_) {
      return;  // no camera_info yet
    }
    k = *intrinsics_;
    envelope.height = robot_height_;
  }
  if (msg->encoding != "mono16" && msg->encoding != "16UC1") {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 10000, "unexpected depth encoding %s", msg->encoding.c_str());
    return;
  }

  std::vector<uint16_t> depth(static_cast<std::size_t>(msg->width) * msg->height);
  if (msg->step != msg->width * 2 || msg->data.size() < depth.size() * 2) {
    return;  // padded or truncated rows: not what the Aurora930 sends
  }
  std::memcpy(depth.data(), msg->data.data(), depth.size() * 2);

  Eigen::Isometry3d base_from_sensor;
  try {
    // The camera pose on the robot changes only with the posture: latest.
    base_from_sensor = tf2::transformToEigen(
      tf_->lookupTransform(base_frame_, sensor_frame_, tf2::TimePointZero));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 10000, "no camera pose: %s", e.what());
    return;
  }

  auto points = projectDepth(depth, msg->width, msg->height, k, projection_);
  for (auto & p : points) {
    p = base_from_sensor * p;
  }
  const Classified c = classify(points, envelope);

  // The classified frame for the Collision Monitor and the global costmap.
  if (publish_cloud_) {
    const auto cloud_points = toCloudPoints(c, floor_cell_);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = msg->header.stamp;
    cloud.header.frame_id = base_frame_;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(cloud_points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & p : cloud_points) {
      *x = static_cast<float>(p.x());
      *y = static_cast<float>(p.y());
      *z = static_cast<float>(p.z());
      ++x;
      ++y;
      ++z;
    }
    obstacle_pub_->publish(cloud);
  }

  store(c, stamp);
}

void DepthObstacleLayer::onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!enabled_) {
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
  if (msg->header.frame_id != base_frame_) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node->get_clock(), 10000, "cloud in frame %s, expected %s",
        msg->header.frame_id.c_str(), base_frame_.c_str());
    }
    return;
  }
  if (!takeFrame(stamp)) {
    return;
  }
  std::vector<Eigen::Vector3d> points;
  points.reserve(static_cast<std::size_t>(msg->width) * msg->height);
  sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    points.emplace_back(*x, *y, *z);
  }
  store(fromCloudPoints(points), stamp);
}

bool DepthObstacleLayer::takeFrame(const rclcpp::Time & stamp)
{
  // Rate limit: the camera sends ~15 Hz, the Pi also runs SLAM and Nav2.
  std::lock_guard<std::mutex> lock(mutex_);
  if ((stamp - last_processed_).seconds() < 1.0 / rate_hz_) {
    return false;
  }
  last_processed_ = stamp;
  return true;
}

void DepthObstacleLayer::store(const Classified & c, const rclcpp::Time & stamp)
{
  Eigen::Isometry3d global_from_base;
  try {
    // The robot at the frame's time through odom, the map correction at
    // its latest (see odom_frame_).
    global_from_base = tf2::transformToEigen(
      tf_->lookupTransform(
        layered_costmap_->getGlobalFrameID(), tf2::TimePointZero, base_frame_,
        tf2_ros::fromRclcpp(stamp), odom_frame_,
        tf2::durationFromSec(transform_tolerance_)));
  } catch (const tf2::TransformException & e) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 10000, "no transform: %s", e.what());
    }
    return;
  }

  Observation obs;
  obs.obstacles.reserve(c.obstacles.size());
  obs.floor.reserve(c.floor.size());
  for (const auto & o : c.obstacles) {
    obs.obstacles.push_back((global_from_base * Eigen::Vector3d(o.x(), o.y(), 0.0)).head<2>());
  }
  for (const auto & f : c.floor) {
    obs.floor.push_back((global_from_base * Eigen::Vector3d(f.x(), f.y(), 0.0)).head<2>());
  }
  std::lock_guard<std::mutex> lock(mutex_);
  pending_ = std::move(obs);
}

void DepthObstacleLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (rolling_window_) {
    // Keep the layer's grid centred on the robot, like the obstacle layer.
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
  std::optional<Observation> obs;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    obs.swap(pending_);
  }
  if (!obs) {
    return;
  }
  unsigned int mx = 0;
  unsigned int my = 0;
  // Floor first, obstacles second: a cell with both is an obstacle.
  for (const auto & f : obs->floor) {
    if (worldToMap(f.x(), f.y(), mx, my)) {
      setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
      touch(f.x(), f.y(), min_x, min_y, max_x, max_y);
    }
  }
  for (const auto & o : obs->obstacles) {
    if (worldToMap(o.x(), o.y(), mx, my)) {
      setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      touch(o.x(), o.y(), min_x, min_y, max_x, max_y);
    }
  }
}

void DepthObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  // Max: this layer can add obstacles but never clear the lidar's.
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

// Nav2's recoveries clear the costmaps entirely (Humble resets every layer,
// clearable or not). This layer's memory is what covers the camera's blind
// zone: close to a bed edge the camera no longer sees it, so forgetting it
// made the planner find a path straight into it (6b test, 2026-10-15). Old
// marks are still cleared, by the floor seen again in those cells.
void DepthObstacleLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.reset();
  current_ = true;
}

}  // namespace landerpi_nav2_depth_layer

PLUGINLIB_EXPORT_CLASS(landerpi_nav2_depth_layer::DepthObstacleLayer, nav2_costmap_2d::Layer)
