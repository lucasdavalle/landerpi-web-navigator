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


#include "landerpi_web_nav/crouch_coordinator_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"

namespace landerpi_web_nav
{

CrouchCoordinatorNode::CrouchCoordinatorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("crouch_coordinator", options)
{
  declare_parameter<double>("lookahead", 1.0);
  declare_parameter<double>("corridor", 0.15);
  declare_parameter<double>("near_radius", 0.5);
  declare_parameter<double>("retry_distance", 0.3);
  declare_parameter<double>("request_timeout", 3.0);
  declare_parameter<double>("plan_timeout", 3.0);
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("base_frame", "base_footprint");

  init_finish_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/init_finish",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      res->success =
      get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    });
}

CrouchCoordinatorNode::CallbackReturn CrouchCoordinatorNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  CrouchPlannerConfig cfg;
  cfg.lookahead = get_parameter("lookahead").as_double();
  cfg.corridor = get_parameter("corridor").as_double();
  cfg.near_radius = get_parameter("near_radius").as_double();
  cfg.retry_distance = get_parameter("retry_distance").as_double();
  cfg.request_timeout = get_parameter("request_timeout").as_double();
  plan_timeout_ = get_parameter("plan_timeout").as_double();
  map_frame_ = get_parameter("map_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  try {
    if (!(plan_timeout_ > 0.0)) {
      throw std::invalid_argument("plan_timeout must be > 0");
    }
    planner_ = std::make_unique<CrouchPlanner>(cfg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "configure failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  const auto latched = rclcpp::QoS(1).reliable().transient_local();

  plan_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/plan", rclcpp::QoS(1).reliable(), [this](nav_msgs::msg::Path::SharedPtr m) {
      std::vector<Point2> path;
      path.reserve(m->poses.size());
      for (const auto & p : m->poses) {
        path.push_back({p.pose.position.x, p.pose.position.y});
      }
      std::lock_guard<std::mutex> lock(mutex_);
      path_ = std::move(path);
      path_received_ = steady_.now();
    });
  cells_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/a7/crouch_cells", latched, [this](sensor_msgs::msg::PointCloud2::SharedPtr m) {
      std::vector<Point2> cells;
      cells.reserve(static_cast<std::size_t>(m->width) * m->height);
      sensor_msgs::PointCloud2ConstIterator<float> x(*m, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*m, "y");
      for (; x != x.end(); ++x, ++y) {
        cells.push_back({*x, *y});
      }
      std::lock_guard<std::mutex> lock(mutex_);
      crouch_cells_ = std::move(cells);
    });
  posture_sub_ = create_subscription<std_msgs::msg::String>(
    "/a7/posture", latched, [this](std_msgs::msg::String::SharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);
      posture_ = m->data;
    });
  check_sub_ = create_subscription<std_msgs::msg::String>(
    "/a7/stand_check", latched,
    [this](std_msgs::msg::String::SharedPtr m) {onStandCheck(m->data);});
  status_pub_ = create_publisher<std_msgs::msg::String>("/a7/crouch_status", latched);
  go_low_client_ = create_client<std_srvs::srv::Trigger>("/posture_manager/go_low");
  try_stand_client_ = create_client<std_srvs::srv::Trigger>("/posture_manager/try_stand");

  timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {onTimer();});
  timer_->cancel();  // runs only while active
  return CallbackReturn::SUCCESS;
}

CrouchCoordinatorNode::CallbackReturn CrouchCoordinatorNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  timer_->reset();
  return CallbackReturn::SUCCESS;
}

CrouchCoordinatorNode::CallbackReturn CrouchCoordinatorNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  timer_->cancel();
  return CallbackReturn::SUCCESS;
}

CrouchCoordinatorNode::CallbackReturn CrouchCoordinatorNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  timer_.reset();
  plan_sub_.reset();
  cells_sub_.reset();
  posture_sub_.reset();
  check_sub_.reset();
  go_low_client_.reset();
  try_stand_client_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  planner_.reset();
  return CallbackReturn::SUCCESS;
}

std::optional<Point2> CrouchCoordinatorNode::robotInMap()
{
  try {
    const auto t = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    return Point2{t.transform.translation.x, t.transform.translation.y};
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "no robot pose: %s", e.what());
    return std::nullopt;
  }
}

void CrouchCoordinatorNode::onStandCheck(const std::string & verdict)
{
  // Only a verdict for a try_stand of ours: the latched one at start-up, or
  // one from the web's enable, says nothing about this navigation.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!stand_pending_) {
      return;
    }
    stand_pending_ = false;
  }
  if (verdict.rfind("clear", 0) == 0) {
    return;
  }
  const auto robot = robotInMap();
  if (robot) {
    std::lock_guard<std::mutex> lock(mutex_);
    planner_->standRefused(*robot);
  }
  publishStatus("staying crouched: " + verdict);
}

void CrouchCoordinatorNode::call(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client, const std::string & what)
{
  if (!client->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000, "posture_manager not available for %s", what.c_str());
    return;
  }
  client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this, what](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
      if (!f.get()->success) {
        RCLCPP_WARN(get_logger(), "%s refused: %s", what.c_str(), f.get()->message.c_str());
      }
    });
}

void CrouchCoordinatorNode::publishStatus(const std::string & status)
{
  if (status == last_status_) {
    return;
  }
  last_status_ = status;
  std_msgs::msg::String s;
  s.data = status;
  status_pub_->publish(s);
  RCLCPP_INFO(get_logger(), "%s", status.c_str());
}

void CrouchCoordinatorNode::onTimer()
{
  const auto robot = robotInMap();
  if (!robot) {
    return;
  }
  CrouchDecision d;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // A plan stops arriving when the navigation ends (or is cancelled).
    const bool navigating = (steady_.now() - path_received_).seconds() < plan_timeout_;
    const std::vector<Point2> no_path;
    const double t = steady_.now().seconds();
    d = planner_->update(
      t, parsePosture(posture_), *robot, navigating ? path_ : no_path, crouch_cells_);
    if (d.action == CrouchAction::kStand) {
      stand_pending_ = true;
    }
  }
  switch (d.action) {
    case CrouchAction::kCrouch:
      call(go_low_client_, "go_low");
      publishStatus(d.why);
      break;
    case CrouchAction::kStand:
      call(try_stand_client_, "try_stand");
      publishStatus(d.why);
      break;
    case CrouchAction::kNone:
      break;
  }
}

}  // namespace landerpi_web_nav
