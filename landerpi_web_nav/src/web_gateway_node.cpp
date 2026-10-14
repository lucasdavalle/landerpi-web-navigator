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

#include "landerpi_web_nav/web_gateway_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "landerpi_web_nav/scan_projector.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/exceptions.h"

namespace landerpi_web_nav
{

namespace
{

using nlohmann::json;

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

HttpResponse jsonResponse(int status, const json & body)
{
  return {status, "application/json", body.dump()};
}

std::string expandHome(const std::string & path)
{
  if (!path.empty() && path[0] == '~') {
    const char * home = std::getenv("HOME");
    return std::string(home ? home : "") + path.substr(1);
  }
  return path;
}

}  // namespace

WebGatewayNode::WebGatewayNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("web_gateway", options)
{
  declare_parameter<std::string>("address", "0.0.0.0");
  declare_parameter<int>("port", 8000);
  declare_parameter<std::vector<std::string>>(
    "allowed_networks", {"192.168.100.0/24", "192.168.149.0/24", "127.0.0.1"});
  declare_parameter<std::string>("password_file", "~/.a7/password");
  declare_parameter<std::string>("static_root", "");
  declare_parameter<int>("token_ttl_s", 12 * 3600);
  declare_parameter<int>("max_login_failures", 5);
  declare_parameter<double>("pose_rate_hz", 10.0);
  declare_parameter<double>("scan_rate_hz", 5.0);
  declare_parameter<double>("status_rate_hz", 1.0);
  declare_parameter<int>("scan_stride", 2);
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("base_frame", "base_footprint");

  init_finish_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/init_finish",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success =
      get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    });
}

WebGatewayNode::CallbackReturn WebGatewayNode::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    // Password: only its PBKDF2 hash lives on disk (scripts/a7_set_password).
    const auto password_path = expandHome(get_parameter("password_file").as_string());
    std::ifstream file(password_path);
    std::string line;
    if (!file || !std::getline(file, line)) {
      RCLCPP_ERROR(
        get_logger(), "no password file at %s: run a7_set_password first",
        password_path.c_str());
      return CallbackReturn::FAILURE;
    }
    AuthConfig auth_cfg;
    auth_cfg.token_ttl = std::chrono::seconds(get_parameter("token_ttl_s").as_int());
    auth_cfg.max_failures = static_cast<int>(get_parameter("max_login_failures").as_int());
    auth_ = std::make_unique<AuthManager>(PasswordHash::parse(line), auth_cfg);
    ip_filter_ = std::make_unique<IpFilter>(get_parameter("allowed_networks").as_string_array());

    pose_rate_hz_ = get_parameter("pose_rate_hz").as_double();
    scan_rate_hz_ = get_parameter("scan_rate_hz").as_double();
    status_rate_hz_ = get_parameter("status_rate_hz").as_double();
    scan_stride_ = static_cast<int>(get_parameter("scan_stride").as_int());
    for (double r : {pose_rate_hz_, scan_rate_hz_, status_rate_hz_}) {
      if (!(r > 0.0 && r <= 30.0)) {
        throw std::invalid_argument("rates must be in (0, 30] Hz");
      }
    }
    if (scan_stride_ < 1) {
      throw std::invalid_argument("scan_stride must be >= 1");
    }
    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    const int port = static_cast<int>(get_parameter("port").as_int());
    if (port < 1024 || port > 65535) {
      throw std::invalid_argument("port must be in [1024, 65535]");
    }
    WebServerConfig cfg;
    cfg.address = get_parameter("address").as_string();
    cfg.port = static_cast<uint16_t>(port);
    cfg.static_root = get_parameter("static_root").as_string();

    WebServerCallbacks cb;
    cb.api = [this](const HttpRequest & r) {return handleApi(r);};
    cb.token_valid = [this](const std::string & t) {return auth_->validate(t);};
    cb.ip_allowed = [this](const std::string & ip) {return ip_filter_->allows(ip);};
    cb.initial_frames = [this]() {return initialFrames();};
    cb.ws_message = [this](WsClientId c, const std::string & t) {onWsMessage(c, t);};
    cb.ws_closed = [this](WsClientId c) {onWsClosed(c);};
    server_ = std::make_unique<WebServer>(cfg, cb);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "configure failed: %s", e.what());
    releaseAll();
    return CallbackReturn::FAILURE;
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // /map: reliable + transient local, as slam_toolbox publishes it.
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {onMap(std::move(m));});
  walls_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
    "/a7/walls", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    [this](visualization_msgs::msg::MarkerArray::SharedPtr m) {onWalls(std::move(m));});
  // /scan_raw best effort, as everywhere in the portfolio.
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan_raw", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::SharedPtr m) {onScan(std::move(m));});
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/pose", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
    [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {onPose(std::move(m));});
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
    [this](nav_msgs::msg::Odometry::SharedPtr) {
      std::lock_guard<std::mutex> lock(mutex_);
      snap_.last_odom = now();
    });
  // Driving goes through velocity_gate, never straight to the wheels.
  teleop_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/a7/cmd_vel_teleop", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  heartbeat_pub_ = create_publisher<std_msgs::msg::Empty>(
    "/a7/heartbeat", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  set_running_client_ = create_client<std_srvs::srv::SetBool>("/velocity_gate/set_running");
  gate_status_sub_ = create_subscription<std_msgs::msg::String>(
    "/a7/gate_status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [this](std_msgs::msg::String::SharedPtr m) {
      bool changed = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        changed = gate_reason_ != m->data;
        gate_reason_ = m->data;
      }
      if (changed && server_) {
        server_->broadcast(WsFrame{false, driveStateJson(), {}});
      }
    });
  // Navigation (stage 5): Nav2's standard action, and the planned path.
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  plan_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/plan", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    [this](nav_msgs::msg::Path::SharedPtr m) {onPlan(std::move(m));});
  battery_sub_ = create_subscription<std_msgs::msg::UInt16>(
    "/ros_robot_controller/battery", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
    [this](std_msgs::msg::UInt16::SharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);
      snap_.battery_mv = m->data;
    });

  RCLCPP_INFO(
    get_logger(), "configured (password from %s)",
    get_parameter("password_file").as_string().c_str());
  return CallbackReturn::SUCCESS;
}

WebGatewayNode::CallbackReturn WebGatewayNode::on_activate(const rclcpp_lifecycle::State &)
{
  try {
    server_->start();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "cannot start the web server: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  using std::chrono::duration;
  pose_timer_ = create_wall_timer(
    duration<double>(1.0 / pose_rate_hz_), [this]() {publishPose();});
  scan_timer_ = create_wall_timer(
    duration<double>(1.0 / scan_rate_hz_), [this]() {publishScan();});
  status_timer_ = create_wall_timer(
    duration<double>(1.0 / status_rate_hz_), [this]() {publishStatus();});
  RCLCPP_INFO(
    get_logger(), "serving on http://%s:%ld",
    get_parameter("address").as_string().c_str(), get_parameter("port").as_int());
  return CallbackReturn::SUCCESS;
}

WebGatewayNode::CallbackReturn WebGatewayNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Whoever was driving loses control; velocity_gate also stops by itself
  // once the heartbeats stop.
  DriveArbiter::Action action = DriveArbiter::Action::kNone;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (arbiter_.driver()) {
      action = arbiter_.disconnected(*arbiter_.driver());
    }
  }
  applyDriveAction(action);
  cancelGoal("gateway deactivated");
  pose_timer_.reset();
  scan_timer_.reset();
  status_timer_.reset();
  if (server_) {
    server_->stop();
  }
  return CallbackReturn::SUCCESS;
}

WebGatewayNode::CallbackReturn WebGatewayNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  releaseAll();
  return CallbackReturn::SUCCESS;
}

WebGatewayNode::CallbackReturn WebGatewayNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  pose_timer_.reset();
  scan_timer_.reset();
  status_timer_.reset();
  releaseAll();
  return CallbackReturn::SUCCESS;
}

void WebGatewayNode::releaseAll()
{
  if (server_) {
    server_->stop();
  }
  server_.reset();
  map_sub_.reset();
  walls_sub_.reset();
  scan_sub_.reset();
  pose_sub_.reset();
  odom_sub_.reset();
  battery_sub_.reset();
  gate_status_sub_.reset();
  plan_sub_.reset();
  nav_client_.reset();
  teleop_pub_.reset();
  heartbeat_pub_.reset();
  set_running_client_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  auth_.reset();
  ip_filter_.reset();
}

// ---------------------------------------------------------------------------
// ROS side: store the latest state.
// ---------------------------------------------------------------------------
void WebGatewayNode::onMap(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  MapMessage m;
  m.width = msg->info.width;
  m.height = msg->info.height;
  m.resolution = static_cast<float>(msg->info.resolution);
  m.origin_x = static_cast<float>(msg->info.origin.position.x);
  m.origin_y = static_cast<float>(msg->info.origin.position.y);
  m.cells = msg->data;
  auto frame = codec::encodeMap(m);  // encoded once, sent to every client
  m.cells.clear();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snap_.map_frame = frame;
    snap_.map_info = m;
    snap_.last_map = now();
  }
  if (server_) {
    server_->broadcast(WsFrame{true, {}, std::move(frame)});
  }
}

void WebGatewayNode::onWalls(visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  // wall_vectorizer publishes a DELETEALL plus one LINE_LIST whose points
  // come in pairs: each pair is a wall.
  std::vector<Segment> walls;
  for (const auto & marker : msg->markers) {
    if (marker.type != visualization_msgs::msg::Marker::LINE_LIST) {
      continue;
    }
    for (std::size_t i = 0; i + 1 < marker.points.size(); i += 2) {
      Segment s;
      s.start = {marker.points[i].x, marker.points[i].y};
      s.end = {marker.points[i + 1].x, marker.points[i + 1].y};
      walls.push_back(s);
    }
  }
  const auto text = codec::wallsJson(walls);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snap_.walls_json = text;
  }
  if (server_) {
    server_->broadcast(WsFrame{false, text, {}});
  }
}

void WebGatewayNode::onScan(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  snap_.scan = std::move(msg);
  snap_.last_scan = now();
}

void WebGatewayNode::onPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // Only the covariance is used: slam_toolbox publishes /pose only when it
  // processes a scan (robot moved), so the position itself comes from TF.
  const auto & c = msg->pose.covariance;
  std::lock_guard<std::mutex> lock(mutex_);
  snap_.position_covariance = std::array<double, 3>{c[0], c[1], c[7]};
  snap_.yaw_variance = c[35];
  snap_.last_pose = now();
}

// ---------------------------------------------------------------------------
// Timers: push to the browsers.
// ---------------------------------------------------------------------------
void WebGatewayNode::publishPose()
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return;  // no SLAM yet
  }
  PoseMessage m;
  m.pose = {tf.transform.translation.x, tf.transform.translation.y, yawOf(tf.transform.rotation)};
  m.stamp = rclcpp::Time(tf.header.stamp).seconds();
  std::string text;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    m.position_covariance = snap_.position_covariance;
    m.yaw_variance = snap_.yaw_variance;
    text = codec::poseJson(m);
    snap_.pose_json = text;
  }
  if (server_->clients() > 0) {
    server_->broadcast(WsFrame{false, text, {}});
  }
}

void WebGatewayNode::publishScan()
{
  if (server_->clients() == 0) {
    return;
  }
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    scan = snap_.scan;
  }
  if (!scan) {
    return;
  }
  geometry_msgs::msg::TransformStamped sensor_tf;
  geometry_msgs::msg::TransformStamped robot_tf;
  try {
    sensor_tf = tf_buffer_->lookupTransform(map_frame_, scan->header.frame_id, tf2::TimePointZero);
    robot_tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return;
  }
  const ScanView view{
    scan->angle_min, scan->angle_increment, scan->range_min, scan->range_max, &scan->ranges};
  const Pose2D sensor{
    sensor_tf.transform.translation.x, sensor_tf.transform.translation.y,
    yawOf(sensor_tf.transform.rotation)};
  ScanMessage m;
  m.robot = {
    robot_tf.transform.translation.x, robot_tf.transform.translation.y,
    yawOf(robot_tf.transform.rotation)};
  m.points = projectScan(view, sensor, scan_stride_);
  server_->broadcast(WsFrame{true, {}, codec::encodeScan(m)});
}

StatusMessage WebGatewayNode::status()
{
  const auto t = now();
  auto age = [&](const std::optional<rclcpp::Time> & last) -> std::optional<double> {
      if (!last) {
        return std::nullopt;
      }
      return (t - *last).seconds();
    };
  StatusMessage s;
  std::lock_guard<std::mutex> lock(mutex_);
  s.battery_mv = snap_.battery_mv;
  s.sensors = {
    {"scan", age(snap_.last_scan)}, {"odom", age(snap_.last_odom)},
    {"map", age(snap_.last_map)}, {"slam_pose", age(snap_.last_pose)}};
  s.slam_running = snap_.last_map && (t - *snap_.last_map).seconds() < 5.0;
  s.clients = server_ ? server_->clients() : 0;
  return s;
}

void WebGatewayNode::publishStatus()
{
  if (server_->clients() > 0) {
    server_->broadcast(WsFrame{false, codec::statusJson(status()), {}});
    server_->broadcast(WsFrame{false, driveStateJson(), {}});
  }
}

// ---------------------------------------------------------------------------
// Driving (stage 4). Runs on the network thread.
// ---------------------------------------------------------------------------
std::string WebGatewayNode::driveStateJson()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto driver = arbiter_.driver();
  return json{
    {"type", "drive"},
    {"driver", driver ? json(*driver) : json(nullptr)},
    {"gate", gate_reason_},
  }.dump();
}

void WebGatewayNode::applyDriveAction(DriveArbiter::Action action)
{
  if (action == DriveArbiter::Action::kNone) {
    return;
  }
  const bool enable = action == DriveArbiter::Action::kEnable;
  if (!enable) {
    stopTeleop();
  }
  if (!set_running_client_ || !set_running_client_->service_is_ready()) {
    // On disable this is not dangerous: velocity_gate stops by itself when
    // the heartbeats stop. On enable the browser will see gate "idle".
    RCLCPP_WARN(get_logger(), "velocity_gate not available (set_running %d)", enable);
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = enable;
  set_running_client_->async_send_request(
    req, [this, enable](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture f) {
      const auto res = f.get();
      if (!res->success) {
        RCLCPP_WARN(get_logger(), "set_running(%d) refused: %s", enable, res->message.c_str());
      }
    });
}

void WebGatewayNode::stopTeleop()
{
  if (teleop_pub_) {
    teleop_pub_->publish(geometry_msgs::msg::Twist());
  }
}

void WebGatewayNode::onWsMessage(WsClientId client, const std::string & text)
{
  // Anything malformed (bad JSON, "vx": "abc", ...) is ignored: a bad
  // message must never take the network thread down.
  try {
    handleWsMessage(client, json::parse(text));
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "ignoring malformed message: %s", e.what());
  }
}

void WebGatewayNode::handleWsMessage(WsClientId client, const nlohmann::json & msg)
{
  const std::string type = msg.value("type", "");

  if (type == "drive") {
    DriveArbiter::Decision d;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      d = arbiter_.request(client, msg.value("enable", false));
    }
    if (d.granted) {
      RCLCPP_INFO(
        get_logger(), "client %lu %s driving", static_cast<unsigned long>(client),  // NOLINT
        msg.value("enable", false) ? "is now" : "stopped");
    }
    applyDriveAction(d.action);
    if (d.action == DriveArbiter::Action::kDisable) {
      cancelGoal("driver handed back control");
    }
    server_->broadcast(WsFrame{false, driveStateJson(), {}});
    return;
  }

  bool is_driver = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    is_driver = arbiter_.isDriver(client);
  }
  if (!is_driver) {
    return;  // viewers cannot move the robot
  }
  if (type == "heartbeat") {
    heartbeat_pub_->publish(std_msgs::msg::Empty());
  } else if (type == "teleop") {
    geometry_msgs::msg::Twist t;
    // velocity_gate clamps and validates again; this only avoids
    // forwarding obvious garbage.
    t.linear.x = msg.value("vx", 0.0);
    t.linear.y = msg.value("vy", 0.0);
    t.angular.z = msg.value("wz", 0.0);
    // Taking the wheel by hand ends autonomous navigation (user decision):
    // it must not resume on its own when the key is released.
    if (std::abs(t.linear.x) + std::abs(t.linear.y) + std::abs(t.angular.z) > 1e-3) {
      cancelGoal("manual teleop");
    }
    teleop_pub_->publish(t);
  } else if (type == "goal") {
    const double x = msg.at("x").get<double>();
    const double y = msg.at("y").get<double>();
    if (!std::isfinite(x) || !std::isfinite(y)) {
      return;
    }
    sendGoal(x, y);
  } else if (type == "cancel") {
    cancelGoal("cancel from the browser");
  }
}

// ---------------------------------------------------------------------------
// Navigation (stage 5).
// ---------------------------------------------------------------------------
std::string WebGatewayNode::navJsonLocked() const
{
  return json{
    {"type", "nav"},
    {"state", nav_.state},
    {"goal", nav_.goal ? json({(*nav_.goal)[0], (*nav_.goal)[1]}) : json(nullptr)},
    {"distance", nav_.distance ? json(*nav_.distance) : json(nullptr)},
    {"eta", nav_.eta ? json(*nav_.eta) : json(nullptr)},
    {"recoveries", nav_.recoveries},
  }.dump();
}

std::string WebGatewayNode::navJson()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return navJsonLocked();
}

void WebGatewayNode::broadcastNav()
{
  if (server_) {
    server_->broadcast(WsFrame{false, navJson(), {}});
  }
}

void WebGatewayNode::sendGoal(double x, double y)
{
  if (!nav_client_ || !nav_client_->action_server_is_ready()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      nav_ = NavState{};
      nav_.state = "unavailable";  // Nav2 not running
    }
    broadcastNav();
    return;
  }
  uint64_t seq = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    seq = ++goal_seq_;
    nav_ = NavState{};
    nav_.state = "pending";
    nav_.goal = std::array<double, 2>{x, y};
  }
  broadcastNav();
  RCLCPP_INFO(get_logger(), "goal (%.2f, %.2f)", x, y);

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = map_frame_;
  goal.pose.header.stamp = now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.orientation.w = 1.0;  // any final heading (yaw tolerance in nav2.yaml)

  // Every callback carries the sequence number of its goal: a newer goal
  // (or a cancel) bumps goal_seq_, and callbacks of the old one are ignored.
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
  opts.goal_response_callback = [this, seq](NavGoalHandle::SharedPtr gh) {
      bool superseded = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        superseded = seq != goal_seq_;
        if (!superseded) {
          goal_handle_ = gh;
          nav_.state = gh ? "active" : "rejected";
        }
      }
      if (superseded) {
        if (gh) {
          nav_client_->async_cancel_goal(gh);  // cancelled before it was accepted
        }
        return;
      }
      broadcastNav();
    };
  opts.feedback_callback = [this, seq](
    NavGoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
      bool send = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (seq != goal_seq_) {
          return;
        }
        nav_.distance = fb->distance_remaining;
        nav_.eta = rclcpp::Duration(fb->estimated_time_remaining).seconds();
        nav_.recoveries = fb->number_of_recoveries;
        // Nav2 sends feedback at the controller rate: 2 Hz is plenty here.
        const rclcpp::Time t = steady_clock_.now();
        if ((t - last_feedback_broadcast_).seconds() >= 0.5) {
          last_feedback_broadcast_ = t;
          send = true;
        }
      }
      if (send) {
        broadcastNav();
      }
    };
  opts.result_callback = [this, seq](const NavGoalHandle::WrappedResult & r) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (seq != goal_seq_) {
          return;
        }
        goal_handle_.reset();
        switch (r.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: nav_.state = "succeeded"; break;
          case rclcpp_action::ResultCode::ABORTED: nav_.state = "aborted"; break;
          case rclcpp_action::ResultCode::CANCELED: nav_.state = "canceled"; break;
          default: nav_.state = "unknown"; break;
        }
      }
      RCLCPP_INFO(get_logger(), "navigation finished: %s", navJson().c_str());
      broadcastNav();
    };
  nav_client_->async_send_goal(goal, opts);
}

void WebGatewayNode::cancelGoal(const char * why)
{
  NavGoalHandle::SharedPtr gh;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (nav_.state != "pending" && nav_.state != "active") {
      return;  // nothing running
    }
    gh = goal_handle_;
    goal_handle_.reset();
    ++goal_seq_;  // late callbacks of this goal are ignored from now on
    nav_.state = "canceled";
  }
  RCLCPP_INFO(get_logger(), "goal canceled: %s", why);
  if (gh && nav_client_) {
    nav_client_->async_cancel_goal(gh);
  }
  broadcastNav();
}

void WebGatewayNode::onPlan(nav_msgs::msg::Path::SharedPtr msg)
{
  // Downsample to at most ~150 points: plenty to draw, small to send.
  const std::size_t n = msg->poses.size();
  const std::size_t step = std::max<std::size_t>(1, n / 150);
  json points = json::array();
  for (std::size_t i = 0; i < n; i += step) {
    const auto & p = msg->poses[i].pose.position;
    points.push_back({p.x, p.y});
  }
  if (n > 0 && (n - 1) % step != 0) {
    const auto & p = msg->poses.back().pose.position;
    points.push_back({p.x, p.y});  // always keep the goal end
  }
  if (server_) {
    server_->broadcast(WsFrame{false, json{{"type", "path"}, {"points", points}}.dump(), {}});
  }
}

void WebGatewayNode::onWsClosed(WsClientId client)
{
  DriveArbiter::Action action;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    action = arbiter_.disconnected(client);
  }
  if (action != DriveArbiter::Action::kNone) {
    RCLCPP_WARN(
      get_logger(), "driver %lu disconnected: stopping", static_cast<unsigned long>(client));  // NOLINT
    applyDriveAction(action);
    cancelGoal("driver disconnected");
    if (server_) {
      server_->broadcast(WsFrame{false, driveStateJson(), {}});
    }
  }
}

// ---------------------------------------------------------------------------
// Network side (runs on the web server thread).
// ---------------------------------------------------------------------------
std::vector<WsFrame> WebGatewayNode::initialFrames()
{
  // A browser that just connected gets the full picture at once instead of
  // waiting for the next map/walls update (up to a second away).
  std::vector<WsFrame> out;
  std::lock_guard<std::mutex> lock(mutex_);
  if (snap_.map_frame) {
    out.push_back(WsFrame{true, {}, *snap_.map_frame});
  }
  out.push_back(WsFrame{false, snap_.walls_json, {}});
  if (snap_.pose_json) {
    out.push_back(WsFrame{false, *snap_.pose_json, {}});
  }
  out.push_back(WsFrame{false, navJsonLocked(), {}});
  const auto driver = arbiter_.driver();
  out.push_back(
    WsFrame{false, json{
        {"type", "drive"},
        {"driver", driver ? json(*driver) : json(nullptr)},
        {"gate", gate_reason_}}.dump(), {}});
  return out;
}

HttpResponse WebGatewayNode::handleApi(const HttpRequest & req)
{
  const std::string path = req.target.substr(0, req.target.find('?'));

  if (path == "/api/login") {
    if (req.method != "POST") {
      return jsonResponse(405, {{"error", "use POST"}});
    }
    std::string password;
    try {
      password = json::parse(req.body).at("password").get<std::string>();
    } catch (const std::exception &) {
      return jsonResponse(400, {{"error", "body must be {\"password\": \"...\"}"}});
    }
    const auto r = auth_->login(password, req.client_ip);
    switch (r.status) {
      case LoginStatus::kOk:
        RCLCPP_INFO(get_logger(), "login ok from %s", req.client_ip.c_str());
        return jsonResponse(200, {{"token", r.token}, {"expires_in", r.expires_in.count()}});
      case LoginStatus::kLockedOut:
        RCLCPP_WARN(get_logger(), "login locked out for %s", req.client_ip.c_str());
        return jsonResponse(429, {{"error", "too many attempts, wait a minute"}});
      default:
        RCLCPP_WARN(get_logger(), "wrong password from %s", req.client_ip.c_str());
        return jsonResponse(401, {{"error", "wrong password"}});
    }
  }

  // Everything else needs a valid token.
  if (!auth_->validate(req.bearer_token)) {
    return jsonResponse(401, {{"error", "missing or expired token"}});
  }
  if (path == "/api/logout" && req.method == "POST") {
    auth_->logout(req.bearer_token);
    return jsonResponse(200, json::object());
  }
  if (req.method != "GET") {
    return jsonResponse(405, {{"error", "read-only API"}});
  }
  if (path == "/api/status") {
    return {200, "application/json", codec::statusJson(status())};
  }
  if (path == "/api/map") {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!snap_.map_info) {
      return jsonResponse(404, {{"error", "no map yet"}});
    }
    return {200, "application/json", codec::mapInfoJson(*snap_.map_info)};
  }
  if (path == "/api/walls") {
    std::lock_guard<std::mutex> lock(mutex_);
    return {200, "application/json", snap_.walls_json};
  }
  return jsonResponse(404, {{"error", "unknown endpoint"}});
}

}  // namespace landerpi_web_nav
