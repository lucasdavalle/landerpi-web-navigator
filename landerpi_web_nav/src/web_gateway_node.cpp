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
