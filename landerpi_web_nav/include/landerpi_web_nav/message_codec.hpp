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

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "landerpi_web_nav/line_extractor.hpp"

namespace landerpi_web_nav
{

// Everything that travels to the browser, kept free of ROS types so it can
// be tested without ROS. Small, readable things go as JSON text frames;
// the heavy ones (map grid, scan points) as binary frames, which the
// browser reads straight into typed arrays.
//
// Binary frames are little-endian and start with a 4-byte preamble:
//   u8 type, u8 version, u16 reserved (0)
// Map  (type 1): u32 width, u32 height, f32 resolution, f32 origin_x,
//                f32 origin_y, then width*height int8 cells (row 0 = lowest y)
// Scan (type 2): u32 count, f32 robot_x, f32 robot_y, f32 robot_yaw,
//                then count * (f32 x, f32 y) points in the map frame

constexpr uint8_t kBinaryVersion = 1;

enum class BinaryType : uint8_t
{
  kMap = 1,
  kScan = 2,
};

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct PoseMessage
{
  Pose2D pose;
  // Position covariance (xx, xy, yy) in m^2 and yaw variance in rad^2, for
  // the uncertainty ellipse. Absent until SLAM has published one.
  std::optional<std::array<double, 3>> position_covariance;
  std::optional<double> yaw_variance;
  double stamp = 0.0;  // s
};

struct MapMessage
{
  uint32_t width = 0;
  uint32_t height = 0;
  float resolution = 0.0f;
  float origin_x = 0.0f;
  float origin_y = 0.0f;
  std::vector<int8_t> cells;
};

struct ScanMessage
{
  Pose2D robot;
  std::vector<Point2> points;
};

struct SensorAge
{
  std::string name;
  std::optional<double> age_s;  // seconds since the last message; none = never
};

struct StatusMessage
{
  std::optional<int> battery_mv;
  std::vector<SensorAge> sensors;
  bool slam_running = false;
  std::size_t clients = 0;
};

namespace codec
{

std::string poseJson(const PoseMessage & m);
std::string wallsJson(const std::vector<Segment> & walls);
std::string statusJson(const StatusMessage & m);
std::string mapInfoJson(const MapMessage & m);  // metadata only, no cells
std::string errorJson(const std::string & message);

std::vector<uint8_t> encodeMap(const MapMessage & m);
std::vector<uint8_t> encodeScan(const ScanMessage & m);
// Inverses, for tests and debugging tools. Throw std::invalid_argument on a
// truncated or foreign frame.
MapMessage decodeMap(const std::vector<uint8_t> & frame);
ScanMessage decodeScan(const std::vector<uint8_t> & frame);

}  // namespace codec

}  // namespace landerpi_web_nav
