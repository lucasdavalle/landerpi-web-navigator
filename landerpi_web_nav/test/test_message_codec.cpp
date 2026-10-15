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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "landerpi_web_nav/message_codec.hpp"

namespace codec = landerpi_web_nav::codec;
using landerpi_web_nav::MapMessage;
using landerpi_web_nav::PoseMessage;
using landerpi_web_nav::ScanMessage;
using landerpi_web_nav::Segment;
using landerpi_web_nav::SensorAge;
using landerpi_web_nav::StatusMessage;
using nlohmann::json;

TEST(MessageCodec, PoseWithAndWithoutCovariance)
{
  PoseMessage m;
  m.pose = {1.5, -2.0, 0.25};
  m.stamp = 12.5;
  auto j = json::parse(codec::poseJson(m));
  EXPECT_EQ(j["type"], "pose");
  EXPECT_DOUBLE_EQ(j["x"], 1.5);
  EXPECT_DOUBLE_EQ(j["yaw"], 0.25);
  EXPECT_FALSE(j.contains("cov"));  // SLAM has not published one yet

  m.position_covariance = std::array<double, 3>{0.01, 0.002, 0.03};
  m.yaw_variance = 0.004;
  j = json::parse(codec::poseJson(m));
  ASSERT_TRUE(j.contains("cov"));
  EXPECT_DOUBLE_EQ(j["cov"][2], 0.03);
  EXPECT_DOUBLE_EQ(j["cov_yaw"], 0.004);
}

TEST(MessageCodec, PoseRefusesNonFiniteValues)
{
  PoseMessage m;
  m.pose.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(codec::poseJson(m), std::invalid_argument);
}

TEST(MessageCodec, WallsAsFlatQuads)
{
  std::vector<Segment> walls(2);
  walls[0].start = {0, 0};
  walls[0].end = {2, 0};
  walls[1].start = {2, 0};
  walls[1].end = {2, 3};
  const auto j = json::parse(codec::wallsJson(walls));
  ASSERT_EQ(j["segments"].size(), 2u);
  EXPECT_EQ(j["segments"][1], json({2.0, 0.0, 2.0, 3.0}));
}

TEST(MessageCodec, StatusWithMissingValuesAsNull)
{
  StatusMessage m;
  m.sensors = {SensorAge{"scan", 0.1}, SensorAge{"camera", std::nullopt}};
  m.slam_running = true;
  const auto j = json::parse(codec::statusJson(m));
  EXPECT_TRUE(j["battery_mv"].is_null());
  EXPECT_DOUBLE_EQ(j["sensors"]["scan"], 0.1);
  EXPECT_TRUE(j["sensors"]["camera"].is_null());
  EXPECT_TRUE(j["slam_running"]);
}

TEST(MessageCodec, MapRoundTrip)
{
  MapMessage m;
  m.width = 3;
  m.height = 2;
  m.resolution = 0.05f;
  m.origin_x = -3.62f;
  m.origin_y = -4.34f;
  m.cells = {-1, 0, 100, 65, 0, -1};
  const auto frame = codec::encodeMap(m);
  EXPECT_EQ(frame.size(), 24u + 6u);
  EXPECT_EQ(frame[0], 1);  // type map
  EXPECT_EQ(frame[1], 1);  // version
  const auto back = codec::decodeMap(frame);
  EXPECT_EQ(back.width, 3u);
  EXPECT_FLOAT_EQ(back.origin_x, -3.62f);
  EXPECT_EQ(back.cells, m.cells);
}

// Stage 6c: the camera's grid (crouchable / camera-only obstacles) travels
// like the map, with its own type so the browser draws it as an overlay.
TEST(MessageCodec, DepthGridRoundTripAndTypeCheck)
{
  MapMessage m;
  m.width = 2;
  m.height = 2;
  m.resolution = 0.05f;
  m.cells = {0, 50, 100, 0};
  const auto frame = codec::encodeMap(m, landerpi_web_nav::BinaryType::kDepthGrid);
  EXPECT_EQ(frame[0], 3);
  const auto back = codec::decodeMap(frame, landerpi_web_nav::BinaryType::kDepthGrid);
  EXPECT_EQ(back.cells, m.cells);
  EXPECT_THROW(codec::decodeMap(frame), std::invalid_argument);  // not a map frame
  EXPECT_THROW(
    codec::encodeMap(m, landerpi_web_nav::BinaryType::kScan), std::invalid_argument);
}

TEST(MessageCodec, MapIsLittleEndian)
{
  MapMessage m;
  m.width = 0x01020304;  // never encoded with matching cells, header only
  m.height = 0;
  const auto frame = codec::encodeMap(m);
  EXPECT_EQ(frame[4], 0x04);
  EXPECT_EQ(frame[7], 0x01);
}

TEST(MessageCodec, ScanRoundTrip)
{
  ScanMessage m;
  m.robot = {1.0, 2.0, 0.5};
  m.points = {{0.1, 0.2}, {-3.5, 4.25}};
  const auto frame = codec::encodeScan(m);
  EXPECT_EQ(frame.size(), 20u + 2u * 8u);
  const auto back = codec::decodeScan(frame);
  ASSERT_EQ(back.points.size(), 2u);
  EXPECT_FLOAT_EQ(static_cast<float>(back.points[1].x), -3.5f);
  EXPECT_FLOAT_EQ(static_cast<float>(back.robot.yaw), 0.5f);
}

TEST(MessageCodec, RejectsBadFrames)
{
  MapMessage m;
  m.width = 2;
  m.height = 2;
  m.cells = {0, 0, 0};  // one short
  EXPECT_THROW(codec::encodeMap(m), std::invalid_argument);

  ScanMessage s;
  s.points = {{1, 1}};
  auto frame = codec::encodeScan(s);
  frame.pop_back();  // truncated
  EXPECT_THROW(codec::decodeScan(frame), std::invalid_argument);
  EXPECT_THROW(codec::decodeMap(codec::encodeScan(s)), std::invalid_argument);  // wrong type
  EXPECT_THROW(codec::decodeMap({}), std::invalid_argument);
}
