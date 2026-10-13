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

#include "landerpi_web_nav/message_codec.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace landerpi_web_nav
{
namespace codec
{

namespace
{

using nlohmann::json;

// Explicit little-endian writes/reads with shifts: correct on any host,
// independent of the CPU's byte order.
void putU32(std::vector<uint8_t> & out, uint32_t v)
{
  for (int i = 0; i < 4; ++i) {
    out.push_back(static_cast<uint8_t>(v >> (8 * i)));
  }
}

void putF32(std::vector<uint8_t> & out, float f)
{
  uint32_t bits = 0;
  std::memcpy(&bits, &f, sizeof(bits));  // the defined way to reinterpret
  putU32(out, bits);
}

void putPreamble(std::vector<uint8_t> & out, BinaryType type)
{
  out.push_back(static_cast<uint8_t>(type));
  out.push_back(kBinaryVersion);
  out.push_back(0);
  out.push_back(0);
}

class Reader
{
public:
  explicit Reader(const std::vector<uint8_t> & data)
  : data_(data) {}

  void expectPreamble(BinaryType type)
  {
    need(4);
    if (data_[0] != static_cast<uint8_t>(type) || data_[1] != kBinaryVersion) {
      throw std::invalid_argument("unexpected frame type or version");
    }
    pos_ = 4;
  }
  uint32_t u32()
  {
    need(pos_ + 4);
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) {
      v |= static_cast<uint32_t>(data_[pos_ + i]) << (8 * i);
    }
    pos_ += 4;
    return v;
  }
  float f32()
  {
    const uint32_t bits = u32();
    float f = 0.0f;
    std::memcpy(&f, &bits, sizeof(f));
    return f;
  }
  std::size_t remaining() const {return data_.size() - pos_;}
  std::size_t pos() const {return pos_;}

private:
  void need(std::size_t n) const
  {
    if (data_.size() < n) {
      throw std::invalid_argument("truncated frame");
    }
  }
  const std::vector<uint8_t> & data_;
  std::size_t pos_ = 0;
};

// JSON cannot hold NaN/inf: nlohmann would write null, which the browser
// would then read as 0. Better to fail loudly upstream.
double finite(double v, const char * what)
{
  if (!std::isfinite(v)) {
    throw std::invalid_argument(std::string("non-finite ") + what);
  }
  return v;
}

}  // namespace

std::string poseJson(const PoseMessage & m)
{
  json j{
    {"type", "pose"},
    {"x", finite(m.pose.x, "x")},
    {"y", finite(m.pose.y, "y")},
    {"yaw", finite(m.pose.yaw, "yaw")},
    {"t", m.stamp},
  };
  if (m.position_covariance) {
    const auto & c = *m.position_covariance;
    j["cov"] = {finite(c[0], "cov"), finite(c[1], "cov"), finite(c[2], "cov")};
  }
  if (m.yaw_variance) {
    j["cov_yaw"] = finite(*m.yaw_variance, "cov_yaw");
  }
  return j.dump();
}

std::string wallsJson(const std::vector<Segment> & walls)
{
  json segs = json::array();
  for (const auto & s : walls) {
    segs.push_back(
      {finite(s.start.x, "wall"), finite(s.start.y, "wall"),
        finite(s.end.x, "wall"), finite(s.end.y, "wall")});
  }
  return json{{"type", "walls"}, {"segments", segs}}.dump();
}

std::string statusJson(const StatusMessage & m)
{
  json sensors = json::object();
  for (const auto & s : m.sensors) {
    sensors[s.name] = s.age_s ? json(*s.age_s) : json(nullptr);
  }
  json j{
    {"type", "status"},
    {"sensors", sensors},
    {"slam_running", m.slam_running},
    {"clients", m.clients},
  };
  j["battery_mv"] = m.battery_mv ? json(*m.battery_mv) : json(nullptr);
  return j.dump();
}

std::string mapInfoJson(const MapMessage & m)
{
  return json{
    {"type", "map_info"},
    {"width", m.width},
    {"height", m.height},
    {"resolution", m.resolution},
    {"origin", {m.origin_x, m.origin_y}},
  }.dump();
}

std::string errorJson(const std::string & message)
{
  return json{{"type", "error"}, {"message", message}}.dump();
}

std::vector<uint8_t> encodeMap(const MapMessage & m)
{
  if (m.cells.size() != static_cast<std::size_t>(m.width) * m.height) {
    throw std::invalid_argument("map cells do not match width * height");
  }
  std::vector<uint8_t> out;
  out.reserve(24 + m.cells.size());
  putPreamble(out, BinaryType::kMap);
  putU32(out, m.width);
  putU32(out, m.height);
  putF32(out, m.resolution);
  putF32(out, m.origin_x);
  putF32(out, m.origin_y);
  for (int8_t c : m.cells) {
    out.push_back(static_cast<uint8_t>(c));
  }
  return out;
}

std::vector<uint8_t> encodeScan(const ScanMessage & m)
{
  std::vector<uint8_t> out;
  out.reserve(20 + 8 * m.points.size());
  putPreamble(out, BinaryType::kScan);
  putU32(out, static_cast<uint32_t>(m.points.size()));
  putF32(out, static_cast<float>(m.robot.x));
  putF32(out, static_cast<float>(m.robot.y));
  putF32(out, static_cast<float>(m.robot.yaw));
  for (const auto & p : m.points) {
    putF32(out, static_cast<float>(p.x));
    putF32(out, static_cast<float>(p.y));
  }
  return out;
}

MapMessage decodeMap(const std::vector<uint8_t> & frame)
{
  Reader r(frame);
  r.expectPreamble(BinaryType::kMap);
  MapMessage m;
  m.width = r.u32();
  m.height = r.u32();
  m.resolution = r.f32();
  m.origin_x = r.f32();
  m.origin_y = r.f32();
  const std::size_t n = static_cast<std::size_t>(m.width) * m.height;
  if (r.remaining() != n) {
    throw std::invalid_argument("map frame size does not match its header");
  }
  m.cells.reserve(n);
  for (std::size_t i = r.pos(); i < frame.size(); ++i) {
    m.cells.push_back(static_cast<int8_t>(frame[i]));
  }
  return m;
}

ScanMessage decodeScan(const std::vector<uint8_t> & frame)
{
  Reader r(frame);
  r.expectPreamble(BinaryType::kScan);
  ScanMessage m;
  const uint32_t count = r.u32();
  m.robot.x = r.f32();
  m.robot.y = r.f32();
  m.robot.yaw = r.f32();
  if (r.remaining() != static_cast<std::size_t>(count) * 8) {
    throw std::invalid_argument("scan frame size does not match its header");
  }
  m.points.reserve(count);
  for (uint32_t i = 0; i < count; ++i) {
    const double x = r.f32();
    const double y = r.f32();
    m.points.push_back({x, y});
  }
  return m;
}

}  // namespace codec
}  // namespace landerpi_web_nav
