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


#include "landerpi_web_nav/crouch_planner.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace landerpi_web_nav
{

namespace
{

double dist(const Point2 & a, const Point2 & b) {return std::hypot(a.x - b.x, a.y - b.y);}

}  // namespace

ArmPosture parsePosture(const std::string & state)
{
  if (state == "high") {
    return ArmPosture::kHigh;
  }
  if (state == "low") {
    return ArmPosture::kLow;
  }
  if (state == "none" || state.empty()) {
    return ArmPosture::kNone;
  }
  return ArmPosture::kBusy;
}

CrouchPlanner::CrouchPlanner(const CrouchPlannerConfig & config)
: config_(config)
{
  for (const double v : {config.lookahead, config.corridor, config.near_radius,
      config.retry_distance, config.arm_clearance, config.request_timeout})
  {
    if (!std::isfinite(v) || v <= 0.0) {
      throw std::invalid_argument("crouch planner distances and timeout must be > 0");
    }
  }
}

void CrouchPlanner::standRefused(const Point2 & where)
{
  refused_at_ = where;
}

bool CrouchPlanner::crouchableAhead(
  const Point2 & robot, const std::vector<Point2> & path,
  const std::vector<Point2> & cells) const
{
  if (path.empty() || cells.empty()) {
    return false;
  }
  // The plan starts near the robot, but it is replanned only once a second:
  // start from the point closest to the robot now.
  std::size_t start = 0;
  double best = dist(robot, path[0]);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double d = dist(robot, path[i]);
    if (d < best) {
      best = d;
      start = i;
    }
  }
  double travelled = 0.0;
  for (std::size_t i = start; i < path.size(); ++i) {
    if (i > start) {
      travelled += dist(path[i - 1], path[i]);
      if (travelled > config_.lookahead) {
        break;
      }
    }
    for (const auto & c : cells) {
      if (dist(path[i], c) <= config_.corridor) {
        return true;
      }
    }
  }
  return false;
}

bool CrouchPlanner::crouchableNear(const Point2 & robot, const std::vector<Point2> & cells) const
{
  for (const auto & c : cells) {
    if (dist(robot, c) <= config_.near_radius) {
      return true;
    }
  }
  return false;
}

CrouchDecision CrouchPlanner::update(
  double t, ArmPosture posture, const Point2 & robot, const std::vector<Point2> & path,
  const std::vector<Point2> & crouch_cells, bool obstacle_near)
{
  if (posture == ArmPosture::kBusy) {
    requested_at_.reset();  // posture_manager took the request
    return {CrouchAction::kNone, "arm busy"};
  }
  if (requested_at_ && t - *requested_at_ < config_.request_timeout) {
    return {CrouchAction::kNone, "waiting for posture_manager"};
  }
  requested_at_.reset();
  if (path.empty()) {
    return {CrouchAction::kNone, "not navigating"};
  }
  if (posture == ArmPosture::kNone) {
    return {CrouchAction::kNone, "no posture yet"};
  }

  const bool ahead = crouchableAhead(robot, path, crouch_cells);
  if (posture == ArmPosture::kHigh) {
    if (ahead) {
      requested_at_ = t;
      return {CrouchAction::kCrouch, "low clearance on the path ahead: crouching"};
    }
    return {CrouchAction::kNone, "standing, path clear above"};
  }

  // Crouched.
  if (ahead || crouchableNear(robot, crouch_cells)) {
    return {CrouchAction::kNone, "crouched, low clearance here or ahead"};
  }
  if (obstacle_near) {
    return {CrouchAction::kNone, "crouched, too close to something to move the arm"};
  }
  if (refused_at_ && dist(robot, *refused_at_) < config_.retry_distance) {
    return {CrouchAction::kNone, "crouched, no room above last time I looked"};
  }
  refused_at_.reset();
  requested_at_ = t;
  return {CrouchAction::kStand, "clear around: trying to stand up"};
}

}  // namespace landerpi_web_nav
