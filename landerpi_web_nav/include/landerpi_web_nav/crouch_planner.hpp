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

#include <optional>
#include <string>
#include <vector>

namespace landerpi_web_nav
{

struct Point2
{
  double x = 0.0;
  double y = 0.0;
};

struct CrouchPlannerConfig
{
  double lookahead = 1.0;        // m of planned path checked ahead of the robot
  double corridor = 0.15;        // m: a crouchable cell this close to the path is "on it"
  double near_radius = 0.5;      // m: crouchable cells this close keep the robot crouched
  double retry_distance = 0.3;   // m to move before trying to stand again after a "no"
  double request_timeout = 3.0;  // s for posture_manager to start acting on a request
};

// What posture_manager reports, reduced to what the planner needs.
enum class ArmPosture
{
  kNone,  // never set: the coordinator leaves it alone
  kHigh,
  kLow,
  kBusy,  // moving, calibrating, looking up, checking
};

ArmPosture parsePosture(const std::string & state);

enum class CrouchAction
{
  kNone,
  kCrouch,  // ask posture_manager for go_low
  kStand,   // ask posture_manager for try_stand
};

struct CrouchDecision
{
  CrouchAction action = CrouchAction::kNone;
  std::string why;  // for /a7/crouch_status and the log
};

// When to crouch and when to stand up during a navigation (stage 6c).
// Pure: positions in the map frame, time in seconds.
// - Standing, and crouchable cells (something only a crouched robot passes
//   under) lie on the next `lookahead` metres of the plan: crouch.
// - Crouched, with no crouchable cell within `near_radius` of the robot nor
//   on the path ahead: try to stand (posture_manager looks up first).
// - After a "no" from that check (blocked or unknown above), do not try
//   again until the robot has moved `retry_distance`.
// It acts only while there is a plan (a navigation in progress): it never
// moves the arm on its own otherwise.
class CrouchPlanner
{
public:
  // Throws std::invalid_argument on non-positive distances or timeout.
  explicit CrouchPlanner(const CrouchPlannerConfig & config);

  CrouchDecision update(
    double t, ArmPosture posture, const Point2 & robot, const std::vector<Point2> & path,
    const std::vector<Point2> & crouch_cells);

  // posture_manager looked up and said no, with the robot at `where`.
  void standRefused(const Point2 & where);

private:
  bool crouchableAhead(
    const Point2 & robot, const std::vector<Point2> & path,
    const std::vector<Point2> & cells) const;
  bool crouchableNear(const Point2 & robot, const std::vector<Point2> & cells) const;

  CrouchPlannerConfig config_;
  std::optional<double> requested_at_;  // a request is in flight
  std::optional<Point2> refused_at_;
};

}  // namespace landerpi_web_nav
