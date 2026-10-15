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

#include <stdexcept>
#include <vector>

#include "landerpi_web_nav/crouch_planner.hpp"

using landerpi_web_nav::ArmPosture;
using landerpi_web_nav::CrouchAction;
using landerpi_web_nav::CrouchPlanner;
using landerpi_web_nav::CrouchPlannerConfig;
using landerpi_web_nav::Point2;
using landerpi_web_nav::parsePosture;

namespace
{

// A bed edge across the room: crouchable cells along x = 1.0 .. 1.1.
std::vector<Point2> bedEdge()
{
  std::vector<Point2> cells;
  for (double y = -0.5; y <= 0.5; y += 0.05) {
    cells.push_back({1.0, y});
    cells.push_back({1.05, y});
    cells.push_back({1.1, y});
  }
  return cells;
}

// A straight plan along x, one point every 5 cm.
std::vector<Point2> straightPath(double from, double to)
{
  std::vector<Point2> path;
  for (double x = from; x <= to + 1e-9; x += 0.05) {
    path.push_back({x, 0.0});
  }
  return path;
}

}  // namespace

TEST(CrouchPlanner, ParsesPostureStates)
{
  EXPECT_EQ(parsePosture("high"), ArmPosture::kHigh);
  EXPECT_EQ(parsePosture("low"), ArmPosture::kLow);
  EXPECT_EQ(parsePosture("none"), ArmPosture::kNone);
  EXPECT_EQ(parsePosture("looking_up"), ArmPosture::kBusy);
  EXPECT_EQ(parsePosture("moving"), ArmPosture::kBusy);
}

TEST(CrouchPlanner, CrouchesWhenTheBedIsOnThePathAhead)
{
  CrouchPlanner p(CrouchPlannerConfig{});
  const auto d = p.update(0.0, ArmPosture::kHigh, {0.3, 0.0}, straightPath(0.3, 2.0), bedEdge());
  EXPECT_EQ(d.action, CrouchAction::kCrouch);
}

TEST(CrouchPlanner, IgnoresTheBedBeyondTheLookahead)
{
  CrouchPlanner p(CrouchPlannerConfig{});
  // 1.5 m before the edge: the next metre of plan does not reach it yet.
  EXPECT_EQ(
    p.update(0.0, ArmPosture::kHigh, {-0.5, 0.0}, straightPath(-0.5, 2.0), bedEdge()).action,
    CrouchAction::kNone);
}

TEST(CrouchPlanner, IgnoresABedBesideThePath)
{
  CrouchPlanner p(CrouchPlannerConfig{});
  std::vector<Point2> path;
  for (double x = 0.3; x <= 2.0; x += 0.05) {
    path.push_back({x, 0.8});  // passes 30 cm beside the bed's end (y = 0.5)
  }
  EXPECT_EQ(
    p.update(0.0, ArmPosture::kHigh, {0.3, 0.8}, path, bedEdge()).action, CrouchAction::kNone);
}

TEST(CrouchPlanner, WaitsForPostureManagerAndRetriesAfterTheTimeout)
{
  CrouchPlanner p(CrouchPlannerConfig{});
  const auto path = straightPath(0.3, 2.0);
  EXPECT_EQ(
    p.update(0.0, ArmPosture::kHigh, {0.3, 0}, path, bedEdge()).action,
    CrouchAction::kCrouch);
  // Not acting yet (the request is in flight): no second request.
  EXPECT_EQ(
    p.update(1.0, ArmPosture::kHigh, {0.3, 0}, path, bedEdge()).action,
    CrouchAction::kNone);
  // Nothing happened for longer than the timeout: ask again.
  EXPECT_EQ(
    p.update(3.5, ArmPosture::kHigh, {0.3, 0}, path, bedEdge()).action,
    CrouchAction::kCrouch);
  // posture_manager starts moving: nothing while busy.
  EXPECT_EQ(
    p.update(4.0, ArmPosture::kBusy, {0.3, 0}, path, bedEdge()).action,
    CrouchAction::kNone);
}

// Demo 1: the goal is under the bed. Crouched there, it stays crouched.
TEST(CrouchPlanner, StaysCrouchedUnderTheBed)
{
  CrouchPlanner p(CrouchPlannerConfig{});
  EXPECT_EQ(
    p.update(0.0, ArmPosture::kLow, {1.05, 0.0}, straightPath(1.05, 1.3), bedEdge()).action,
    CrouchAction::kNone);
}

// Demo 2: across the bed. Past the edge and clear of it, it stands up.
TEST(CrouchPlanner, StandsUpOnceClearOfTheBed)
{
  CrouchPlanner p(CrouchPlannerConfig{});
  // Just past the edge: still within near_radius of it.
  EXPECT_EQ(
    p.update(0.0, ArmPosture::kLow, {1.4, 0.0}, straightPath(1.4, 2.5), bedEdge()).action,
    CrouchAction::kNone);
  // 70 cm past it.
  EXPECT_EQ(
    p.update(1.0, ArmPosture::kLow, {1.8, 0.0}, straightPath(1.8, 2.5), bedEdge()).action,
    CrouchAction::kStand);
}

// A "no" from the look-up check (a bed the camera had not marked): do not
// look up again on the spot, only after moving 30 cm.
TEST(CrouchPlanner, DoesNotRetryStandingInTheSamePlace)
{
  CrouchPlanner p(CrouchPlannerConfig{});
  const std::vector<Point2> none;
  EXPECT_EQ(
    p.update(0.0, ArmPosture::kLow, {1.8, 0}, straightPath(1.8, 3.0), none).action,
    CrouchAction::kStand);
  p.update(0.5, ArmPosture::kBusy, {1.8, 0}, straightPath(1.8, 3.0), none);
  p.standRefused({1.8, 0.0});
  EXPECT_EQ(
    p.update(10.0, ArmPosture::kLow, {1.9, 0}, straightPath(1.9, 3.0), none).action,
    CrouchAction::kNone);
  EXPECT_EQ(
    p.update(12.0, ArmPosture::kLow, {2.2, 0}, straightPath(2.2, 3.0), none).action,
    CrouchAction::kStand);
}

// Activating it, or crouching by hand, never moves the arm by itself.
TEST(CrouchPlanner, DoesNothingWithoutANavigation)
{
  CrouchPlanner p(CrouchPlannerConfig{});
  EXPECT_EQ(p.update(0.0, ArmPosture::kLow, {3.0, 0}, {}, bedEdge()).action, CrouchAction::kNone);
  EXPECT_EQ(
    p.update(0.0, ArmPosture::kHigh, {0.3, 0}, {}, bedEdge()).action,
    CrouchAction::kNone);
  EXPECT_EQ(
    p.update(0.0, ArmPosture::kNone, {0.3, 0}, straightPath(0.3, 2.0), bedEdge()).action,
    CrouchAction::kNone);
}

// First 6c demo: crouched next to a wardrobe door, the look-up gesture
// brushed it. With an obstacle that close it does not move the arm.
TEST(CrouchPlanner, DoesNotStandUpNextToAnObstacle)
{
  CrouchPlanner p(CrouchPlannerConfig{});
  const std::vector<Point2> none;
  EXPECT_EQ(
    p.update(0.0, ArmPosture::kLow, {1.8, 0}, straightPath(1.8, 3.0), none, true).action,
    CrouchAction::kNone);
  EXPECT_EQ(
    p.update(1.0, ArmPosture::kLow, {1.8, 0}, straightPath(1.8, 3.0), none, false).action,
    CrouchAction::kStand);
}

TEST(CrouchPlanner, RejectsBadConfig)
{
  CrouchPlannerConfig bad;
  bad.lookahead = 0.0;
  EXPECT_THROW(CrouchPlanner{bad}, std::invalid_argument);
}
