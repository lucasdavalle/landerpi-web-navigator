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

#include "landerpi_web_nav/drive_arbiter.hpp"

using landerpi_web_nav::DriveArbiter;
using Action = DriveArbiter::Action;

TEST(DriveArbiter, FirstRequestGetsControl)
{
  DriveArbiter a;
  const auto d = a.request(1, true);
  EXPECT_TRUE(d.granted);
  EXPECT_EQ(d.action, Action::kEnable);
  EXPECT_TRUE(a.isDriver(1));
}

TEST(DriveArbiter, OnlyOneDriverAtATime)
{
  DriveArbiter a;
  a.request(1, true);
  const auto d = a.request(2, true);
  EXPECT_FALSE(d.granted);
  EXPECT_EQ(d.action, Action::kNone);
  EXPECT_TRUE(a.isDriver(1));
  EXPECT_FALSE(a.isDriver(2));
}

TEST(DriveArbiter, RepeatedEnableByTheDriverIsHarmless)
{
  DriveArbiter a;
  a.request(1, true);
  const auto d = a.request(1, true);
  EXPECT_TRUE(d.granted);
  EXPECT_EQ(d.action, Action::kNone);
}

TEST(DriveArbiter, OnlyTheDriverCanStop)
{
  DriveArbiter a;
  a.request(1, true);
  EXPECT_FALSE(a.request(2, false).granted);
  EXPECT_TRUE(a.isDriver(1));
  const auto d = a.request(1, false);
  EXPECT_TRUE(d.granted);
  EXPECT_EQ(d.action, Action::kDisable);
  EXPECT_FALSE(a.driver().has_value());
}

TEST(DriveArbiter, DriverDisconnectingStopsTheRobot)
{
  DriveArbiter a;
  a.request(1, true);
  EXPECT_EQ(a.disconnected(2), Action::kNone);  // a viewer leaving: nothing
  EXPECT_EQ(a.disconnected(1), Action::kDisable);
  EXPECT_FALSE(a.driver().has_value());
  // Control is free again.
  EXPECT_TRUE(a.request(2, true).granted);
}
