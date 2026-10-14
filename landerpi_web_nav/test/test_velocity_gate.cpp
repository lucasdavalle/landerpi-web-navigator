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

#include <limits>
#include <stdexcept>

#include "landerpi_web_nav/velocity_gate.hpp"

using landerpi_web_nav::GateReason;
using landerpi_web_nav::Velocity;
using landerpi_web_nav::VelocityGate;
using landerpi_web_nav::VelocityGateConfig;

namespace
{

constexpr double kDt = 0.05;  // 20 Hz

// Enabled, alive gate, with the ramp run long enough to reach the target.
VelocityGate runningGate(double & t)
{
  VelocityGate gate{VelocityGateConfig{}};
  gate.setEnabled(true);
  gate.heartbeat(t);
  gate.update(t);
  return gate;
}

// Keeps heartbeat and command fresh for `seconds`, stepping at 20 Hz.
landerpi_web_nav::GateOutput drive(
  VelocityGate & gate, double & t, const Velocity & v, double seconds)
{
  landerpi_web_nav::GateOutput out;
  for (double end = t + seconds; t < end - 1e-9; ) {
    t += kDt;
    gate.heartbeat(t);
    gate.teleop(v, t);
    out = gate.update(t);
  }
  return out;
}

}  // namespace

TEST(VelocityGate, StartsIdleAndOutputsZero)
{
  VelocityGate gate{VelocityGateConfig{}};
  gate.heartbeat(0.0);
  gate.teleop({0.2, 0.0, 0.0}, 0.0);
  const auto out = gate.update(0.0);
  EXPECT_EQ(out.reason, GateReason::kIdle);
  EXPECT_DOUBLE_EQ(out.cmd.vx, 0.0);
}

TEST(VelocityGate, FollowsTeleopWithinTheRamp)
{
  double t = 0.0;
  auto gate = runningGate(t);
  // One 50 ms step at 0.5 m/s^2 allows at most 0.025 m/s.
  t += kDt;
  gate.heartbeat(t);
  gate.teleop({0.2, 0.0, 0.0}, t);
  auto out = gate.update(t);
  EXPECT_EQ(out.reason, GateReason::kTeleop);
  EXPECT_NEAR(out.cmd.vx, 0.025, 1e-9);
  // After a second it has reached the target.
  out = drive(gate, t, {0.2, 0.0, 0.0}, 1.0);
  EXPECT_NEAR(out.cmd.vx, 0.2, 1e-9);
}

TEST(VelocityGate, ClampsToTheLimits)
{
  double t = 0.0;
  auto gate = runningGate(t);
  const auto out = drive(gate, t, {5.0, -5.0, 9.0}, 2.0);
  EXPECT_NEAR(out.cmd.vx, 0.25, 1e-9);
  EXPECT_NEAR(out.cmd.vy, -0.25, 1e-9);
  EXPECT_NEAR(out.cmd.wz, 1.0, 1e-9);
}

TEST(VelocityGate, NonFiniteCommandsBecomeZero)
{
  double t = 0.0;
  auto gate = runningGate(t);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto out = drive(gate, t, {nan, std::numeric_limits<double>::infinity(), 0.0}, 1.0);
  EXPECT_DOUBLE_EQ(out.cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd.vy, 0.0);
}

TEST(VelocityGate, DeadmanStopsAtOnceWhenTheHeartbeatStops)
{
  double t = 0.0;
  auto gate = runningGate(t);
  drive(gate, t, {0.2, 0.0, 0.5}, 2.0);
  // The browser goes away: commands and heartbeats stop arriving.
  t += 0.45;
  auto out = gate.update(t);
  EXPECT_NE(out.reason, GateReason::kNoHeartbeat);  // still within 0.5 s
  t += 0.1;
  out = gate.update(t);
  EXPECT_EQ(out.reason, GateReason::kNoHeartbeat);
  EXPECT_DOUBLE_EQ(out.cmd.vx, 0.0);  // zero at once, no ramp down
  EXPECT_DOUBLE_EQ(out.cmd.wz, 0.0);
}

TEST(VelocityGate, StaleCommandMeansStopEvenWithHeartbeat)
{
  double t = 0.0;
  auto gate = runningGate(t);
  drive(gate, t, {0.2, 0.0, 0.0}, 2.0);
  // Heartbeat keeps coming but the key was released: no more teleop.
  GateReason reason = GateReason::kTeleop;
  for (int i = 0; i < 20; ++i) {
    t += kDt;
    gate.heartbeat(t);
    reason = gate.update(t).reason;
  }
  EXPECT_EQ(reason, GateReason::kNoCommand);
  EXPECT_NEAR(gate.update(t).cmd.vx, 0.0, 1e-9);  // ramped down by now
}

TEST(VelocityGate, TeleopOverridesNav)
{
  double t = 0.0;
  auto gate = runningGate(t);
  t += kDt;
  gate.heartbeat(t);
  gate.nav({0.1, 0.0, 0.0}, t);
  EXPECT_EQ(gate.update(t).reason, GateReason::kNav);
  t += kDt;
  gate.heartbeat(t);
  gate.nav({0.1, 0.0, 0.0}, t);
  gate.teleop({0.0, 0.0, 0.0}, t);  // a person pressing "stop" wins
  EXPECT_EQ(gate.update(t).reason, GateReason::kTeleop);
}

TEST(VelocityGate, DisablingStopsAtOnceAndForgetsCommands)
{
  double t = 0.0;
  auto gate = runningGate(t);
  drive(gate, t, {0.2, 0.0, 0.0}, 2.0);
  gate.setEnabled(false);
  auto out = gate.update(t);
  EXPECT_EQ(out.reason, GateReason::kIdle);
  EXPECT_DOUBLE_EQ(out.cmd.vx, 0.0);
  // Re-enabling does not resume the old command.
  gate.setEnabled(true);
  gate.heartbeat(t);
  out = gate.update(t);
  EXPECT_EQ(out.reason, GateReason::kNoCommand);
  EXPECT_DOUBLE_EQ(out.cmd.vx, 0.0);
}

TEST(VelocityGate, RejectsInvalidConfig)
{
  VelocityGateConfig bad;
  bad.heartbeat_timeout = 0.0;
  EXPECT_THROW(VelocityGate{bad}, std::invalid_argument);
  VelocityGateConfig nan;
  nan.max_vx = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(VelocityGate{nan}, std::invalid_argument);
}
