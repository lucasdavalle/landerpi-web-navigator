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

#include "landerpi_web_nav/velocity_gate.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace landerpi_web_nav
{

namespace
{

void validatePositive(double v, const char * name)
{
  if (!std::isfinite(v) || v <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and > 0");
  }
}

double clampAbs(double v, double limit)
{
  // NaN or infinity from a bad publisher must never reach the wheels.
  if (!std::isfinite(v)) {
    return 0.0;
  }
  return std::clamp(v, -limit, limit);
}

double stepToward(double from, double to, double max_step)
{
  return from + std::clamp(to - from, -max_step, max_step);
}

}  // namespace

std::string toString(GateReason reason)
{
  switch (reason) {
    case GateReason::kIdle: return "idle";
    case GateReason::kNoHeartbeat: return "no_heartbeat";
    case GateReason::kNoCommand: return "no_command";
    case GateReason::kTeleop: return "teleop";
    case GateReason::kNav: return "nav";
  }
  return "unknown";
}

VelocityGate::VelocityGate(const VelocityGateConfig & config)
: config_(config)
{
  validatePositive(config_.max_vx, "max_vx");
  validatePositive(config_.max_vy, "max_vy");
  validatePositive(config_.max_wz, "max_wz");
  validatePositive(config_.max_linear_accel, "max_linear_accel");
  validatePositive(config_.max_angular_accel, "max_angular_accel");
  validatePositive(config_.heartbeat_timeout, "heartbeat_timeout");
  validatePositive(config_.command_timeout, "command_timeout");
}

void VelocityGate::setEnabled(bool enabled)
{
  enabled_ = enabled;
  if (!enabled) {
    // Forget pending commands: re-enabling must not resume an old one.
    teleop_time_.reset();
    nav_time_.reset();
    last_out_ = {};
  }
}

void VelocityGate::heartbeat(double t) {last_heartbeat_ = t;}

void VelocityGate::teleop(const Velocity & v, double t)
{
  teleop_cmd_ = v;
  teleop_time_ = t;
}

void VelocityGate::nav(const Velocity & v, double t)
{
  nav_cmd_ = v;
  nav_time_ = t;
}

GateOutput VelocityGate::update(double t)
{
  const double dt = last_update_ ? std::max(0.0, t - *last_update_) : 0.0;
  last_update_ = t;

  // Stopping is immediate, never ramped.
  if (!enabled_) {
    last_out_ = {};
    return {{}, GateReason::kIdle};
  }
  if (!last_heartbeat_ || t - *last_heartbeat_ > config_.heartbeat_timeout) {
    last_out_ = {};
    return {{}, GateReason::kNoHeartbeat};
  }

  Velocity target;
  GateReason reason = GateReason::kNoCommand;
  auto fresh = [&](const std::optional<double> & when) {
      return when && t - *when <= config_.command_timeout;
    };
  if (fresh(teleop_time_)) {  // the person at the keyboard wins over Nav2
    target = teleop_cmd_;
    reason = GateReason::kTeleop;
  } else if (fresh(nav_time_)) {
    target = nav_cmd_;
    reason = GateReason::kNav;
  }

  target.vx = clampAbs(target.vx, config_.max_vx);
  target.vy = clampAbs(target.vy, config_.max_vy);
  target.wz = clampAbs(target.wz, config_.max_wz);

  // Acceleration limits: move from the last output toward the target by at
  // most accel * dt per axis.
  const double lin = config_.max_linear_accel * dt;
  const double ang = config_.max_angular_accel * dt;
  last_out_.vx = stepToward(last_out_.vx, target.vx, lin);
  last_out_.vy = stepToward(last_out_.vy, target.vy, lin);
  last_out_.wz = stepToward(last_out_.wz, target.wz, ang);
  return {last_out_, reason};
}

}  // namespace landerpi_web_nav
