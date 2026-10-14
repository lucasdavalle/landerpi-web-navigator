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

namespace landerpi_web_nav
{

struct Velocity
{
  double vx = 0.0;  // m/s, forward
  double vy = 0.0;  // m/s, left (mecanum)
  double wz = 0.0;  // rad/s, counter-clockwise
};

struct VelocityGateConfig
{
  double max_vx = 0.25;   // m/s
  double max_vy = 0.25;   // m/s
  double max_wz = 1.0;    // rad/s
  // Acceleration limits for normal driving. Stopping (idle, deadman) is
  // never ramped: it goes to zero at once.
  double max_linear_accel = 0.5;   // m/s^2
  double max_angular_accel = 2.0;  // rad/s^2
  // Deadman: no heartbeat from the gateway for this long -> zero.
  double heartbeat_timeout = 0.5;  // s
  // A command older than this is stale and treated as "stop".
  double command_timeout = 0.3;    // s
};

// Why the gate is outputting what it outputs (shown in the web UI).
enum class GateReason
{
  kIdle,         // not enabled: always zero
  kNoHeartbeat,  // enabled, but the deadman tripped
  kNoCommand,    // enabled and alive, nobody is commanding
  kTeleop,       // following the web teleop
  kNav,          // following Nav2 (stage 5)
};

std::string toString(GateReason reason);

struct GateOutput
{
  Velocity cmd;
  GateReason reason = GateReason::kIdle;
};

// The only door to the wheels: arbitration (teleop over Nav2), clamping,
// acceleration limits and a deadman. Pure: time comes in as seconds, so
// tests drive the clock by hand. Starts idle (zero) until enabled.
class VelocityGate
{
public:
  // Throws std::invalid_argument on non-positive or non-finite limits.
  explicit VelocityGate(const VelocityGateConfig & config);

  void setEnabled(bool enabled);
  bool enabled() const {return enabled_;}

  void heartbeat(double t);
  void teleop(const Velocity & v, double t);
  void nav(const Velocity & v, double t);

  // Called at the publishing rate. Returns the command to publish now.
  GateOutput update(double t);

private:
  VelocityGateConfig config_;
  bool enabled_ = false;
  std::optional<double> last_heartbeat_;
  std::optional<double> teleop_time_;
  std::optional<double> nav_time_;
  Velocity teleop_cmd_;
  Velocity nav_cmd_;
  Velocity last_out_;
  std::optional<double> last_update_;
};

}  // namespace landerpi_web_nav
