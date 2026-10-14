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

#include <cstdint>
#include <optional>

namespace landerpi_web_nav
{

// Who may drive. At most one browser (the "driver") controls the robot;
// everyone else watches. Pure, not thread-safe (the gateway calls it under
// its own mutex).
class DriveArbiter
{
public:
  using ClientId = uint64_t;

  enum class Action
  {
    kNone,     // nothing to do
    kEnable,   // tell velocity_gate ~/set_running(true)
    kDisable,  // tell velocity_gate ~/set_running(false)
  };

  struct Decision
  {
    bool granted = false;  // the request was accepted
    Action action = Action::kNone;
  };

  // A client asks to start (enable = true) or stop driving.
  Decision request(ClientId client, bool enable);
  // A client went away (tab closed, network lost). If it was driving, the
  // robot must stop: returns kDisable.
  Action disconnected(ClientId client);

  bool isDriver(ClientId client) const {return driver_ && *driver_ == client;}
  std::optional<ClientId> driver() const {return driver_;}

private:
  std::optional<ClientId> driver_;
};

}  // namespace landerpi_web_nav
