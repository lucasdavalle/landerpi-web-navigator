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

#include "landerpi_web_nav/drive_arbiter.hpp"

namespace landerpi_web_nav
{

DriveArbiter::Decision DriveArbiter::request(ClientId client, bool enable)
{
  if (enable) {
    if (driver_ && *driver_ != client) {
      return {false, Action::kNone};  // someone else is driving
    }
    const bool was_driving = driver_.has_value();
    driver_ = client;
    // Enabling twice is harmless, but only the first one needs the call.
    return {true, was_driving ? Action::kNone : Action::kEnable};
  }
  if (!isDriver(client)) {
    return {false, Action::kNone};  // only the driver can hand back control
  }
  driver_.reset();
  return {true, Action::kDisable};
}

DriveArbiter::Action DriveArbiter::disconnected(ClientId client)
{
  if (!isDriver(client)) {
    return Action::kNone;
  }
  driver_.reset();
  return Action::kDisable;
}

}  // namespace landerpi_web_nav
