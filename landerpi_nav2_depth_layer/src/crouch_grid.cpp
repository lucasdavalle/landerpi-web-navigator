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


#include "landerpi_nav2_depth_layer/crouch_grid.hpp"

#include <cstdint>

namespace landerpi_nav2_depth_layer
{

std::uint8_t observe(std::uint8_t cell, Seen seen)
{
  switch (seen) {
    case Seen::kFloor:
      return cell == kCrouchCell ? kCrouchCell : kFreeCell;
    case Seen::kCrouchable:
      return cell == kLethalCell ? kLethalCell : kCrouchCell;
    case Seen::kObstacle:
      return kLethalCell;
  }
  return cell;
}

std::uint8_t costOf(std::uint8_t cell, CrouchableCost mode, bool standing)
{
  if (cell == kCrouchCell) {
    return mode == CrouchableCost::kByPosture && standing ? kLethalCell : kFreeCell;
  }
  return cell;
}

}  // namespace landerpi_nav2_depth_layer
