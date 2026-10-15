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

namespace landerpi_nav2_depth_layer
{

// The rules for one cell of the depth layer's own grid (stage 6c). The
// grid remembers three things: free, obstacle, and "crouchable" (something
// overhead that only a crouched robot passes under). The values match
// nav2_costmap_2d's FREE_SPACE and LETHAL_OBSTACLE; kCrouchCell is this
// layer's own marker and never reaches the master costmap as such.
constexpr std::uint8_t kFreeCell = 0;
constexpr std::uint8_t kLethalCell = 254;
constexpr std::uint8_t kCrouchCell = 128;

enum class Seen
{
  kFloor,       // floor seen in the cell
  kCrouchable,  // something overhead, crouched-passable
  kObstacle,    // something the robot hits in any posture
};

// What a cell becomes after an observation:
// - floor clears an obstacle, but NOT a crouchable mark: under a bed the
//   floor is always in view, and the bed above it is not (the camera looks
//   ahead), so letting the floor clear it would forget the bed overhead;
// - crouchable marks a free cell, never downgrades an obstacle;
// - an obstacle always wins.
std::uint8_t observe(std::uint8_t cell, Seen seen);

// How a cell is used. kByPosture (the local costmap): a crouchable cell
// blocks a standing robot and is free for a crouched one. kFree (the
// global costmap, the user's choice): crouchable costs nothing, so the
// planner goes under freely and the robot crouches on the way.
enum class CrouchableCost
{
  kByPosture,
  kFree,
};

std::uint8_t costOf(std::uint8_t cell, CrouchableCost mode, bool standing);

}  // namespace landerpi_nav2_depth_layer
