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

#include <cstddef>
#include <cstdint>
#include <vector>

namespace landerpi_nav2_depth_layer
{

// "Can the robot stand up here?" (stage 6c), from one depth image taken
// with the camera looking up (posture_manager's look-up pose, the robot
// crouched). The camera cannot measure closer than ~0.2 m, so the answer
// is built on positive evidence: enough pixels must read, and almost none
// of them close. Measured (look-up pose, camera at 19 cm): under the bed
// 69 % valid, 98 % of them closer than 0.5 m; open room 60 % valid, none.
struct ClearanceConfig
{
  int stride = 4;                    // pixels: use 1 in stride x stride
  double near_distance = 0.5;        // m: a reading closer than this is "near"
  double min_valid_fraction = 0.3;   // fewer readings than this: unknown
  double max_near_fraction = 0.05;   // of the valid ones: more is blocked
};

enum class Clearance
{
  kClear,    // enough readings, almost none near: standing up is safe
  kBlocked,  // something near above: stay crouched
  kUnknown,  // too few readings to tell: stay crouched
};

struct ClearanceResult
{
  Clearance verdict = Clearance::kUnknown;
  double valid_fraction = 0.0;  // of the sampled pixels
  double near_fraction = 0.0;   // of the valid ones
  std::size_t sampled = 0;
};

// `depth_mm` is row-major, width x height, 0 = no reading. Throws
// std::invalid_argument on a size mismatch or a bad config.
ClearanceResult checkClearance(
  const std::vector<std::uint16_t> & depth_mm, int width, int height,
  const ClearanceConfig & config);

}  // namespace landerpi_nav2_depth_layer
