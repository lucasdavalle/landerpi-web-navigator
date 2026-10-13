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
#include <vector>

#include <opencv2/core.hpp>

#include "landerpi_web_nav/line_extractor.hpp"

namespace landerpi_web_nav
{

// The parts of a nav_msgs/OccupancyGrid needed here, without ROS types.
// Data is row-major starting at the origin cell (row 0 = lowest y), values
// -1 (unknown) or 0..100 (occupancy probability), as in the message.
struct GridView
{
  int width = 0;
  int height = 0;
  double resolution = 0.05;  // m per cell
  double origin_x = 0.0;     // world position of cell (0, 0)'s corner
  double origin_y = 0.0;
  const std::vector<int8_t> * data = nullptr;
};

enum class WallTracing
{
  // findContours: follows the boundary of each wall blob. A wall several
  // cells thick yields one line per face.
  kContours,
  // Zhang-Suen thinning to a 1-cell-wide skeleton, then traced into
  // polylines: one line along the middle of each wall, however thick.
  kSkeleton,
  // Boundary of the free-space region the robot is in, kept only where it
  // touches a wall: the inner face of the room's walls, in order, with gaps
  // where the boundary meets unknown space (open doors, unexplored areas).
  // Islands inside the room (objects on the floor) are ignored.
  kFreeSpace,
};

struct MapContoursConfig
{
  WallTracing method = WallTracing::kFreeSpace;
  // Cells at or above this occupancy count as walls (map_saver uses 65).
  int occupied_threshold = 65;
  // kFreeSpace: cells in [0, free_threshold) are free (map_saver uses 25).
  int free_threshold = 25;
  // kFreeSpace: a boundary cell counts as wall face when an occupied cell
  // lies within this many cells of it.
  int wall_adjacency_cells = 2;
  // kFreeSpace: a point inside the region to trace (the robot position, in
  // world coordinates). Without it, the largest free region is used.
  std::optional<Point2> seed;
  // Morphological closing radius in cells: joins walls broken by a missing
  // cell or two, so a wall comes out as one contour instead of several.
  int close_radius_cells = 1;
};

// Occupied cells -> ordered point chains in world coordinates (cell
// centers), by the configured method. kContours gives closed chains,
// kSkeleton open ones (closed for rings with no ends), kFreeSpace open
// chains along wall faces (one closed chain if the room is fully walled). Throws
// std::invalid_argument on an inconsistent grid (size mismatch,
// non-positive resolution).
std::vector<PointChain> occupiedContours(const GridView & grid, const MapContoursConfig & config);

// Zhang-Suen thinning of a binary image (non-zero = foreground), in place.
// Exposed for tests. Pixels on the image border are treated as background.
void thinZhangSuen(cv::Mat & binary);

// Splits a 1-pixel-wide skeleton into polylines (pixel coordinates),
// breaking at ends and junctions. Exposed for tests.
std::vector<std::vector<cv::Point>> traceSkeleton(const cv::Mat & skeleton);

}  // namespace landerpi_web_nav
