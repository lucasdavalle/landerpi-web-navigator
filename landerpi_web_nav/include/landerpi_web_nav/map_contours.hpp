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
#include <vector>

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

struct MapContoursConfig
{
  // Cells at or above this occupancy count as walls (map_saver uses 65).
  int occupied_threshold = 65;
  // Morphological closing radius in cells: joins walls broken by a missing
  // cell or two, so a wall comes out as one contour instead of several.
  int close_radius_cells = 1;
};

// Occupied cells -> ordered, closed contours in world coordinates (cell
// centers), via OpenCV's findContours. Throws std::invalid_argument on an
// inconsistent grid (size mismatch, non-positive resolution).
std::vector<PointChain> occupiedContours(const GridView & grid, const MapContoursConfig & config);

}  // namespace landerpi_web_nav
