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

#include "landerpi_web_nav/map_contours.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace landerpi_web_nav
{

std::vector<PointChain> occupiedContours(const GridView & grid, const MapContoursConfig & config)
{
  const auto cells = static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height);
  if (grid.data == nullptr || grid.width <= 0 || grid.height <= 0 || grid.data->size() != cells) {
    throw std::invalid_argument("grid data does not match width * height");
  }
  if (!std::isfinite(grid.resolution) || grid.resolution <= 0.0) {
    throw std::invalid_argument("grid resolution must be > 0");
  }

  // Binary image: 255 = wall. Same row order as the grid (row 0 = lowest y);
  // the image is never displayed, so there is no need to flip it.
  cv::Mat walls(grid.height, grid.width, CV_8UC1, cv::Scalar(0));
  for (int r = 0; r < grid.height; ++r) {
    auto * row = walls.ptr<uint8_t>(r);
    for (int c = 0; c < grid.width; ++c) {
      const int8_t v = (*grid.data)[static_cast<std::size_t>(r) * grid.width + c];
      if (v >= config.occupied_threshold) {
        row[c] = 255;
      }
    }
  }

  if (config.close_radius_cells > 0) {
    const int k = 2 * config.close_radius_cells + 1;
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, k));
    cv::morphologyEx(walls, walls, cv::MORPH_CLOSE, kernel);
  }

  // RETR_LIST: every contour, no hierarchy needed. CHAIN_APPROX_NONE keeps
  // every boundary cell, so the line fit gets all the evidence.
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(walls, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

  std::vector<PointChain> out;
  out.reserve(contours.size());
  for (const auto & contour : contours) {
    PointChain chain;
    chain.closed = true;
    chain.points.reserve(contour.size());
    for (const auto & p : contour) {
      chain.points.push_back(
        {grid.origin_x + (p.x + 0.5) * grid.resolution,
          grid.origin_y + (p.y + 0.5) * grid.resolution});
    }
    out.push_back(std::move(chain));
  }
  return out;
}

}  // namespace landerpi_web_nav
