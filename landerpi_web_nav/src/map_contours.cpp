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

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace landerpi_web_nav
{

namespace
{

// 8-neighbourhood in Zhang-Suen order: P2 (north) clockwise to P9 (north-west),
// as {row offset, column offset}.
constexpr std::array<std::array<int, 2>, 8> kRing{{
  {-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}}};

bool on(const cv::Mat & img, int r, int c)
{
  return r >= 0 && c >= 0 && r < img.rows && c < img.cols && img.at<uint8_t>(r, c) != 0;
}

std::vector<cv::Point> neighbours(const cv::Mat & img, const cv::Point & p)
{
  std::vector<cv::Point> out;
  for (const auto & d : kRing) {
    if (on(img, p.y + d[0], p.x + d[1])) {
      out.emplace_back(p.x + d[1], p.y + d[0]);
    }
  }
  return out;
}

PointChain toWorld(const std::vector<cv::Point> & pixels, const GridView & grid, bool closed)
{
  PointChain chain;
  chain.closed = closed;
  chain.points.reserve(pixels.size());
  for (const auto & p : pixels) {
    chain.points.push_back(
      {grid.origin_x + (p.x + 0.5) * grid.resolution,
        grid.origin_y + (p.y + 0.5) * grid.resolution});
  }
  return chain;
}

}  // namespace

void thinZhangSuen(cv::Mat & binary)
{
  // Pad by one pixel so walls touching the image edge are thinned like any
  // other (the neighbourhood test never reads outside the image).
  cv::Mat img;
  cv::copyMakeBorder(binary, img, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
  img = img != 0;  // 0 / 255

  std::vector<cv::Point> to_delete;
  bool changed = true;
  while (changed) {
    changed = false;
    for (int step = 0; step < 2; ++step) {
      to_delete.clear();
      for (int r = 1; r < img.rows - 1; ++r) {
        for (int c = 1; c < img.cols - 1; ++c) {
          if (img.at<uint8_t>(r, c) == 0) {
            continue;
          }
          std::array<int, 8> p{};
          int b = 0;  // foreground neighbours
          for (std::size_t i = 0; i < 8; ++i) {
            p[i] = img.at<uint8_t>(r + kRing[i][0], c + kRing[i][1]) != 0;
            b += p[i];
          }
          int a = 0;  // 0 -> 1 transitions going around the ring
          for (std::size_t i = 0; i < 8; ++i) {
            a += (p[i] == 0 && p[(i + 1) % 8] == 1);
          }
          // p[0]=P2 (N), p[2]=P4 (E), p[4]=P6 (S), p[6]=P8 (W).
          const bool cond = step == 0 ?
            (p[0] * p[2] * p[4] == 0 && p[2] * p[4] * p[6] == 0) :
            (p[0] * p[2] * p[6] == 0 && p[0] * p[4] * p[6] == 0);
          if (b >= 2 && b <= 6 && a == 1 && cond) {
            to_delete.emplace_back(c, r);
          }
        }
      }
      // Deleting after the scan (not during it) is what makes each
      // sub-iteration symmetric: every decision sees the same image.
      for (const auto & q : to_delete) {
        img.at<uint8_t>(q.y, q.x) = 0;
      }
      changed = changed || !to_delete.empty();
    }
  }
  binary = img(cv::Rect(1, 1, binary.cols, binary.rows)).clone();
}

std::vector<std::vector<cv::Point>> traceSkeleton(const cv::Mat & skeleton)
{
  // Nodes are ends (1 neighbour) and junctions (3+). Every path between two
  // nodes becomes one polyline; whatever is left afterwards are rings with
  // no node at all, traced on their own.
  cv::Mat visited(skeleton.size(), CV_8UC1, cv::Scalar(0));
  auto is_node = [&](const cv::Point & p) {return neighbours(skeleton, p).size() != 2;};

  auto walk = [&](cv::Point prev, cv::Point cur, std::vector<cv::Point> path) {
      while (true) {
        path.push_back(cur);
        if (is_node(cur)) {
          break;
        }
        visited.at<uint8_t>(cur.y, cur.x) = 1;
        cv::Point next{-1, -1};
        for (const auto & n : neighbours(skeleton, cur)) {
          if (n != prev && (is_node(n) || visited.at<uint8_t>(n.y, n.x) == 0)) {
            next = n;
            break;
          }
        }
        if (next.x < 0) {
          break;  // ring closed: the only way on is already visited
        }
        prev = cur;
        cur = next;
      }
      return path;
    };

  std::vector<std::vector<cv::Point>> paths;
  for (int r = 0; r < skeleton.rows; ++r) {
    for (int c = 0; c < skeleton.cols; ++c) {
      const cv::Point p{c, r};
      if (skeleton.at<uint8_t>(r, c) == 0 || !is_node(p)) {
        continue;
      }
      for (const auto & n : neighbours(skeleton, p)) {
        // Node-to-node steps are 2-pixel stubs inside junctions: skipped.
        if (!is_node(n) && visited.at<uint8_t>(n.y, n.x) == 0) {
          paths.push_back(walk(p, n, {p}));
        }
      }
    }
  }
  for (int r = 0; r < skeleton.rows; ++r) {
    for (int c = 0; c < skeleton.cols; ++c) {
      if (skeleton.at<uint8_t>(r, c) != 0 && visited.at<uint8_t>(r, c) == 0) {
        const cv::Point start{c, r};
        if (is_node(start)) {
          continue;  // isolated pixel or junction already handled
        }
        auto ring = walk(neighbours(skeleton, start).front(), start, {});
        ring.push_back(start);  // close it explicitly
        paths.push_back(std::move(ring));
      }
    }
  }
  return paths;
}

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

  std::vector<PointChain> out;
  if (config.method == WallTracing::kSkeleton) {
    thinZhangSuen(walls);
    for (const auto & path : traceSkeleton(walls)) {
      // A path that starts and ends on the same pixel is a ring.
      const bool closed = path.size() > 2 && path.front() == path.back();
      out.push_back(toWorld(path, grid, closed));
    }
    return out;
  }

  // RETR_LIST: every contour, no hierarchy needed. CHAIN_APPROX_NONE keeps
  // every boundary cell, so the line fit gets all the evidence.
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(walls, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
  out.reserve(contours.size());
  for (const auto & contour : contours) {
    out.push_back(toWorld(contour, grid, true));
  }
  return out;
}

}  // namespace landerpi_web_nav
