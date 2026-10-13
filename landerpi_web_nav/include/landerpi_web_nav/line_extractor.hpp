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
#include <vector>

namespace landerpi_web_nav
{

struct Point2
{
  double x = 0.0;
  double y = 0.0;
};

struct Segment
{
  Point2 start;
  Point2 end;
  std::size_t num_points = 0;  // points that support it
  double rms_error = 0.0;      // m, RMS perpendicular distance of those points
  double length() const;
};

// An ordered chain of points (a contour or a scan). Closed chains (contours
// from findContours) wrap around: the last point is adjacent to the first.
struct PointChain
{
  std::vector<Point2> points;
  bool closed = false;
};

struct LineExtractorConfig
{
  // Consecutive points farther apart than this belong to different objects.
  double max_gap = 0.15;  // m
  // Split: a chain is cut at its point farthest from the chord between its
  // ends while that distance exceeds this.
  double split_distance = 0.05;  // m
  // Merge: two segments become one when their directions differ by less
  // than merge_angle, each one's ends lie within merge_distance of the
  // other's line, and the gap between them along the line is below
  // merge_gap. It is global (any pair, not only neighbours) so the two
  // parallel contours of a thick wall collapse into one line.
  double merge_angle = 0.087;     // rad (~5 deg)
  double merge_distance = 0.08;   // m
  double merge_gap = 0.25;        // m
  // Output filter: drops clutter (chair legs, noise).
  std::size_t min_points = 5;
  double min_length = 0.20;  // m
};

// Split-and-merge line extraction (map contours -> wall segments). Pure: no
// ROS, no OpenCV.
class LineExtractor
{
public:
  // Throws std::invalid_argument on non-positive or non-finite thresholds.
  explicit LineExtractor(const LineExtractorConfig & config);

  std::vector<Segment> extract(const std::vector<PointChain> & chains) const;

  const LineExtractorConfig & config() const {return config_;}

private:
  LineExtractorConfig config_;
};

// Total least squares fit (principal direction of the points), with the
// segment ends at the projections of the extreme points. Exposed for tests.
// Needs at least 2 points; throws std::invalid_argument otherwise.
Segment fitSegment(const std::vector<Point2> & points);

}  // namespace landerpi_web_nav
