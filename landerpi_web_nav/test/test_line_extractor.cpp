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

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "landerpi_web_nav/line_extractor.hpp"

using landerpi_web_nav::LineExtractor;
using landerpi_web_nav::LineExtractorConfig;
using landerpi_web_nav::Point2;
using landerpi_web_nav::PointChain;
using landerpi_web_nav::Segment;
using landerpi_web_nav::fitSegment;

namespace
{

constexpr double kStep = 0.05;  // one map cell

// Points every kStep from a to b, b excluded (so polylines do not repeat
// their corners).
void appendLine(std::vector<Point2> & pts, Point2 a, Point2 b)
{
  const double len = std::hypot(b.x - a.x, b.y - a.y);
  const int n = static_cast<int>(std::round(len / kStep));
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / n;
    pts.push_back({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)});
  }
}

// Deterministic "noise" in [-amp, amp], so the test never flakes.
double wobble(int i, double amp) {return amp * std::sin(i * 2.399);}

double angleDeg(const Segment & s)
{
  double a = std::atan2(s.end.y - s.start.y, s.end.x - s.start.x) * 180.0 / M_PI;
  if (a < 0) {
    a += 180.0;
  }
  return a >= 180.0 ? a - 180.0 : a;
}

}  // namespace

TEST(FitSegment, RecoversAHorizontalLine)
{
  std::vector<Point2> pts;
  appendLine(pts, {0.0, 1.0}, {2.0, 1.0});
  pts.push_back({2.0, 1.0});
  const Segment s = fitSegment(pts);
  EXPECT_NEAR(s.start.y, 1.0, 1e-9);
  EXPECT_NEAR(s.end.y, 1.0, 1e-9);
  EXPECT_NEAR(s.length(), 2.0, 1e-9);
  EXPECT_NEAR(s.rms_error, 0.0, 1e-9);
}

TEST(FitSegment, MinimizesPerpendicularNotVerticalDistance)
{
  // A vertical line: an ordinary y-on-x regression would fail here.
  std::vector<Point2> pts;
  appendLine(pts, {0.5, 0.0}, {0.5, 1.0});
  const Segment s = fitSegment(pts);
  EXPECT_NEAR(s.start.x, 0.5, 1e-9);
  EXPECT_NEAR(s.end.x, 0.5, 1e-9);
  EXPECT_NEAR(angleDeg(s), 90.0, 1e-6);
}

TEST(FitSegment, NeedsTwoPoints)
{
  EXPECT_THROW(fitSegment({{0.0, 0.0}}), std::invalid_argument);
}

TEST(LineExtractor, ClosedSquareGivesFourWalls)
{
  PointChain room;
  room.closed = true;
  appendLine(room.points, {0, 0}, {2, 0});
  appendLine(room.points, {2, 0}, {2, 2});
  appendLine(room.points, {2, 2}, {0, 2});
  appendLine(room.points, {0, 2}, {0, 0});
  const auto segs = LineExtractor{LineExtractorConfig{}}.extract({room});
  ASSERT_EQ(segs.size(), 4u);
  for (const auto & s : segs) {
    EXPECT_NEAR(s.length(), 2.0, 0.1);
  }
}

TEST(LineExtractor, LShapeGivesTwoPerpendicularWalls)
{
  PointChain l;
  appendLine(l.points, {0, 1.5}, {0, 0});
  appendLine(l.points, {0, 0}, {2, 0});
  l.points.push_back({2, 0});
  const auto segs = LineExtractor{LineExtractorConfig{}}.extract({l});
  ASSERT_EQ(segs.size(), 2u);
  const double diff = std::abs(angleDeg(segs[0]) - angleDeg(segs[1]));
  EXPECT_NEAR(diff, 90.0, 1.0);
}

TEST(LineExtractor, NoisyWallStaysOneSegment)
{
  PointChain wall;
  appendLine(wall.points, {0, 0}, {3, 0});
  for (std::size_t i = 0; i < wall.points.size(); ++i) {
    wall.points[i].y += wobble(static_cast<int>(i), 0.015);  // +-1.5 cm
  }
  const auto segs = LineExtractor{LineExtractorConfig{}}.extract({wall});
  ASSERT_EQ(segs.size(), 1u);
  EXPECT_NEAR(segs[0].length(), 3.0, 0.1);
  EXPECT_LT(segs[0].rms_error, 0.015);
}

TEST(LineExtractor, DoorwayKeepsTwoWalls)
{
  // Same line, 0.8 m gap (a door): wider than merge_gap, must stay apart.
  PointChain left;
  PointChain right;
  appendLine(left.points, {0, 0}, {1.5, 0});
  appendLine(right.points, {2.3, 0}, {4, 0});
  const auto segs = LineExtractor{LineExtractorConfig{}}.extract({left, right});
  EXPECT_EQ(segs.size(), 2u);
}

TEST(LineExtractor, BrokenWallIsMergedAcrossASmallGap)
{
  // A wall with a 10 cm hole in the map (a missed cell or two).
  PointChain a;
  PointChain b;
  appendLine(a.points, {0, 0}, {1.5, 0});
  appendLine(b.points, {1.6, 0.01}, {3, 0.01});
  const auto segs = LineExtractor{LineExtractorConfig{}}.extract({a, b});
  ASSERT_EQ(segs.size(), 1u);
  EXPECT_NEAR(segs[0].length(), 3.0, 0.1);
}

TEST(LineExtractor, ThickWallContourCollapsesToOneLine)
{
  // findContours walks around a 2-cell-thick wall: out along one face and
  // back along the other, 5 cm apart. That is one wall, not two.
  PointChain contour;
  contour.closed = true;
  appendLine(contour.points, {0, 0}, {2, 0});
  appendLine(contour.points, {2, 0}, {2, 0.05});
  appendLine(contour.points, {2, 0.05}, {0, 0.05});
  appendLine(contour.points, {0, 0.05}, {0, 0});
  const auto segs = LineExtractor{LineExtractorConfig{}}.extract({contour});
  ASSERT_EQ(segs.size(), 1u);
  EXPECT_NEAR(segs[0].start.y, 0.025, 0.02);
  EXPECT_NEAR(segs[0].length(), 2.0, 0.1);
}

TEST(LineExtractor, DropsClutter)
{
  PointChain chair_leg;
  chair_leg.points = {{1.0, 1.0}, {1.05, 1.0}, {1.05, 1.05}};
  EXPECT_TRUE(LineExtractor{LineExtractorConfig{}}.extract({chair_leg}).empty());
}

TEST(LineExtractor, EmptyInputGivesNoSegments)
{
  EXPECT_TRUE(LineExtractor{LineExtractorConfig{}}.extract({}).empty());
}

TEST(CloseCorners, ExtendsTwoWallsToTheirCrossing)
{
  // Two walls that stop 10 cm short of the corner at (0, 0).
  std::vector<Segment> segs(2);
  segs[0].start = {0.1, 0.0};
  segs[0].end = {2.0, 0.0};
  segs[1].start = {0.0, 2.0};
  segs[1].end = {0.0, 0.1};
  landerpi_web_nav::closeCorners(segs, 0.52, 0.35);
  EXPECT_NEAR(segs[0].start.x, 0.0, 1e-9);
  EXPECT_NEAR(segs[0].start.y, 0.0, 1e-9);
  EXPECT_NEAR(segs[1].end.x, 0.0, 1e-9);
  EXPECT_NEAR(segs[1].end.y, 0.0, 1e-9);
  // The far ends do not move.
  EXPECT_NEAR(segs[0].end.x, 2.0, 1e-9);
  EXPECT_NEAR(segs[1].start.y, 2.0, 1e-9);
}

TEST(CloseCorners, LeavesShallowAnglesAndFarCrossingsAlone)
{
  // 10 deg apart: not a corner, a slightly bent wall.
  std::vector<Segment> shallow(2);
  shallow[0].start = {0.0, 0.0};
  shallow[0].end = {1.0, 0.0};
  shallow[1].start = {1.1, 0.0};
  shallow[1].end = {2.1, std::tan(10.0 * M_PI / 180.0)};
  const auto before = shallow;
  landerpi_web_nav::closeCorners(shallow, 0.52, 0.35);
  EXPECT_DOUBLE_EQ(shallow[0].end.x, before[0].end.x);
  EXPECT_DOUBLE_EQ(shallow[1].start.x, before[1].start.x);

  // Perpendicular but 1 m away from the crossing: a door or another room.
  std::vector<Segment> far(2);
  far[0].start = {1.0, 0.0};
  far[0].end = {3.0, 0.0};
  far[1].start = {0.0, 1.0};
  far[1].end = {0.0, 3.0};
  landerpi_web_nav::closeCorners(far, 0.52, 0.35);
  EXPECT_DOUBLE_EQ(far[0].start.x, 1.0);
  EXPECT_DOUBLE_EQ(far[1].start.y, 1.0);
}

TEST(LineExtractor, RejectsInvalidConfig)
{
  LineExtractorConfig bad;
  bad.split_distance = 0.0;
  EXPECT_THROW(LineExtractor{bad}, std::invalid_argument);
  LineExtractorConfig bad_points;
  bad_points.min_points = 1;
  EXPECT_THROW(LineExtractor{bad_points}, std::invalid_argument);
}
