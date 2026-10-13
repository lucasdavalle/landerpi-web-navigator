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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "landerpi_web_nav/line_extractor.hpp"
#include "landerpi_web_nav/map_contours.hpp"

using landerpi_web_nav::GridView;
using landerpi_web_nav::LineExtractor;
using landerpi_web_nav::LineExtractorConfig;
using landerpi_web_nav::MapContoursConfig;
using landerpi_web_nav::Segment;
using landerpi_web_nav::WallTracing;
using landerpi_web_nav::occupiedContours;
using landerpi_web_nav::thinZhangSuen;
using landerpi_web_nav::traceSkeleton;

namespace
{

double angleDeg(const Segment & s)
{
  double a = std::atan2(s.end.y - s.start.y, s.end.x - s.start.x) * 180.0 / M_PI;
  if (a < 0) {
    a += 180.0;
  }
  return a >= 180.0 ? a - 180.0 : a;
}

MapContoursConfig contoursMethod()
{
  MapContoursConfig c;
  c.method = WallTracing::kContours;
  return c;
}

// map_saver "trinary" PGM -> OccupancyGrid data: black (0) = 100,
// light (254) = 0, gray (205) = -1. The PGM's first row is the top of the
// map (highest y); the grid's row 0 is the lowest y, hence the flip.
std::vector<int8_t> loadTrinaryPgm(const std::string & path, int & width, int & height)
{
  const cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
  if (img.empty()) {
    throw std::runtime_error("cannot read " + path);
  }
  width = img.cols;
  height = img.rows;
  std::vector<int8_t> data(static_cast<std::size_t>(width) * height);
  for (int r = 0; r < height; ++r) {
    for (int c = 0; c < width; ++c) {
      const uint8_t v = img.at<uint8_t>(height - 1 - r, c);
      data[static_cast<std::size_t>(r) * width + c] = v < 50 ? 100 : (v > 250 ? 0 : -1);
    }
  }
  return data;
}

}  // namespace

TEST(MapContours, SyntheticRoomGivesFourWalls)
{
  // 40x40 cells (2 m), 1-cell-thick walls around the border.
  const int n = 40;
  std::vector<int8_t> data(n * n, 0);
  for (int i = 0; i < n; ++i) {
    data[i] = 100;                  // bottom row
    data[(n - 1) * n + i] = 100;    // top row
    data[i * n] = 100;              // left column
    data[i * n + (n - 1)] = 100;    // right column
  }
  const GridView grid{n, n, 0.05, -1.0, -1.0, &data};
  for (const auto & cfg : {MapContoursConfig{}, contoursMethod()}) {
    const auto contours = occupiedContours(grid, cfg);
    ASSERT_FALSE(contours.empty());
    const auto segs = LineExtractor{LineExtractorConfig{}}.extract(contours);
    ASSERT_EQ(segs.size(), 4u);
    for (const auto & s : segs) {
      EXPECT_NEAR(s.length(), 1.95, 0.1);  // cell centers: (n - 1) * 0.05
    }
  }
}

TEST(ZhangSuen, ThinsAThickBarToOnePixel)
{
  cv::Mat img(20, 40, CV_8UC1, cv::Scalar(0));
  img(cv::Rect(5, 6, 30, 5)).setTo(255);  // 5 px thick, rows 6..10
  thinZhangSuen(img);
  // Every column inside the bar (away from its ends) keeps exactly 1 pixel,
  // in the middle rows.
  for (int c = 10; c < 30; ++c) {
    int count = 0;
    int row = -1;
    for (int r = 0; r < img.rows; ++r) {
      if (img.at<uint8_t>(r, c)) {
        ++count;
        row = r;
      }
    }
    EXPECT_EQ(count, 1) << "column " << c;
    EXPECT_NEAR(row, 8, 1) << "column " << c;
  }
}

TEST(ZhangSuen, KeepsAOnePixelLineIntact)
{
  cv::Mat img(10, 30, CV_8UC1, cv::Scalar(0));
  img(cv::Rect(2, 4, 25, 1)).setTo(255);
  const cv::Mat before = img.clone();
  thinZhangSuen(img);
  EXPECT_EQ(cv::countNonZero(img != before), 0);
}

TEST(TraceSkeleton, SplitsATeeIntoThreeBranches)
{
  cv::Mat img(30, 30, CV_8UC1, cv::Scalar(0));
  img(cv::Rect(2, 10, 26, 1)).setTo(255);   // horizontal bar
  img(cv::Rect(15, 11, 1, 15)).setTo(255);  // stem going down from it
  const auto paths = traceSkeleton(img);
  int long_paths = 0;
  for (const auto & p : paths) {
    long_paths += p.size() >= 10;
  }
  EXPECT_EQ(long_paths, 3);
}

TEST(MapContours, ThickWallGivesOneCentredLine)
{
  // A 4-cell-thick wall (rows 8..11): contours see two faces 15 cm apart,
  // the skeleton sees one line through the middle.
  const int w = 60;
  const int h = 20;
  std::vector<int8_t> data(w * h, 0);
  for (int r = 8; r < 12; ++r) {
    for (int c = 5; c < 55; ++c) {
      data[r * w + c] = 100;
    }
  }
  const GridView grid{w, h, 0.05, 0.0, 0.0, &data};
  const LineExtractor extractor{LineExtractorConfig{}};

  const auto skel = extractor.extract(occupiedContours(grid, MapContoursConfig{}));
  ASSERT_EQ(skel.size(), 1u);
  EXPECT_NEAR(skel[0].start.y, 10.0 * 0.05, 0.05);  // centre of rows 8..11
  EXPECT_NEAR(skel[0].length(), 2.4, 0.2);

  const auto faces = extractor.extract(occupiedContours(grid, contoursMethod()));
  EXPECT_EQ(faces.size(), 2u);  // the defect the skeleton fixes
}

TEST(MapContours, CellCentersMapToWorldCoordinates)
{
  // A single horizontal wall in row 10 of a grid whose origin is (1, 2).
  const int w = 30;
  const int h = 20;
  std::vector<int8_t> data(w * h, 0);
  for (int c = 5; c < 25; ++c) {
    data[10 * w + c] = 100;
  }
  const GridView grid{w, h, 0.05, 1.0, 2.0, &data};
  const auto segs = LineExtractor{LineExtractorConfig{}}.extract(
    occupiedContours(grid, MapContoursConfig{}));
  ASSERT_EQ(segs.size(), 1u);
  EXPECT_NEAR(segs[0].start.y, 2.0 + 10.5 * 0.05, 1e-6);
  EXPECT_NEAR(std::min(segs[0].start.x, segs[0].end.x), 1.0 + 5.5 * 0.05, 1e-6);
}

TEST(MapContours, UnknownAndFreeCellsAreNotWalls)
{
  std::vector<int8_t> data(100, -1);
  data[0] = 0;
  data[1] = 64;  // just below the threshold
  const GridView grid{10, 10, 0.05, 0.0, 0.0, &data};
  EXPECT_TRUE(occupiedContours(grid, MapContoursConfig{}).empty());
}

TEST(MapContours, RejectsInconsistentGrid)
{
  std::vector<int8_t> data(10, 0);
  EXPECT_THROW(
    occupiedContours(GridView{5, 5, 0.05, 0.0, 0.0, &data}, MapContoursConfig{}),
    std::invalid_argument);
  EXPECT_THROW(
    occupiedContours(GridView{2, 5, 0.0, 0.0, 0.0, &data}, MapContoursConfig{}),
    std::invalid_argument);
}

TEST(MapContours, RealRoomFromStageZero)
{
  // The room mapped on the robot in stage 0 (~4.5 x 4.4 m).
  int w = 0;
  int h = 0;
  const auto data = loadTrinaryPgm(std::string(TEST_DATA_DIR) + "/stage0_room.pgm", w, h);
  const GridView grid{w, h, 0.05, -3.62, -4.34, &data};
  const LineExtractor extractor{LineExtractorConfig{}};
  const auto segs = extractor.extract(occupiedContours(grid, MapContoursConfig{}));
  const auto face_segs = extractor.extract(occupiedContours(grid, contoursMethod()));
  // The skeleton never gives more segments than tracing both faces.
  EXPECT_LE(segs.size(), face_segs.size());

  std::vector<Segment> walls;
  for (const auto & s : segs) {
    if (s.length() > 1.5) {
      walls.push_back(s);
    }
  }
  // The four walls of the room show up as long segments...
  EXPECT_GE(walls.size(), 4u);
  // ...and they are mutually parallel or perpendicular: every long segment
  // is within 6 deg of the longest one or of its perpendicular.
  const auto longest = *std::max_element(
    walls.begin(), walls.end(), [](const Segment & a, const Segment & b) {
      return a.length() < b.length();
    });
  for (const auto & s : walls) {
    double d = std::fmod(std::abs(angleDeg(s) - angleDeg(longest)), 90.0);
    d = std::min(d, 90.0 - d);
    EXPECT_LT(d, 6.0) << "segment at " << angleDeg(s) << " deg, length " << s.length();
  }

  // Optional visual check: A7_DEBUG_DIR=/some/dir writes the map with the
  // segments drawn on it (not part of the pass/fail criteria).
  if (const char * dir = std::getenv("A7_DEBUG_DIR")) {
    auto draw = [&](const std::vector<Segment> & ss, const std::string & name) {
        cv::Mat img = cv::imread(
          std::string(TEST_DATA_DIR) + "/stage0_room.pgm", cv::IMREAD_COLOR);
        cv::resize(img, img, cv::Size(), 4, 4, cv::INTER_NEAREST);
        auto px = [&](const landerpi_web_nav::Point2 & p) {
            return cv::Point(
              static_cast<int>((p.x + 3.62) / 0.05 * 4),
              static_cast<int>((h - (p.y + 4.34) / 0.05) * 4));
          };
        for (const auto & s : ss) {
          cv::line(img, px(s.start), px(s.end), cv::Scalar(0, 0, 255), 2);
        }
        cv::imwrite(std::string(dir) + "/" + name, img);
      };
    for (const auto & s : segs) {
      if (s.length() > 0.8) {
        std::printf(
          "[debug] segment %.2f m at %.1f deg, from (%.2f, %.2f) to (%.2f, %.2f)\n",
          s.length(), angleDeg(s), s.start.x, s.start.y, s.end.x, s.end.y);
      }
    }
    draw(segs, "stage0_room_skeleton.png");
    draw(face_segs, "stage0_room_contours.png");
  }
}
