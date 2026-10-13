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

#include "landerpi_web_nav/line_extractor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace landerpi_web_nav
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double distance(const Point2 & a, const Point2 & b) {return std::hypot(b.x - a.x, b.y - a.y);}

// Distance from p to the infinite line through a and b (or to a if a == b).
double distanceToLine(const Point2 & p, const Point2 & a, const Point2 & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len = std::hypot(dx, dy);
  if (len < 1e-12) {
    return distance(p, a);
  }
  return std::abs(dy * (p.x - a.x) - dx * (p.y - a.y)) / len;
}

// A fitted line plus the points it came from (kept so merges can refit).
struct Candidate
{
  Segment segment;
  std::vector<Point2> points;
  double angle = 0.0;  // direction in [0, pi): lines have no orientation
};

Candidate makeCandidate(std::vector<Point2> points)
{
  Candidate c;
  c.segment = fitSegment(points);
  c.points = std::move(points);
  double a = std::atan2(c.segment.end.y - c.segment.start.y, c.segment.end.x - c.segment.start.x);
  if (a < 0.0) {
    a += kPi;
  }
  c.angle = a >= kPi ? a - kPi : a;
  return c;
}

void validatePositive(double v, const std::string & name)
{
  if (!std::isfinite(v) || v <= 0.0) {
    throw std::invalid_argument(name + " must be finite and > 0");
  }
}

// Step 1: cut a chain wherever two consecutive points are farther apart
// than max_gap. A closed chain is first opened at its point farthest from
// the first one, so split-and-merge (which needs two ends) can run on it.
std::vector<std::vector<Point2>> breakAtGaps(const PointChain & chain, double max_gap)
{
  std::vector<Point2> pts = chain.points;
  if (chain.closed && pts.size() > 2) {
    std::size_t far = 0;
    double best = -1.0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
      const double d = distance(pts[0], pts[i]);
      if (d > best) {
        best = d;
        far = i;
      }
    }
    // [0..far] and [far..end, 0]: two open halves sharing their ends.
    std::vector<Point2> second(pts.begin() + static_cast<std::ptrdiff_t>(far), pts.end());
    second.push_back(pts[0]);
    pts.resize(far + 1);
    PointChain a{pts, false};
    PointChain b{second, false};
    auto out = breakAtGaps(a, max_gap);
    auto more = breakAtGaps(b, max_gap);
    out.insert(out.end(), more.begin(), more.end());
    return out;
  }

  std::vector<std::vector<Point2>> out;
  std::vector<Point2> current;
  for (const auto & p : pts) {
    if (!current.empty() && distance(current.back(), p) > max_gap) {
      out.push_back(std::move(current));
      current.clear();
    }
    current.push_back(p);
  }
  if (!current.empty()) {
    out.push_back(std::move(current));
  }
  return out;
}

// Step 2: iterative split (an explicit stack instead of recursion, so a
// long contour cannot overflow the call stack).
std::vector<std::vector<Point2>> split(const std::vector<Point2> & pts, double split_distance)
{
  std::vector<std::vector<Point2>> out;
  std::vector<std::pair<std::size_t, std::size_t>> stack{{0, pts.size() - 1}};
  while (!stack.empty()) {
    const auto [first, last] = stack.back();
    stack.pop_back();
    std::size_t worst = first;
    double worst_d = 0.0;
    for (std::size_t i = first + 1; i < last; ++i) {
      const double d = distanceToLine(pts[i], pts[first], pts[last]);
      if (d > worst_d) {
        worst_d = d;
        worst = i;
      }
    }
    if (worst_d > split_distance && worst > first && worst < last) {
      // Process the left half first: push the right one underneath.
      stack.emplace_back(worst, last);
      stack.emplace_back(first, worst);
    } else {
      out.emplace_back(
        pts.begin() + static_cast<std::ptrdiff_t>(first),
        pts.begin() + static_cast<std::ptrdiff_t>(last) + 1);
    }
  }
  return out;
}

// Gap between two segments along a's direction: negative when they overlap.
double gapAlong(const Candidate & a, const Candidate & b)
{
  const double dx = a.segment.end.x - a.segment.start.x;
  const double dy = a.segment.end.y - a.segment.start.y;
  const double len = std::hypot(dx, dy);
  const double ux = dx / len;
  const double uy = dy / len;
  auto proj = [&](const Point2 & p) {
      return (p.x - a.segment.start.x) * ux + (p.y - a.segment.start.y) * uy;
    };
  const double a0 = 0.0;
  const double a1 = len;
  const double b0 = std::min(proj(b.segment.start), proj(b.segment.end));
  const double b1 = std::max(proj(b.segment.start), proj(b.segment.end));
  return std::max(b0 - a1, a0 - b1);
}

bool mergeable(const Candidate & a, const Candidate & b, const LineExtractorConfig & cfg)
{
  double dang = std::abs(a.angle - b.angle);
  dang = std::min(dang, kPi - dang);
  if (dang > cfg.merge_angle) {
    return false;
  }
  const auto & sa = a.segment;
  const auto & sb = b.segment;
  if (distanceToLine(sb.start, sa.start, sa.end) > cfg.merge_distance ||
    distanceToLine(sb.end, sa.start, sa.end) > cfg.merge_distance ||
    distanceToLine(sa.start, sb.start, sb.end) > cfg.merge_distance ||
    distanceToLine(sa.end, sb.start, sb.end) > cfg.merge_distance)
  {
    return false;
  }
  return gapAlong(a, b) <= cfg.merge_gap;
}

// Crossing point of the infinite lines through two segments, or nullopt if
// they are (nearly) parallel.
std::optional<Point2> lineCrossing(const Segment & a, const Segment & b)
{
  const double ax = a.end.x - a.start.x;
  const double ay = a.end.y - a.start.y;
  const double bx = b.end.x - b.start.x;
  const double by = b.end.y - b.start.y;
  const double den = ax * by - ay * bx;
  if (std::abs(den) < 1e-12) {
    return std::nullopt;
  }
  const double t = ((b.start.x - a.start.x) * by - (b.start.y - a.start.y) * bx) / den;
  return Point2{a.start.x + t * ax, a.start.y + t * ay};
}

double undirectedAngle(const Segment & a, const Segment & b)
{
  const double aa = std::atan2(a.end.y - a.start.y, a.end.x - a.start.x);
  const double ab = std::atan2(b.end.y - b.start.y, b.end.x - b.start.x);
  double d = std::fmod(std::abs(aa - ab), kPi);
  return std::min(d, kPi - d);
}

}  // namespace

void closeCorners(std::vector<Segment> & segments, double min_angle, double max_extension)
{
  if (max_extension <= 0.0) {
    return;
  }
  // Every (segment end, segment end) pair that could form a corner, cheapest
  // first, applied greedily so each end moves at most once.
  struct Move
  {
    double cost;
    std::size_t i;
    int end_i;  // 0 = start, 1 = end
    std::size_t j;
    int end_j;
    Point2 corner;
  };
  auto end_of = [&](std::size_t k, int e) -> Point2 & {
      return e == 0 ? segments[k].start : segments[k].end;
    };

  std::vector<Move> moves;
  for (std::size_t i = 0; i < segments.size(); ++i) {
    for (std::size_t j = i + 1; j < segments.size(); ++j) {
      if (undirectedAngle(segments[i], segments[j]) < min_angle) {
        continue;
      }
      const auto corner = lineCrossing(segments[i], segments[j]);
      if (!corner) {
        continue;
      }
      for (int ei = 0; ei < 2; ++ei) {
        for (int ej = 0; ej < 2; ++ej) {
          const double di = distance(end_of(i, ei), *corner);
          const double dj = distance(end_of(j, ej), *corner);
          if (di <= max_extension && dj <= max_extension) {
            moves.push_back({di + dj, i, ei, j, ej, *corner});
          }
        }
      }
    }
  }
  std::sort(
    moves.begin(), moves.end(), [](const Move & a, const Move & b) {
      return a.cost < b.cost;
    });
  std::vector<std::array<bool, 2>> used(segments.size(), {false, false});
  for (const auto & m : moves) {
    if (used[m.i][m.end_i] || used[m.j][m.end_j]) {
      continue;
    }
    end_of(m.i, m.end_i) = m.corner;
    end_of(m.j, m.end_j) = m.corner;
    used[m.i][m.end_i] = true;
    used[m.j][m.end_j] = true;
  }
}

double Segment::length() const {return distance(start, end);}

Segment fitSegment(const std::vector<Point2> & points)
{
  if (points.size() < 2) {
    throw std::invalid_argument("fitSegment needs at least 2 points");
  }
  const double n = static_cast<double>(points.size());
  double mx = 0.0;
  double my = 0.0;
  for (const auto & p : points) {
    mx += p.x;
    my += p.y;
  }
  mx /= n;
  my /= n;
  double sxx = 0.0;
  double syy = 0.0;
  double sxy = 0.0;
  for (const auto & p : points) {
    sxx += (p.x - mx) * (p.x - mx);
    syy += (p.y - my) * (p.y - my);
    sxy += (p.x - mx) * (p.y - my);
  }
  // Direction of the principal axis of the 2x2 covariance matrix: the line
  // that minimizes the perpendicular (not vertical) distances.
  const double theta = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
  const double ux = std::cos(theta);
  const double uy = std::sin(theta);

  double tmin = std::numeric_limits<double>::max();
  double tmax = std::numeric_limits<double>::lowest();
  double sq = 0.0;
  for (const auto & p : points) {
    const double t = (p.x - mx) * ux + (p.y - my) * uy;
    const double perp = -(p.x - mx) * uy + (p.y - my) * ux;
    tmin = std::min(tmin, t);
    tmax = std::max(tmax, t);
    sq += perp * perp;
  }
  Segment s;
  s.start = {mx + tmin * ux, my + tmin * uy};
  s.end = {mx + tmax * ux, my + tmax * uy};
  s.num_points = points.size();
  s.rms_error = std::sqrt(sq / n);
  return s;
}

LineExtractor::LineExtractor(const LineExtractorConfig & config)
: config_(config)
{
  validatePositive(config_.max_gap, "max_gap");
  validatePositive(config_.split_distance, "split_distance");
  validatePositive(config_.merge_angle, "merge_angle");
  validatePositive(config_.merge_distance, "merge_distance");
  validatePositive(config_.merge_gap, "merge_gap");
  validatePositive(config_.min_length, "min_length");
  validatePositive(config_.corner_min_angle, "corner_min_angle");
  if (!std::isfinite(config_.corner_max_extension) || config_.corner_max_extension < 0.0) {
    throw std::invalid_argument("corner_max_extension must be finite and >= 0");
  }
  if (config_.min_points < 2) {
    throw std::invalid_argument("min_points must be >= 2");
  }
}

std::vector<Segment> LineExtractor::extract(const std::vector<PointChain> & chains) const
{
  // 1-3: break at gaps, split, fit.
  std::vector<Candidate> candidates;
  for (const auto & chain : chains) {
    for (const auto & piece : breakAtGaps(chain, config_.max_gap)) {
      if (piece.size() < 2) {
        continue;
      }
      for (auto & part : split(piece, config_.split_distance)) {
        if (part.size() >= 2) {
          candidates.push_back(makeCandidate(std::move(part)));
        }
      }
    }
  }

  // 4: global merge until nothing changes. Longest first, so short pieces
  // get absorbed into the walls they belong to.
  bool merged = true;
  while (merged) {
    merged = false;
    std::sort(
      candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
        return a.segment.length() > b.segment.length();
      });
    for (std::size_t i = 0; i < candidates.size() && !merged; ++i) {
      for (std::size_t j = i + 1; j < candidates.size(); ++j) {
        if (mergeable(candidates[i], candidates[j], config_)) {
          auto pts = std::move(candidates[i].points);
          pts.insert(pts.end(), candidates[j].points.begin(), candidates[j].points.end());
          candidates[i] = makeCandidate(std::move(pts));
          candidates.erase(candidates.begin() + static_cast<std::ptrdiff_t>(j));
          merged = true;
          break;
        }
      }
    }
  }

  // 5: filter.
  std::vector<Segment> out;
  for (const auto & c : candidates) {
    if (c.segment.num_points >= config_.min_points && c.segment.length() >= config_.min_length) {
      out.push_back(c.segment);
    }
  }

  // 6: close corners between the surviving walls.
  closeCorners(out, config_.corner_min_angle, config_.corner_max_extension);
  return out;
}

}  // namespace landerpi_web_nav
