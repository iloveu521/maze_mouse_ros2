#include "maze_mouse_navigation/scan_geometry.hpp"
#include <algorithm>
#include <functional>
#include <limits>

namespace maze_mouse_navigation {
ScanGeometry analyse_scan(std::span<const float> ranges, double amin,
                          double inc, double rmin, double rmax,
                          ScanGeometryParameters p) {
  ScanGeometry out;
  out.sectors = {extract_sector(ranges, amin, inc, rmin, rmax, 0,
                                p.front_degrees * pi / 180),
                 extract_sector(ranges, amin, inc, rmin, rmax, pi / 2,
                                p.side_degrees * pi / 180),
                 extract_sector(ranges, amin, inc, rmin, rmax, -pi / 2,
                                p.side_degrees * pi / 180)};
  if (ranges.empty() || !std::isfinite(inc) || inc == 0)
    return out;
  std::vector<WallPoint> points(ranges.size());
  std::vector<bool> hit(ranges.size());
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i], a = amin + i * inc;
    hit[i] = std::isfinite(r) && r >= rmin && r <= rmax;
    points[i] = {r * std::cos(a), r * std::sin(a)};
  }
  auto connected = [&](std::size_t i, std::size_t j) {
    return hit[i] && hit[j] &&
           std::hypot(points[i].x - points[j].x, points[i].y - points[j].y) <
               0.16;
  };
  std::size_t first = 0;
  // Start after a discontinuity so a front wall spanning the scan seam stays
  // whole.
  for (std::size_t i = 0; i < ranges.size(); ++i)
    if (!connected((i + ranges.size() - 1) % ranges.size(), i)) {
      first = i;
      break;
    }
  std::vector<WallPoint> cluster;
  std::function<void(std::size_t, std::size_t)> fit;
  fit = [&](std::size_t lo, std::size_t hi) {
    if (hi <= lo + 2)
      return;
    WallPoint mean{};
    for (auto i = lo; i < hi; ++i) {
      mean.x += cluster[i].x;
      mean.y += cluster[i].y;
    }
    mean.x /= hi - lo;
    mean.y /= hi - lo;
    double xx = 0, yy = 0, xy = 0;
    for (auto i = lo; i < hi; ++i) {
      const double x = cluster[i].x - mean.x, y = cluster[i].y - mean.y;
      xx += x * x;
      yy += y * y;
      xy += x * y;
    }
    const double a = 0.5 * std::atan2(2 * xy, xx - yy);
    const double dx = std::cos(a), dy = std::sin(a);
    double residual = 0;
    auto split = lo;
    for (auto i = lo + 1; i + 1 < hi; ++i) {
      const double d = std::abs(-(cluster[i].x - mean.x) * dy +
                                (cluster[i].y - mean.y) * dx);
      if (d > residual) {
        residual = d;
        split = i;
      }
    }
    // Curved caps/corners must not get fitted as one diagonal long wall.
    const auto start = cluster[lo], end = cluster[hi - 1];
    residual = std::max(
        residual, std::abs(-(start.x - mean.x) * dy + (start.y - mean.y) * dx));
    residual = std::max(
        residual, std::abs(-(end.x - mean.x) * dy + (end.y - mean.y) * dx));
    if (residual > p.fit_tolerance && hi - lo > 5) {
      if (split <= lo + 1 || split + 2 >= hi)
        split = (lo + hi) / 2;
      fit(lo, split + 1);
      fit(split, hi);
      return;
    }
    if (residual > p.fit_tolerance * 1.5)
      return;
    double tmin = 1e9, tmax = -1e9;
    for (auto i = lo; i < hi; ++i) {
      const double t =
          (cluster[i].x - mean.x) * dx + (cluster[i].y - mean.y) * dy;
      tmin = std::min(tmin, t);
      tmax = std::max(tmax, t);
    }
    WallSegment segment{{mean.x + tmin * dx, mean.y + tmin * dy},
                        {mean.x + tmax * dx, mean.y + tmax * dy}};
    if (segment.length() >= p.minimum_wall_length)
      out.walls.push_back(segment);
    else
      ++out.short_faces;
  };
  for (std::size_t k = 0; k < ranges.size(); ++k) {
    const auto i = (first + k) % ranges.size();
    if (!cluster.empty() &&
        !connected((i + ranges.size() - 1) % ranges.size(), i)) {
      fit(0, cluster.size());
      cluster.clear();
    }
    if (hit[i])
      cluster.push_back(points[i]);
  }
  fit(0, cluster.size());
  // Preserve raw validity/minimum range for fail-stop and collision
  // diagnostics. Only the wall-distance estimate ignores returns belonging to
  // short faces.
  auto filtered = [&](Sector &s, double center, double width) {
    if (!s.usable)
      return;
    std::vector<double> distances;
    for (std::size_t i = 0; i < ranges.size(); ++i) {
      const double delta = wrap_angle(amin + i * inc - center);
      if (!hit[i] || std::abs(delta) > width / 2 + 1e-8)
        continue;
      for (const auto &wall : out.walls) {
        const double dx = wall.b.x - wall.a.x, dy = wall.b.y - wall.a.y;
        const double t = std::clamp(
            ((points[i].x - wall.a.x) * dx + (points[i].y - wall.a.y) * dy) /
                (dx * dx + dy * dy),
            0.0, 1.0);
        if (std::hypot(points[i].x - wall.a.x - t * dx,
                       points[i].y - wall.a.y - t * dy) <=
            p.fit_tolerance * 1.5) {
          distances.push_back(ranges[i] * std::cos(delta));
          break;
        }
      }
    }
    if (distances.size() < 3)
      s.distance = rmax;
    else {
      const auto middle = distances.begin() + distances.size() / 2;
      std::nth_element(distances.begin(), middle, distances.end());
      s.distance = *middle;
    }
  };
  filtered(out.sectors.front, 0, p.front_degrees * pi / 180);
  filtered(out.sectors.left, pi / 2, p.side_degrees * pi / 180);
  filtered(out.sectors.right, -pi / 2, p.side_degrees * pi / 180);
  return out;
}
} // namespace maze_mouse_navigation
