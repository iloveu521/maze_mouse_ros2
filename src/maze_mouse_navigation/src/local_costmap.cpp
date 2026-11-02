#include "maze_mouse_navigation/local_costmap.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace maze_mouse_navigation {
namespace {
std::vector<double> distance_transform(const std::vector<double> &f) {
  const int n = static_cast<int>(f.size());
  std::vector<int> v(n);
  std::vector<double> z(n + 1), d(n);
  int k = 0;
  v[0] = 0;
  z[0] = -1e30;
  z[1] = 1e30;
  for (int q = 1; q < n; ++q) {
    double s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * (q - v[k]));
    while (s <= z[k]) {
      --k;
      s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * (q - v[k]));
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = 1e30;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q)
      ++k;
    d[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
  }
  return d;
}
} // namespace
LocalCostmap::LocalCostmap(CostmapParameters p) : p_(p) {
  const std::array positive{p.resolution, p.size, p.robot_radius,
                            p.soft_radius};
  if (!std::all_of(positive.begin(), positive.end(),
                   [](double v) { return std::isfinite(v) && v > 0; }) ||
      !std::isfinite(p.safety_margin) || p.safety_margin < 0 || p.size < 1 ||
      p.size / p.resolution > 400 || !std::isfinite(p.laser_x) ||
      !std::isfinite(p.laser_y) || !std::isfinite(p.laser_yaw) ||
      p.soft_radius <= hard_radius() ||
      !(p.side_sector_degrees > 0 && p.side_sector_degrees <= 180) ||
      !(p.front_sector_degrees > 0 && p.front_sector_degrees <= 180) ||
      !(p.minimum_wall_length > 0 && std::isfinite(p.minimum_wall_length)) ||
      !(p.wall_fit_tolerance > 0 && std::isfinite(p.wall_fit_tolerance)))
    throw std::invalid_argument("Invalid local costmap parameters");
  n_ = static_cast<int>(std::ceil(p.size / p.resolution)) | 1;
  odds_.assign(n_ * n_, 0);
  seen_.assign(n_ * n_, false);
  distance_.assign(n_ * n_, 0);
  guide_distance_ = distance_;
}
void LocalCostmap::configure(CostmapParameters p) {
  LocalCostmap checked(p);
  const bool reset = p.resolution != p_.resolution || p.size != p_.size ||
                     p.laser_x != p_.laser_x || p.laser_y != p_.laser_y ||
                     p.laser_yaw != p_.laser_yaw;
  if (reset)
    *this = std::move(checked);
  else {
    if (p.minimum_wall_length != p_.minimum_wall_length ||
        p.wall_fit_tolerance != p_.wall_fit_tolerance)
      guide_walls_.clear();
    p_ = p;
  }
}
int LocalCostmap::index(double x, double y) const {
  if (!std::isfinite(x) || !std::isfinite(y))
    return -1;
  const int ix = static_cast<int>(std::floor((x - ox_) / p_.resolution));
  const int iy = static_cast<int>(std::floor((y - oy_) / p_.resolution));
  return ix >= 0 && iy >= 0 && ix < n_ && iy < n_ ? iy * n_ + ix : -1;
}
void LocalCostmap::shift(double x, double y) {
  const double nx = (std::floor(x / p_.resolution) - n_ / 2) * p_.resolution;
  const double ny = (std::floor(y / p_.resolution) - n_ / 2) * p_.resolution;
  if (!ready_) {
    ox_ = nx;
    oy_ = ny;
    return;
  }
  const int dx = static_cast<int>(std::llround((nx - ox_) / p_.resolution));
  const int dy = static_cast<int>(std::llround((ny - oy_) / p_.resolution));
  if (dx == 0 && dy == 0)
    return;
  std::vector<std::int8_t> odds(n_ * n_, 0);
  std::vector<bool> seen(n_ * n_, false);
  for (int iy = 0; iy < n_; ++iy)
    for (int ix = 0; ix < n_; ++ix) {
      const int xold = ix + dx, yold = iy + dy;
      if (xold >= 0 && yold >= 0 && xold < n_ && yold < n_) {
        odds[iy * n_ + ix] = odds_[yold * n_ + xold];
        seen[iy * n_ + ix] = seen_[yold * n_ + xold];
      }
    }
  odds_ = std::move(odds);
  seen_ = std::move(seen);
  ox_ = nx;
  oy_ = ny;
}
void LocalCostmap::trace(double ax, double ay, double bx, double by,
                         std::vector<bool> &mask) const {
  const int steps =
      std::max(1, static_cast<int>(
                      std::ceil(std::max(std::abs(bx - ax), std::abs(by - ay)) /
                                p_.resolution * 1.5)));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const int j = index(ax + t * (bx - ax), ay + t * (by - ay));
    if (j >= 0)
      mask[j] = true;
  }
}
bool LocalCostmap::update(const WallPose &pose, std::span<const float> ranges,
                          double amin, double inc, double rmin, double rmax) {
  if (ranges.size() < 10 || !std::isfinite(amin) || !std::isfinite(inc) ||
      inc == 0 || !std::isfinite(rmax) || !(rmin >= 0 && rmax > rmin) ||
      !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !std::isfinite(pose.yaw))
    return false;
  int valid = 0;
  for (float r : ranges)
    if ((std::isinf(r) && r > 0) ||
        (std::isfinite(r) && r >= rmin && r <= rmax))
      ++valid;
  if (valid < 0.7 * ranges.size())
    return false;
  shift(pose.x, pose.y);
  scan_geometry_ =
      analyse_scan(ranges, amin + p_.laser_yaw, inc, rmin, rmax,
                   {p_.side_sector_degrees, p_.front_sector_degrees,
                    p_.minimum_wall_length, p_.wall_fit_tolerance});
  std::vector<bool> free(n_ * n_, false), hit(n_ * n_, false);
  const double sx = pose.x + std::cos(pose.yaw) * p_.laser_x -
                    std::sin(pose.yaw) * p_.laser_y;
  const double sy = pose.y + std::sin(pose.yaw) * p_.laser_x +
                    std::cos(pose.yaw) * p_.laser_y;
  WallPoint previous{};
  bool previous_hit = false;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    const bool occupied = std::isfinite(r) && r >= rmin && r <= rmax;
    const bool no_return = std::isinf(r) && r > 0;
    if (!occupied && !no_return) {
      previous_hit = false;
      continue;
    }
    const double theta = pose.yaw + p_.laser_yaw + amin + i * inc;
    const double length = occupied ? r : rmax;
    const WallPoint end{sx + length * std::cos(theta),
                        sy + length * std::sin(theta)};
    trace(sx, sy, end.x, end.y, free);
    if (occupied) {
      const int j = index(end.x, end.y);
      if (j >= 0)
        hit[j] = true;
      if (previous_hit &&
          std::hypot(end.x - previous.x, end.y - previous.y) < 0.18)
        trace(previous.x, previous.y, end.x, end.y, hit);
    }
    previous = end;
    previous_hit = occupied;
  }
  for (int y = 0; y < n_; ++y)
    for (int x = 0; x < n_; ++x) {
      const int i = y * n_ + x;
      const double cx = ox_ + (x + 0.5) * p_.resolution,
                   cy = oy_ + (y + 0.5) * p_.resolution;
      if (std::hypot(cx - pose.x, cy - pose.y) < p_.robot_radius)
        free[i] = true;
      if (hit[i]) {
        odds_[i] = std::min(8, odds_[i] + 4);
        seen_[i] = true;
      } else if (free[i]) {
        odds_[i] = std::max(-4, odds_[i] - 2);
        seen_[i] = true;
      }
    }
  // Keep observed long wall faces in world coordinates across occlusions.
  // Short end faces still remain in odds_ and the collision distance field.
  guide_walls_.erase(
      std::remove_if(guide_walls_.begin(), guide_walls_.end(),
                     [&](const WallSegment &w) {
                       return std::min(
                                  std::hypot(w.a.x - pose.x, w.a.y - pose.y),
                                  std::hypot(w.b.x - pose.x, w.b.y - pose.y)) >
                              p_.size;
                     }),
      guide_walls_.end());
  for (auto wall : scan_geometry_.walls) {
    auto world = [&](WallPoint q) {
      return WallPoint{sx + std::cos(pose.yaw) * q.x - std::sin(pose.yaw) * q.y,
                       sy + std::sin(pose.yaw) * q.x +
                           std::cos(pose.yaw) * q.y};
    };
    wall = {world(wall.a), world(wall.b)};
    bool merged = false;
    for (auto &old : guide_walls_) {
      const double length = old.length();
      const double dx = (old.b.x - old.a.x) / length,
                   dy = (old.b.y - old.a.y) / length;
      const double da = (wall.a.x - old.a.x) * dx + (wall.a.y - old.a.y) * dy;
      const double db = (wall.b.x - old.a.x) * dx + (wall.b.y - old.a.y) * dy;
      const double error = std::max(
          std::abs(-(wall.a.x - old.a.x) * dy + (wall.a.y - old.a.y) * dx),
          std::abs(-(wall.b.x - old.a.x) * dy + (wall.b.y - old.a.y) * dx));
      if (error > p_.wall_fit_tolerance * 1.8 ||
          std::min(da, db) > length + 0.12 || std::max(da, db) < -0.12)
        continue;
      const WallPoint origin = old.a;
      const double lo = std::min(0.0, std::min(da, db)),
                   hi = std::max(length, std::max(da, db));
      old = {{origin.x + lo * dx, origin.y + lo * dy},
             {origin.x + hi * dx, origin.y + hi * dy}};
      merged = true;
      break;
    }
    if (!merged && guide_walls_.size() < 160)
      guide_walls_.push_back(wall);
  }
  inflate();
  ready_ = true;
  return true;
}
void LocalCostmap::inflate() {
  auto transform = [&](const std::vector<bool> &occupied,
                       std::vector<double> &field) {
    std::vector<double> temporary(n_ * n_), input(n_);
    for (int y = 0; y < n_; ++y) {
      for (int x = 0; x < n_; ++x)
        input[x] = occupied[y * n_ + x] ? 0 : 1e9;
      const auto result = distance_transform(input);
      for (int x = 0; x < n_; ++x)
        temporary[y * n_ + x] = result[x];
    }
    for (int x = 0; x < n_; ++x) {
      for (int y = 0; y < n_; ++y)
        input[y] = temporary[y * n_ + x];
      const auto result = distance_transform(input);
      for (int y = 0; y < n_; ++y)
        field[y * n_ + x] = std::max(0.0, std::sqrt(result[y]) * p_.resolution -
                                              p_.resolution / std::sqrt(2.0));
    }
  };
  std::vector<bool> occupied(n_ * n_);
  for (int i = 0; i < n_ * n_; ++i)
    occupied[i] = odds_[i] >= 2;
  transform(occupied, distance_);
  if (guide_walls_.empty()) {
    std::fill(guide_distance_.begin(), guide_distance_.end(), 1e9);
    return;
  }
  std::fill(occupied.begin(), occupied.end(), false);
  for (const auto &wall : guide_walls_)
    trace(wall.a.x, wall.a.y, wall.b.x, wall.b.y, occupied);
  transform(occupied, guide_distance_);
}

double LocalCostmap::clearance(double x, double y) const {
  const int cell = index(x, y);
  if (!ready_ || cell < 0 || !seen_[cell])
    return 0;
  const double gx = (x - ox_) / p_.resolution - 0.5;
  const double gy = (y - oy_) / p_.resolution - 0.5;
  const int ix = static_cast<int>(std::floor(gx)),
            iy = static_cast<int>(std::floor(gy));
  const double fx = gx - ix, fy = gy - iy;
  double distance = 0;
  for (int dy = 0; dy <= 1; ++dy)
    for (int dx = 0; dx <= 1; ++dx) {
      const int cx = std::clamp(ix + dx, 0, n_ - 1),
                cy = std::clamp(iy + dy, 0, n_ - 1);
      distance +=
          distance_[cy * n_ + cx] * (dx ? fx : 1 - fx) * (dy ? fy : 1 - fy);
    }
  // Bilinear interpolation is continuous. Subtract its maximum possible
  // overestimate using the 1-Lipschitz property of obstacle distance.
  return std::max(0.0, distance - p_.resolution / std::sqrt(2.0));
}
bool LocalCostmap::traversable(double x, double y) const {
  return clearance(x, y) > hard_radius();
}
bool LocalCostmap::line_clear(WallPoint a, WallPoint b) const {
  const double length = std::hypot(a.x - b.x, a.y - b.y);
  const int steps =
      std::max(1, static_cast<int>(std::ceil(length / (p_.resolution * 0.2))));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    if (clearance(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)) <=
        hard_radius())
      return false;
  }
  return true;
}
std::vector<WallPoint> LocalCostmap::route(WallPoint start,
                                           WallPoint goal) const {
  const int first = index(start.x, start.y), last = index(goal.x, goal.y);
  if (first < 0 || last < 0 || !traversable(start.x, start.y) ||
      !traversable(goal.x, goal.y))
    return {};
  auto point = [&](int i) {
    if (i == first)
      return start;
    if (i == last)
      return goal;
    return WallPoint{ox_ + (i % n_ + 0.5) * p_.resolution,
                     oy_ + (i / n_ + 0.5) * p_.resolution};
  };
  auto heuristic = [&](int i) {
    const auto p = point(i);
    return std::hypot(p.x - goal.x, p.y - goal.y);
  };
  std::vector<double> costs(n_ * n_, 1e30);
  std::vector<int> parents(n_ * n_, -1);
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  costs[first] = 0;
  open.push({heuristic(first), first});
  while (!open.empty()) {
    const auto [priority, i] = open.top();
    open.pop();
    if (priority > costs[i] + heuristic(i) + 1e-9)
      continue;
    if (i == last)
      break;
    for (int dy = -1; dy <= 1; ++dy)
      for (int dx = -1; dx <= 1; ++dx) {
        const int x = i % n_ + dx, y = i / n_ + dy;
        if ((!dx && !dy) || x < 0 || y < 0 || x >= n_ || y >= n_)
          continue;
        const int j = y * n_ + x;
        if (!line_clear(point(i), point(j)))
          continue;
        const double d = clearance(point(j).x, point(j).y);
        const double cost =
            costs[i] + p_.resolution * std::hypot(dx, dy) *
                           (1 + 4.0 * std::exp(-20 * (d - hard_radius())));
        if (cost < costs[j]) {
          costs[j] = cost;
          parents[j] = i;
          open.push({cost + heuristic(j), j});
        }
      }
  }
  if (first != last && parents[last] < 0)
    return {};
  std::vector<WallPoint> path{goal};
  for (int i = last; i != first; i = parents[i])
    path.push_back(point(i));
  path.push_back(point(first));
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  std::vector<WallPoint> smooth{start};
  for (std::size_t i = 0; i + 1 < path.size();) {
    std::size_t next = i + 1;
    for (std::size_t j = next + 1; j < path.size(); ++j)
      if (line_clear(path[i], path[j])) {
        const double length =
            std::hypot(path[j].x - path[i].x, path[j].y - path[i].y);
        const int steps =
            std::max(1, static_cast<int>(std::ceil(length / 0.01)));
        bool comfortable = true;
        const double required = std::min(
            hard_radius() + 0.055, clearance(path[i].x, path[i].y) - 0.002);
        for (int k = 1; k <= steps; ++k) {
          const double t = static_cast<double>(k) / steps;
          if (clearance(path[i].x + t * (path[j].x - path[i].x),
                        path[i].y + t * (path[j].y - path[i].y)) < required) {
            comfortable = false;
            break;
          }
        }
        if (comfortable)
          next = j;
      }
    smooth.push_back(path[next]);
    i = next;
  }
  return smooth;
}
std::vector<WallPoint> LocalCostmap::boundary_path(const WallPose &pose,
                                                   double offset,
                                                   WallPoint &anchor,
                                                   bool &have_anchor) const {
  struct Edge {
    int a, b;
  };
  const int vertices = (n_ + 1) * (n_ + 1);
  std::vector<Edge> edges;
  std::vector<std::vector<int>> outgoing(vertices);
  auto vertex = [&](int x, int y) { return y * (n_ + 1) + x; };
  // Trace an offset wall with occupied space always on the left.
  auto blocked = [&](int x, int y) {
    return x >= 0 && y >= 0 && x < n_ && y < n_ &&
           guide_distance_[y * n_ + x] <= offset;
  };
  auto add = [&](int a, int b) {
    const int i = edges.size();
    edges.push_back({a, b});
    outgoing[a].push_back(i);
  };
  for (int y = 0; y < n_; ++y)
    for (int x = 0; x < n_; ++x)
      if (blocked(x, y)) {
        if (y + 1 < n_ && !blocked(x, y + 1))
          add(vertex(x + 1, y + 1), vertex(x, y + 1));
        if (x + 1 < n_ && !blocked(x + 1, y))
          add(vertex(x + 1, y), vertex(x + 1, y + 1));
        if (y > 0 && !blocked(x, y - 1))
          add(vertex(x, y), vertex(x + 1, y));
        if (x > 0 && !blocked(x - 1, y))
          add(vertex(x, y + 1), vertex(x, y));
      }
  if (edges.empty())
    return {};
  auto point = [&](int id) {
    return WallPoint{ox_ + (id % (n_ + 1)) * p_.resolution,
                     oy_ + (id / (n_ + 1)) * p_.resolution};
  };
  auto projection = [&](int i, WallPoint p) {
    const auto a = point(edges[i].a), b = point(edges[i].b);
    const double t =
        std::clamp(((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) /
                       (p_.resolution * p_.resolution),
                   0.0, 1.0);
    return WallPoint{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
  };
  int seed = -1;
  double minimum = 1e9;
  if (have_anchor) {
    for (int i = 0; i < static_cast<int>(edges.size()); ++i) {
      const auto p = projection(i, anchor);
      const double d = std::hypot(p.x - anchor.x, p.y - anchor.y);
      if (d < minimum) {
        minimum = d;
        seed = i;
      }
    }
    if (minimum > 0.18)
      seed = -1;
  }
  if (seed < 0) {
    minimum = 1e9;
    for (int i = 0; i < static_cast<int>(edges.size()); ++i) {
      const auto p = projection(i, {pose.x, pose.y});
      const double dx = p.x - pose.x, dy = p.y - pose.y;
      const double left = -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
      if (left < 0)
        continue;
      const auto a = point(edges[i].a), b = point(edges[i].b);
      const double alignment = ((b.x - a.x) * std::cos(pose.yaw) +
                                (b.y - a.y) * std::sin(pose.yaw)) /
                               p_.resolution;
      const double score = std::hypot(dx, dy) + 0.03 * (1 - alignment);
      if (score < minimum) {
        minimum = score;
        seed = i;
      }
    }
  }
  if (seed < 0)
    return {};
  int start = seed;
  double nearest = 1e9;
  std::vector<std::pair<int, int>> stack{{seed, 0}};
  std::vector<bool> visited(edges.size(), false);
  while (!stack.empty()) {
    const auto [i, depth] = stack.back();
    stack.pop_back();
    if (visited[i])
      continue;
    visited[i] = true;
    const auto p = projection(i, {pose.x, pose.y});
    const double d = std::hypot(p.x - pose.x, p.y - pose.y);
    if (d < nearest - 1e-6) {
      nearest = d;
      start = i;
    }
    if (depth < static_cast<int>(1.6 / p_.resolution))
      for (int j : outgoing[edges[i].b])
        stack.push_back({j, depth + 1});
  }
  anchor = projection(start, {pose.x, pose.y});
  const auto finish = point(edges[start].b);
  const double remaining = std::hypot(finish.x - anchor.x, finish.y - anchor.y);
  if (remaining > 0.002) {
    anchor.x += 0.001 * (finish.x - anchor.x) / remaining;
    anchor.y += 0.001 * (finish.y - anchor.y) / remaining;
  }
  have_anchor = true;
  std::vector<WallPoint> path{anchor};
  int current = start;
  double length = 0;
  for (int count = 0; count < 300; ++count) {
    const auto a = point(edges[current].a), b = point(edges[current].b);
    length += std::hypot(b.x - path.back().x, b.y - path.back().y);
    path.push_back(b);
    if (length >= 3.0)
      break;
    int next = -1;
    double best = 10;
    for (int candidate : outgoing[edges[current].b]) {
      const auto c = point(edges[candidate].b);
      const double angle =
          std::atan2((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x),
                     (b.x - a.x) * (c.x - b.x) + (b.y - a.y) * (c.y - b.y));
      if (angle < best) {
        best = angle;
        next = candidate;
      }
    }
    if (next < 0 || next == start)
      break;
    current = next;
  }
  return path;
}

std::vector<std::int8_t> LocalCostmap::raw_data() const {
  std::vector<std::int8_t> out(n_ * n_, -1);
  for (int i = 0; i < n_ * n_; ++i)
    if (seen_[i])
      out[i] = odds_[i] >= 2 ? 100 : 0;
  return out;
}
std::vector<std::int8_t> LocalCostmap::cost_data() const {
  std::vector<std::int8_t> out(n_ * n_, -1);
  for (int i = 0; i < n_ * n_; ++i) {
    const double d =
        std::max(0.0, distance_[i] - p_.resolution / std::sqrt(2.0));
    if (d <= hard_radius())
      out[i] = 100;
    else if (seen_[i])
      out[i] = d < p_.soft_radius ? static_cast<std::int8_t>(
                                        90 * std::exp(-8 * (d - hard_radius())))
                                  : 0;
  }
  return out;
}
const char *local_mode_name(LocalMode m) {
  switch (m) {
  case LocalMode::following:
    return "FOLLOW_COSTMAP";
  case LocalMode::recovering:
    return "RECOVER_CLEARANCE";
  case LocalMode::u_turn:
    return "U_TURN";
  case LocalMode::exiting:
    return "EXITING";
  case LocalMode::complete:
    return "COMPLETE";
  case LocalMode::blocked:
    return "BLOCKED";
  }
  return "BLOCKED";
}
CostmapFollower::CostmapFollower(LocalPlannerParameters p) : p_(p) {
  const std::array values{
      p.max_speed,        p.max_angular,          p.acceleration,
      p.deceleration,     p.angular_acceleration, p.horizon,
      p.wall_distance,    p.goal_tolerance,       p.target_lookahead,
      p.target_tolerance, p.recovery_speed,       p.heading_stop_angle,
      p.progress_weight,  p.path_weight,          p.heading_weight,
      p.clearance_weight, p.speed_weight,         p.pursuit_lookahead,
      p.pursuit_lookahead_gain, p.pursuit_lookahead_max,
      p.path_preview_distance, p.path_refresh_distance,
      p.path_validation_distance, p.heading_kp, p.heading_ki, p.heading_kd,
      p.lateral_gain, p.curvature_feedforward, p.max_lateral_acceleration,
      p.angular_command_acceleration, p.angular_filter_time_constant,
      p.pursuit_weight, p.dead_end_distance, p.dead_end_front_gap,
      p.u_turn_speed, p.u_turn_lookahead, p.u_turn_horizon};
  if (!std::all_of(values.begin(), values.end(),
                   [](double v) { return std::isfinite(v) && v > 0; }) ||
      !std::isfinite(p.reaction_time) || p.reaction_time < 0 ||
      p.horizon <= p.reaction_time ||
      p.dead_end_front_gap >= p.dead_end_distance ||
      p.u_turn_horizon <= p.reaction_time || p.heading_stop_angle >= pi / 2 ||
      p.pursuit_lookahead_max < p.pursuit_lookahead ||
      p.linear_samples < 2 || p.linear_samples > 40 || p.angular_samples < 8 ||
      p.angular_samples > 120 || !std::isfinite(p.wall_offset) ||
      p.wall_offset < 0 || !std::isfinite(p.recovery_enter) ||
      p.recovery_enter < 0 || !std::isfinite(p.recovery_exit) ||
      p.recovery_exit <= p.recovery_enter || !std::isfinite(p.goal_x) ||
      !std::isfinite(p.goal_y))
    throw std::invalid_argument("Invalid local planner parameters");
}
void CostmapFollower::configure(LocalPlannerParameters p) {
  CostmapFollower checked(p);
  if (p.wall_distance != p_.wall_distance ||
      p.target_lookahead != p_.target_lookahead ||
      p.wall_offset != p_.wall_offset ||
      p.path_refresh_distance != p_.path_refresh_distance ||
      p.path_validation_distance != p_.path_validation_distance) {
    have_anchor_ = false;
    cached_reference_.clear();
  }
  if (p.heading_kp != p_.heading_kp || p.heading_ki != p_.heading_ki ||
      p.heading_kd != p_.heading_kd || p.lateral_gain != p_.lateral_gain) {
    heading_integral_ = 0;
    filtered_derivative_ = 0;
    have_control_error_ = false;
  }
  previous_command_angular_ =
      std::clamp(previous_command_angular_, -p.max_angular, p.max_angular);
  filtered_target_angular_ =
      std::clamp(filtered_target_angular_, -p.max_angular, p.max_angular);
  p_ = p;
}
std::vector<WallPoint>
CostmapFollower::wall_reference(const WallPose &pose, const LocalCostmap &map) {
  WallPose intent = pose;
  intent.yaw = guide_yaw_;
  // A continuous left wall defines the corridor heading even when the right
  // wall opens into a branch. Do not chase that branch's short end face.
  double best_wall = 1e9;
  std::vector<WallPoint> straight;
  const double length = std::max(1.8, p_.target_lookahead);
  for (const auto &wall : map.guide_walls()) {
    double dx = (wall.b.x - wall.a.x) / wall.length(),
           dy = (wall.b.y - wall.a.y) / wall.length();
    if (dx * std::cos(intent.yaw) + dy * std::sin(intent.yaw) < 0) {
      dx = -dx;
      dy = -dy;
    }
    if (dx * std::cos(intent.yaw) + dy * std::sin(intent.yaw) < std::cos(.55))
      continue;
    const double lateral = -(wall.a.x - pose.x) * dy + (wall.a.y - pose.y) * dx;
    const double ta = (wall.a.x - pose.x) * dx + (wall.a.y - pose.y) * dy;
    const double tb = (wall.b.x - pose.x) * dx + (wall.b.y - pose.y) * dy;
    if (lateral < .2 || lateral > 1.0 || std::min(ta, tb) > .1 ||
        std::max(ta, tb) < length + .10)
      continue;
    const double shift = lateral - p_.wall_distance - p_.wall_offset;
    std::vector<WallPoint> line;
    bool clear = true;
    for (double t = -.15; t <= length; t += .04) {
      WallPoint q{pose.x + t * dx - shift * dy, pose.y + t * dy + shift * dx};
      if (!map.traversable(q.x, q.y) ||
          (!line.empty() && !map.line_clear(line.back(), q))) {
        clear = false;
        break;
      }
      line.push_back(q);
    }
    const double score = lateral + .2 * (1 - dx * std::cos(intent.yaw) -
                                         dy * std::sin(intent.yaw));
    if (clear && score < best_wall && line.size() > 10 &&
        map.line_clear({pose.x, pose.y}, line[10])) {
      best_wall = score;
      straight = std::move(line);
    }
  }
  if (!straight.empty()) {
    // Keep the same wall component while using its fitted straight face.
    // Reseeding at a gap could jump to the next wall and skip a left opening.
    map.boundary_path(intent, p_.wall_distance, anchor_, have_anchor_);
    return straight;
  }
  auto path =
      map.boundary_path(intent, p_.wall_distance, anchor_, have_anchor_);
  if (path.size() < 2)
    return {};
  const auto boundary = path;
  // Estimate tangents over several cells, rather than the 0/90-degree grid
  // edges.
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto a = boundary[i > 3 ? i - 3 : 0];
    const auto b = boundary[std::min(i + 3, boundary.size() - 1)];
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    if (length < 1e-6)
      continue;
    for (double offset = p_.wall_offset; offset <= p_.wall_offset + .16;
         offset += .02) {
      const WallPoint q{path[i].x + offset * (b.y - a.y) / length,
                        path[i].y - offset * (b.x - a.x) / length};
      if (map.clearance(q.x, q.y) > map.hard_radius() + .015) {
        path[i] = q;
        break;
      }
    }
  }
  for (int pass = 0; pass < 60; ++pass) {
    const auto previous = path;
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
      const WallPoint q{.25 * previous[i - 1].x + .5 * previous[i].x +
                            .25 * previous[i + 1].x,
                        .25 * previous[i - 1].y + .5 * previous[i].y +
                            .25 * previous[i + 1].y};
      if (map.traversable(q.x, q.y) && map.line_clear(path[i - 1], q) &&
          map.line_clear(q, previous[i + 1]))
        path[i] = q;
    }
  }
  // Keep the ordered contour. A* to a far point would shortcut a U bend and
  // defer its steering until the robot is next to the end wall.
  std::vector<WallPoint> result;
  for (auto q : path) {
    if (!map.traversable(q.x, q.y) ||
        (!result.empty() && !map.line_clear(result.back(), q)))
      break;
    if (result.empty() ||
        std::hypot(q.x - result.back().x, q.y - result.back().y) > .015)
      result.push_back(q);
  }
  return result;
}
std::vector<WallPoint> CostmapFollower::refresh_reference(
    std::vector<WallPoint> candidate, const WallPose &pose,
    const LocalCostmap &map) {
  if (mode_ == LocalMode::recovering)
    return candidate;
  // The dead-end semicircle already has its own persistent, collision-checked
  // turn_path_. Do not project or splice it through the wall-follow cache.
  if (mode_ == LocalMode::u_turn) {
    cached_reference_.clear();
    cached_mode_ = mode_;
    return candidate;
  }
  if (cached_mode_ != mode_) {
    cached_reference_.clear();
    cached_mode_ = mode_;
  }
  const bool has_candidate = candidate.size() >= 2;
  if (!has_candidate && cached_reference_.size() < 2)
    return candidate;
  if (mode_ == LocalMode::exiting) {
    cached_reference_ = candidate;
    return candidate;
  }
  if (cached_reference_.size() < 2) {
    cached_reference_ = candidate;
    return candidate;
  }

  double best_distance2 = 1e30, along = 0;
  std::size_t nearest_segment = 0;
  WallPoint projected{};
  std::vector<double> cumulative(cached_reference_.size(), 0.0);
  for (std::size_t i = 1; i < cached_reference_.size(); ++i) {
    const auto a = cached_reference_[i - 1], b = cached_reference_[i];
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    const double length = std::sqrt(length2);
    cumulative[i] = cumulative[i - 1] + length;
    if (length2 < 1e-10)
      continue;
    const double t = std::clamp(
        ((pose.x - a.x) * dx + (pose.y - a.y) * dy) / length2, 0.0, 1.0);
    const WallPoint q{a.x + t * dx, a.y + t * dy};
    const double d2 = (pose.x - q.x) * (pose.x - q.x) +
                      (pose.y - q.y) * (pose.y - q.y);
    if (d2 < best_distance2) {
      best_distance2 = d2;
      along = cumulative[i - 1] + t * length;
      nearest_segment = i;
      projected = q;
    }
  }
  if (best_distance2 > .50 * .50) {
    cached_reference_ = candidate;
    return candidate;
  }

  std::vector<WallPoint> active{projected};
  for (std::size_t i = nearest_segment + 1; i < cached_reference_.size(); ++i)
    if (std::hypot(cached_reference_[i].x - active.back().x,
                   cached_reference_[i].y - active.back().y) > .015)
      active.push_back(cached_reference_[i]);
  const double remaining = std::max(0.0, cumulative.back() - along);

  bool valid = map.traversable(pose.x, pose.y);
  double checked = 0.0;
  WallPoint previous{pose.x, pose.y};
  for (std::size_t i = 1; valid && i < active.size() &&
                           checked < p_.path_validation_distance;
       ++i) {
    const auto q = active[i];
    const double length = std::hypot(q.x - previous.x, q.y - previous.y);
    if (!map.line_clear(previous, q)) {
      valid = false;
      break;
    }
    checked += length;
    previous = q;
  }
  if (!valid) {
    cached_reference_ = has_candidate ? candidate : std::vector<WallPoint>{};
    return candidate;
  }

  if (remaining <= p_.path_refresh_distance && has_candidate) {
    std::size_t join = 0;
    double join_distance = 1e30;
    for (std::size_t i = 0; i < candidate.size(); ++i) {
      const double d = std::hypot(candidate[i].x - active.back().x,
                                  candidate[i].y - active.back().y);
      if (d < join_distance) {
        join_distance = d;
        join = i;
      }
    }
    const auto old_a = active[active.size() - 2], old_b = active.back();
    const auto new_a = candidate[join];
    const auto new_b = candidate[std::min(join + 1, candidate.size() - 1)];
    const double old_yaw = std::atan2(old_b.y - old_a.y, old_b.x - old_a.x);
    const double new_yaw = std::atan2(new_b.y - new_a.y, new_b.x - new_a.x);
    if (join_distance < .45 && std::abs(wrap_angle(new_yaw - old_yaw)) < .95 &&
        map.line_clear(active.back(), candidate[join])) {
      active.back() = candidate[join];
      for (std::size_t i = join + 1; i < candidate.size(); ++i)
        if (std::hypot(candidate[i].x - active.back().x,
                       candidate[i].y - active.back().y) > .015)
          active.push_back(candidate[i]);
    } else if (remaining < .25) {
      active = candidate;
    }
  }
  cached_reference_ = active;
  return active;
}
std::vector<WallPoint> CostmapFollower::dead_end_path(const WallPose &pose,
                                                      const LocalCostmap &map) {
  if (std::abs(pose.angular_speed) > 1.0)
    return {};
  for (const auto &cap : map.guide_walls()) {
    double heading = std::atan2(cap.b.y - cap.a.y, cap.b.x - cap.a.x) + pi / 2;
    if (std::cos(heading - pose.yaw) < 0)
      heading = wrap_angle(heading + pi);
    if (std::abs(wrap_angle(heading - pose.yaw)) > .65)
      continue;
    const double fx = std::cos(heading), fy = std::sin(heading);
    auto local = [&](WallPoint p) {
      const double x = p.x - pose.x, y = p.y - pose.y;
      return WallPoint{x * fx + y * fy, -x * fy + y * fx};
    };
    auto world = [&](double x, double y) {
      return WallPoint{pose.x + x * fx - y * fy, pose.y + x * fy + y * fx};
    };
    const auto a = local(cap.a), b = local(cap.b);
    const double front = (a.x + b.x) / 2;
    if (std::abs(a.x - b.x) > .10 * cap.length() || front < .45 ||
        front > p_.dead_end_distance || std::min(a.y, b.y) > -.22 ||
        std::max(a.y, b.y) < .22)
      continue;
    struct Side {
      double y, lo, hi;
    };
    std::vector<Side> sides;
    for (const auto &side : map.guide_walls()) {
      const auto c = local(side.a), d = local(side.b);
      if (std::abs(c.y - d.y) > .15 * side.length())
        continue;
      const double y = (c.y + d.y) / 2;
      if (std::abs(y) > .18 && std::abs(y) < .85)
        sides.push_back({y, std::min(c.x, d.x), std::max(c.x, d.x)});
    }
    double left = 1e9, right = -1e9;
    for (const auto &side : sides) {
      if (side.lo > .2 || side.hi < .1)
        continue;
      double reach = side.hi;
      for (std::size_t pass = 0; pass < sides.size(); ++pass)
        for (const auto &next : sides)
          if (std::abs(next.y - side.y) < .08 && next.lo <= reach + .10 &&
              next.hi > reach)
            reach = next.hi;
      if (reach < front - .14)
        continue;
      if (side.y > 0)
        left = std::min(left, side.y);
      if (side.y < 0)
        right = std::max(right, side.y);
    }
    if (left == 1e9 || right == -1e9 || left - right > 1.25 ||
        std::min(a.y, b.y) > right + .14 || std::max(a.y, b.y) < left - .14)
      continue;
    const double margin =
        map.hard_radius() + std::sqrt(2.0) * map.resolution() + .045;
    const double hi = left - margin, lo = right + margin;
    const double radius = (hi - lo) / 2;
    if (radius < .07 || radius > .4)
      continue;
    const double entry = std::max(.12, front - p_.dead_end_front_gap - radius);
    std::vector<WallPoint> path;
    // Tangent-continuous entry blend, right semicircle, then an exit straight.
    for (int i = 0; i <= 24; ++i) {
      const double t = i / 24.0, u = 1 - t;
      const double x = 3 * u * u * t * std::min(.10, entry * .3) *
                           std::cos(pose.yaw - heading) +
                       3 * u * t * t * entry * .5 + t * t * t * entry;
      const double y = 3 * u * u * t * std::min(.10, entry * .3) *
                           std::sin(pose.yaw - heading) +
                       (3 * u * t * t + t * t * t) * hi;
      path.push_back(world(x, y));
    }
    for (int i = 1; i <= 40; ++i) {
      const double a = pi * i / 40;
      path.push_back(
          world(entry + radius * std::sin(a), hi - radius * (1 - std::cos(a))));
    }
    for (int i = 1; i <= 20; ++i) {
      const double t = i / 20.;
      const double exit_lane =
          std::clamp(right + p_.wall_distance + p_.wall_offset, lo, hi);
      path.push_back(
          world(entry - .5 * t, lo + (exit_lane - lo) * t * t * (3 - 2 * t)));
    }
    // The entry, semicircle, and exit meet with matching tangents, but their
    // sampled curvature changes abruptly. Smooth the joins while keeping both
    // endpoints fixed, accepting a point only when its neighboring segments
    // remain inside the observed inflated-free corridor.
    for (int pass = 0; pass < 32; ++pass) {
      const auto previous = path;
      for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        const WallPoint q{.2 * previous[i - 1].x + .6 * previous[i].x +
                              .2 * previous[i + 1].x,
                          .2 * previous[i - 1].y + .6 * previous[i].y +
                              .2 * previous[i + 1].y};
        if (map.traversable(q.x, q.y) &&
            map.line_clear(previous[i - 1], q) &&
            map.line_clear(q, previous[i + 1]))
          path[i] = q;
      }
    }
    bool clear = true;
    for (std::size_t i = 0; i < path.size(); ++i)
      if (!map.traversable(path[i].x, path[i].y) ||
          (i > 0 && !map.line_clear(path[i - 1], path[i]))) {
        clear = false;
        break;
      }
    if (clear) {
      dead_end_front_distance_ = front;
      ++dead_ends_;
      return path;
    }
  }
  return {};
}
WallCommand CostmapFollower::step(const WallPose &pose, const Sectors &s,
                                  const LocalCostmap &map, double dt,
                                  bool fresh) {
  best_path_.clear();
  reference_.clear();
  predicted_clearance_ = 0;
  moving_candidates_ = 0;
  if (!fresh || !map.ready() || !s.usable() || !std::isfinite(pose.x) ||
      !std::isfinite(pose.y) || !std::isfinite(pose.yaw) ||
      !std::isfinite(pose.linear_speed) || !std::isfinite(pose.angular_speed) ||
      !std::isfinite(dt)) {
    previous_command_angular_ = 0;
    filtered_target_angular_ = 0;
    have_control_error_ = false;
    return {};
  }
  if (mode_ == LocalMode::complete) {
    previous_command_angular_ = 0;
    filtered_target_angular_ = 0;
    return {};
  }
  if (!have_guide_yaw_) {
    guide_yaw_ = pose.yaw;
    have_guide_yaw_ = true;
  }
  const double goal_distance =
      std::hypot(p_.goal_x - pose.x, p_.goal_y - pose.y);
  if (goal_distance < p_.goal_tolerance) {
    mode_ = LocalMode::complete;
    previous_command_angular_ = 0;
    filtered_target_angular_ = 0;
    heading_integral_ = 0;
    return {};
  }
  const double initial_clearance = map.clearance(pose.x, pose.y);
  // If scan registration puts the footprint just inside its inflated radius,
  // rotating or commanding zero cannot escape the local minimum. A short
  // reverse is allowed only through observed cells whose clearance improves
  // along the complete retreat segment.
  if (initial_clearance <=
      std::max(0.0, map.hard_radius() - map.resolution())) {
    double previous = initial_clearance;
    bool retreat_clear = initial_clearance > 0;
    double retreat_end = initial_clearance;
    for (double distance = .04; distance <= .32; distance += .04) {
      const double x = pose.x - distance * std::cos(pose.yaw);
      const double y = pose.y - distance * std::sin(pose.yaw);
      const double clearance = map.clearance(x, y);
      if (clearance <= 0 || clearance + .005 < previous) {
        retreat_clear = false;
        break;
      }
      previous = clearance;
      retreat_end = clearance;
    }
    if (retreat_clear && retreat_end > initial_clearance + .06) {
      mode_ = LocalMode::recovering;
      previous_command_angular_ = 0;
      filtered_target_angular_ = 0;
      heading_integral_ = 0;
      have_control_error_ = false;
      return {-std::min(.12, p_.recovery_speed), 0};
    }
  }
  // A newly observed obstacle may overlap an existing footprint. Only a
  // shallow overlap permits escape; unknown space and deep overlaps stop.
  if (initial_clearance <=
      std::max(0.0, map.hard_radius() - map.resolution())) {
    mode_ = LocalMode::blocked;
    previous_command_angular_ = 0;
    return {};
  }
  if (initial_clearance <
      map.hard_radius() + (turn_path_.empty() ? p_.recovery_enter : .005))
    recovering_ = true;
  if (initial_clearance > map.hard_radius() + p_.recovery_exit)
    recovering_ = false;
  std::vector<WallPoint> candidate_reference;
  if (recovering_) {
    double best = -1e9, direction = pose.yaw;
    for (int i = 0; i < 32; ++i) {
      const double angle = i * 2 * pi / 32;
      const double d = map.clearance(pose.x + 0.15 * std::cos(angle),
                                     pose.y + 0.15 * std::sin(angle));
      const double score = d + 0.08 * std::cos(angle - guide_yaw_);
      if (score > best) {
        best = score;
        direction = angle;
      }
    }
    candidate_reference = {{pose.x, pose.y},
                           {pose.x + 0.4 * std::cos(direction),
                            pose.y + 0.4 * std::sin(direction)}};
    mode_ = LocalMode::recovering;
  } else if (goal_distance < 1.2 &&
             map.line_clear({pose.x, pose.y}, {p_.goal_x, p_.goal_y})) {
    mode_ = LocalMode::exiting;
    candidate_reference = {{pose.x, pose.y}, {p_.goal_x, p_.goal_y}};
  } else {
    mode_ = LocalMode::following;
    if (turn_path_.empty()) {
      turn_path_ = dead_end_path(pose, map);
      turn_index_ = 0;
    }
    if (!turn_path_.empty()) {
      const auto last = std::min(turn_index_ + 8, turn_path_.size() - 1);
      double best = 1e9;
      for (auto i = turn_index_; i <= last; ++i) {
        const auto next = turn_path_[std::min(i + 1, turn_path_.size() - 1)];
        const double tangent =
            std::atan2(next.y - turn_path_[i].y, next.x - turn_path_[i].x);
        const double d =
            std::hypot(turn_path_[i].x - pose.x, turn_path_[i].y - pose.y) +
            .08 * (1 - std::cos(tangent - pose.yaw));
        if (d < best) {
          best = d;
          turn_index_ = i;
        }
      }
      if (turn_index_ + 7 >= turn_path_.size() &&
          std::hypot(turn_path_.back().x - pose.x,
                     turn_path_.back().y - pose.y) < p_.target_tolerance) {
        const auto a = turn_path_[turn_path_.size() - 3], b = turn_path_.back();
        guide_yaw_ = std::atan2(b.y - a.y, b.x - a.x);
        turn_path_.clear();
        have_anchor_ = false;
      } else {
        mode_ = LocalMode::u_turn;
        candidate_reference.assign(
            turn_path_.begin() + (turn_index_ > 2 ? turn_index_ - 2 : 0),
            turn_path_.end());
      }
    }
    if (turn_path_.empty())
      candidate_reference = wall_reference(pose, map);
  }
  reference_ = refresh_reference(std::move(candidate_reference), pose, map);
  if (reference_.size() < 2) {
    mode_ = LocalMode::blocked;
    previous_command_angular_ = 0;
    return {};
  }
  const auto &reference = reference_;
  std::vector<double> cumulative(reference.size(), 0);
  for (std::size_t i = 1; i < reference.size(); ++i)
    cumulative[i] =
        cumulative[i - 1] + std::hypot(reference[i].x - reference[i - 1].x,
                                       reference[i].y - reference[i - 1].y);
  auto projection = [&](double x, double y, double limit = 1e9) {
    double best = 1e9, along = 0, heading = pose.yaw, lateral = 0;
    for (std::size_t i = 1; i < reference.size(); ++i) {
      const auto a = reference[i - 1], b = reference[i];
      const double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
      if (l2 < 1e-10)
        continue;
      if (cumulative[i - 1] > limit)
        break;
      const double t = std::clamp(
          ((x - a.x) * dx + (y - a.y) * dy) / l2, 0.0,
          std::clamp((limit - cumulative[i - 1]) / std::sqrt(l2), 0.0, 1.0));
      const double distance = (x - a.x - t * dx) * (x - a.x - t * dx) +
                              (y - a.y - t * dy) * (y - a.y - t * dy);
      if (distance < best) {
        best = distance;
        along = cumulative[i - 1] + t * std::sqrt(l2);
        heading = std::atan2(dy, dx);
        const double px = a.x + t * dx, py = a.y + t * dy;
        lateral = -std::sin(heading) * (px - x) +
                  std::cos(heading) * (py - y);
      }
    }
    return std::array<double, 4>{std::sqrt(best), along, heading, lateral};
  };
  dt = std::clamp(dt, .01, .1);
  const auto projected =
      projection(pose.x, pose.y, mode_ == LocalMode::u_turn ? .15 : .65);
  if (!recovering_ && mode_ != LocalMode::exiting)
    guide_yaw_ = wrap_angle(
        guide_yaw_ + std::clamp(wrap_angle(projected[2] - guide_yaw_),
                                -p_.max_angular * std::clamp(dt, .01, .1),
                                p_.max_angular * std::clamp(dt, .01, .1)));
  const double lookahead =
      mode_ == LocalMode::u_turn
          ? p_.u_turn_lookahead
          : std::clamp(p_.pursuit_lookahead +
                           p_.pursuit_lookahead_gain *
                               std::max(0.0, pose.linear_speed),
                       p_.pursuit_lookahead, p_.pursuit_lookahead_max);
  const double look_distance = projected[1] + lookahead;
  auto sample = [&](double distance) {
    distance = std::clamp(distance, 0.0, cumulative.back());
    for (std::size_t i = 1; i < reference.size(); ++i)
      if (cumulative[i] >= distance) {
        const double t = (distance - cumulative[i - 1]) /
                         std::max(1e-9, cumulative[i] - cumulative[i - 1]);
        return std::array<double, 3>{
            reference[i - 1].x + t * (reference[i].x - reference[i - 1].x),
            reference[i - 1].y + t * (reference[i].y - reference[i - 1].y),
            std::atan2(reference[i].y - reference[i - 1].y,
                       reference[i].x - reference[i - 1].x)};
      }
    const auto a = reference[reference.size() - 2], b = reference.back();
    return std::array<double, 3>{b.x, b.y, std::atan2(b.y - a.y, b.x - a.x)};
  };
  // The path may bend around an inner wall. Chasing its far point directly
  // would cut through that wall and produce a late emergency correction.
  double visible_distance = look_distance;
  while (visible_distance > projected[1] + .28) {
    const auto candidate = sample(visible_distance);
    if (map.line_clear({pose.x, pose.y}, {candidate[0], candidate[1]}))
      break;
    visible_distance -= .04;
  }
  control_lookahead_ = std::max(0.0, visible_distance - projected[1]);
  const auto look_sample = sample(visible_distance);
  const WallPoint look{look_sample[0], look_sample[1]};
  desired_yaw_ = std::atan2(look.y - pose.y, look.x - pose.x);
  const double pursuit_heading_error = wrap_angle(desired_yaw_ - pose.yaw);
  const double look_length =
      std::max(.12, std::hypot(look.x - pose.x, look.y - pose.y));
  const double pursuit_curvature =
      2 * std::sin(pursuit_heading_error) / look_length;
  const double heading_distance =
      std::min(cumulative.back(), projected[1] + .25);
  const auto heading_sample = sample(heading_distance);
  const double preview_end =
      std::min(cumulative.back(), projected[1] + p_.path_preview_distance);
  const auto preview = sample(preview_end);
  const double preview_length = std::max(.12, preview_end - heading_distance);
  const double path_curvature =
      wrap_angle(preview[2] - heading_sample[2]) / preview_length;
  const double curvature = .65 * pursuit_curvature + .35 * path_curvature;
  const double heading_error = wrap_angle(heading_sample[2] - pose.yaw);
  tracking_error_ = wrap_angle(
      heading_error +
      std::atan2(p_.lateral_gain * projected[3],
                 std::max(.22, std::max(0.0, pose.linear_speed))));
  const double raw_derivative =
      have_control_error_
          ? wrap_angle(tracking_error_ - previous_tracking_error_) / dt
          : 0;
  filtered_derivative_ = .90 * filtered_derivative_ + .10 * raw_derivative;
  const double proposed_integral =
      std::clamp(heading_integral_ + tracking_error_ * dt, -.35, .35);
  const double feedback_without_integral =
      p_.heading_kp * tracking_error_ + p_.heading_kd * filtered_derivative_;
  if (std::abs(tracking_error_) < .8 &&
      std::abs(feedback_without_integral + p_.heading_ki * proposed_integral) <
          p_.max_angular)
    heading_integral_ = proposed_integral;
  previous_tracking_error_ = tracking_error_;
  have_control_error_ = true;
  const double feedback_angular =
      feedback_without_integral + p_.heading_ki * heading_integral_;
  const double speed_limit =
      recovering_ ? std::min(p_.recovery_speed, p_.max_speed)
                  : (mode_ == LocalMode::u_turn
                         ? std::min(p_.u_turn_speed, p_.max_speed)
                         : p_.max_speed);
  double preview_speed = std::min(
      speed_limit,
      std::sqrt(p_.max_lateral_acceleration / std::max(.01, std::abs(curvature))));
  for (std::size_t i = 3; i + 3 < reference.size(); ++i) {
    const double ahead = cumulative[i] - projected[1];
    if (ahead < 0 || ahead > 1.0)
      continue;
    const auto a = reference[i - 3], b = reference[i], c = reference[i + 3];
    const double ab = std::hypot(b.x - a.x, b.y - a.y),
                 bc = std::hypot(c.x - b.x, c.y - b.y),
                 ac = std::hypot(c.x - a.x, c.y - a.y);
    if (ab * bc * ac < 1e-8)
      continue;
    const double k =
        2 * std::abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) /
        (ab * bc * ac);
    const double curve_speed =
        std::sqrt(p_.max_lateral_acceleration / std::max(.01, k));
    const double braking_distance = std::max(
        0., ahead - .12 - p_.reaction_time * std::max(0., pose.linear_speed));
    preview_speed = std::min(preview_speed,
                             std::sqrt(curve_speed * curve_speed +
                                       2 * p_.deceleration * braking_distance));
  }
  const double upper_v =
      std::abs(tracking_error_) > p_.heading_stop_angle
          ? 0
          : preview_speed * std::max(.25, std::cos(tracking_error_));
  target_angular_ = std::clamp(
      p_.curvature_feedforward * curvature * upper_v + feedback_angular,
      -p_.max_angular, p_.max_angular);
  const double filter_alpha = dt / (p_.angular_filter_time_constant + dt);
  filtered_target_angular_ +=
      filter_alpha * (target_angular_ - filtered_target_angular_);
  target_angular_ = filtered_target_angular_;
  const double horizon = mode_ == LocalMode::u_turn
                             ? std::min(p_.horizon, p_.u_turn_horizon)
                             : p_.horizon;
  const double stationary_error = std::abs(heading_error) > .08
                                      ? heading_error
                                      : pursuit_heading_error;
  const double stationary_angular =
      std::abs(target_angular_) > .3
          ? target_angular_
          : std::clamp(p_.heading_kp * stationary_error, -p_.max_angular,
                       p_.max_angular);
  double best = 1e30, best_moving = 1e30, moving_clearance = 0,
         tracking_speed = -1, tracking_clearance = 0;
  WallCommand chosen{}, moving_command{}, tracking_command{};
  std::vector<WallPoint> moving_path, tracking_path;
  const double command_delta = p_.angular_command_acceleration * dt;
  for (int vi = 0; vi <= p_.linear_samples; ++vi) {
    const double v = upper_v * vi / p_.linear_samples;
    // When collision prediction temporarily rejects every advancing arc, a
    // stopped candidate must still rotate toward the planned tangent. Using
    // feedback alone here can be nearly zero even though the path ahead bends,
    // leaving a differential drive indefinitely parked at the corner entry.
    const double desired_w =
        v < .005
            ? stationary_angular
            : std::clamp(target_angular_ + p_.curvature_feedforward *
                                                curvature * (v - upper_v),
                         -p_.max_angular, p_.max_angular);
    const double low_w =
        std::max(-p_.max_angular, previous_command_angular_ - command_delta);
    const double high_w =
        std::min(p_.max_angular, previous_command_angular_ + command_delta);
    const double limited_w = std::clamp(desired_w, low_w, high_w);
    const double search_step =
        std::max(.04, 2 * command_delta / std::max(8, p_.angular_samples));
    std::vector<double> candidates{limited_w, previous_command_angular_};
    for (int offset = 1; offset <= 2; ++offset) {
      candidates.push_back(
          std::clamp(limited_w - offset * search_step, low_w, high_w));
      candidates.push_back(
          std::clamp(limited_w + offset * search_step, low_w, high_w));
    }
    if (low_w <= 0 && high_w >= 0)
      candidates.push_back(0);
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(
        std::unique(candidates.begin(), candidates.end(),
                    [](double a, double b) { return std::abs(a - b) < 1e-5; }),
        candidates.end());
    for (const double w : candidates) {
      WallPose predicted = pose;
      double clearance = 100, penalty = 0;
      bool valid = true;
      std::vector<WallPoint> path;
      constexpr double step = 0.04;
      // Include the complete stop from the candidate speed, even when
      // the measured velocity is still zero at the start of acceleration.
      const double duration =
          horizon +
          std::max(std::max(v, std::max(0.0, pose.linear_speed)) /
                       p_.deceleration,
                   std::max(std::abs(w), std::abs(pose.angular_speed)) /
                       p_.angular_acceleration);
      for (double time = 0; time < duration; time += step) {
        const double desired_v = time < std::max(p_.reaction_time, dt)
                                     ? pose.linear_speed
                                     : (time < horizon ? v : 0);
        const double desired_w = time < std::max(p_.reaction_time, dt)
                                     ? pose.angular_speed
                                     : (time < horizon ? w : 0);
        predicted.linear_speed +=
            std::clamp(desired_v - predicted.linear_speed,
                       -p_.deceleration * step, p_.acceleration * step);
        predicted.angular_speed += std::clamp(
            desired_w - predicted.angular_speed,
            -p_.angular_acceleration * step, p_.angular_acceleration * step);
        predicted.x += predicted.linear_speed * std::cos(predicted.yaw) * step;
        predicted.y += predicted.linear_speed * std::sin(predicted.yaw) * step;
        predicted.yaw =
            wrap_angle(predicted.yaw + predicted.angular_speed * step);
        const double d = map.clearance(predicted.x, predicted.y);
        if (d <= std::min(map.hard_radius(), initial_clearance - 0.002)) {
          valid = false;
          break;
        }
        clearance = std::min(clearance, d);
        penalty += std::exp(-8 * std::max(0.0, d - map.hard_radius()));
        path.push_back({predicted.x, predicted.y});
      }
      if (!valid || path.empty())
        continue;
      if (v > 0.005)
        ++moving_candidates_;
      const auto end = projection(
          predicted.x, predicted.y,
          projected[1] +
              std::max(v, std::max(0.0, pose.linear_speed)) * duration * 1.2 +
              .10);
      const double score =
          -p_.progress_weight * std::clamp(end[1] - projected[1], -.25, 1.0) +
          p_.path_weight * end[0] +
          p_.heading_weight * std::abs(wrap_angle(end[2] - predicted.yaw)) +
          0.5 * penalty / path.size() +
          p_.clearance_weight *
              std::exp(-40 * (clearance - map.hard_radius())) +
          p_.speed_weight * (p_.max_speed - v) +
          0.12 * std::abs(w - pose.angular_speed) +
          p_.pursuit_weight * std::abs(w - desired_w);
      if (v > .005 && end[1] - projected[1] > .005 &&
          std::abs(w - limited_w) < 1e-5 && v > tracking_speed &&
          clearance >
              map.hard_radius() + (mode_ == LocalMode::u_turn ? .005 : .035)) {
        tracking_speed = v;
        tracking_command = {v, w};
        tracking_clearance = clearance;
        tracking_path = path;
      }
      if (v > .005 && end[1] - projected[1] > .005 && score < best_moving) {
        best_moving = score;
        moving_command = {v, w};
        moving_path = path;
        moving_clearance = clearance;
      }
      if (score < best) {
        best = score;
        chosen = {v, w};
        best_path_ = std::move(path);
        predicted_clearance_ = clearance;
      }
    }
  }
  if (best == 1e30) {
    mode_ = LocalMode::blocked;
    previous_command_angular_ = 0;
    return {};
  }
  // Collision-checked translational progress takes precedence over a stationary
  // local minimum. Rotation remains available when no advancing arc is safe.
  if (chosen.linear < .005 && best_moving < 1e30) {
    chosen = moving_command;
    best_path_ = std::move(moving_path);
    predicted_clearance_ = moving_clearance;
  }
  if (tracking_speed > .005 && !recovering_) {
    chosen = tracking_command;
    best_path_ = std::move(tracking_path);
    predicted_clearance_ = tracking_clearance;
  }
  previous_command_angular_ = chosen.angular;
  return chosen;
}
} // namespace maze_mouse_navigation
