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
      p.soft_radius <= hard_radius())
    throw std::invalid_argument("Invalid local costmap parameters");
  n_ = static_cast<int>(std::ceil(p.size / p.resolution)) | 1;
  odds_.assign(n_ * n_, 0);
  seen_.assign(n_ * n_, false);
  distance_.assign(n_ * n_, 0);
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
  inflate();
  ready_ = true;
  return true;
}
void LocalCostmap::inflate() {
  std::vector<double> temporary(n_ * n_), input(n_);
  for (int y = 0; y < n_; ++y) {
    for (int x = 0; x < n_; ++x)
      input[x] = odds_[y * n_ + x] >= 2 ? 0 : 1e9;
    const auto result = distance_transform(input);
    for (int x = 0; x < n_; ++x)
      temporary[y * n_ + x] = result[x];
  }
  for (int x = 0; x < n_; ++x) {
    for (int y = 0; y < n_; ++y)
      input[y] = temporary[y * n_ + x];
    const auto result = distance_transform(input);
    for (int y = 0; y < n_; ++y)
      distance_[y * n_ + x] =
          std::max(0.0, std::sqrt(result[y]) * p_.resolution -
                            p_.resolution / std::sqrt(2.0));
  }
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
           distance_[y * n_ + x] <= offset;
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
    if (depth < 6)
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
      p.max_speed,     p.max_angular,          p.acceleration,
      p.deceleration,  p.angular_acceleration, p.horizon,
      p.wall_distance, p.goal_tolerance};
  if (!std::all_of(values.begin(), values.end(),
                   [](double v) { return std::isfinite(v) && v > 0; }) ||
      !std::isfinite(p.reaction_time) || p.reaction_time < 0 ||
      p.horizon <= p.reaction_time || !std::isfinite(p.goal_x) ||
      !std::isfinite(p.goal_y))
    throw std::invalid_argument("Invalid local planner parameters");
}
WallCommand CostmapFollower::step(const WallPose &pose, const Sectors &s,
                                  const LocalCostmap &map, double dt,
                                  bool fresh) {
  best_path_.clear();
  predicted_clearance_ = 0;
  moving_candidates_ = 0;
  if (!fresh || !map.ready() || !s.usable() || !std::isfinite(pose.x) ||
      !std::isfinite(pose.y) || !std::isfinite(pose.yaw) ||
      !std::isfinite(pose.linear_speed) || !std::isfinite(pose.angular_speed) ||
      !std::isfinite(dt))
    return {};
  if (mode_ == LocalMode::complete)
    return {};
  const double goal_distance =
      std::hypot(p_.goal_x - pose.x, p_.goal_y - pose.y);
  if (goal_distance < p_.goal_tolerance) {
    mode_ = LocalMode::complete;
    return {};
  }
  const double initial_clearance = map.clearance(pose.x, pose.y);
  // A newly observed obstacle may overlap an existing footprint. Only a
  // shallow overlap permits escape; unknown space and deep overlaps stop.
  if (initial_clearance <=
      std::max(0.0, map.hard_radius() - map.resolution())) {
    mode_ = LocalMode::blocked;
    return {};
  }
  if (initial_clearance < map.hard_radius() + 0.04)
    recovering_ = true;
  if (initial_clearance > map.hard_radius() + 0.075)
    recovering_ = false;
  std::vector<WallPoint> reference;
  if (recovering_) {
    double best = -1e9, direction = pose.yaw;
    for (int i = 0; i < 32; ++i) {
      const double angle = i * 2 * pi / 32;
      const double d = map.clearance(pose.x + 0.15 * std::cos(angle),
                                     pose.y + 0.15 * std::sin(angle));
      const double score = d + 0.005 * std::cos(angle - pose.yaw);
      if (score > best) {
        best = score;
        direction = angle;
      }
    }
    reference = {{pose.x, pose.y},
                 {pose.x + 0.4 * std::cos(direction),
                  pose.y + 0.4 * std::sin(direction)}};
    mode_ = LocalMode::recovering;
  } else if (goal_distance < 1.2 &&
             map.line_clear({pose.x, pose.y}, {p_.goal_x, p_.goal_y})) {
    mode_ = LocalMode::exiting;
    reference = {{pose.x, pose.y}, {p_.goal_x, p_.goal_y}};
  } else {
    mode_ = LocalMode::following;
    // Hold a laser-derived local target until reached. Re-selecting it at
    // every scan can alternate between both sides of a dead end.
    if (have_target_ &&
        std::hypot(target_.x - pose.x, target_.y - pose.y) < 0.18) {
      anchor_ = target_anchor_;
      have_anchor_ = true;
      have_target_ = false;
    }
    if (have_target_ && !map.traversable(target_.x, target_.y))
      have_target_ = false;
    if (!have_target_) {
      const auto boundary =
          map.boundary_path(pose, p_.wall_distance, anchor_, have_anchor_);
      double arc = 0;
      for (std::size_t i = 1; i < boundary.size(); ++i) {
        arc += std::hypot(boundary[i].x - boundary[i - 1].x,
                          boundary[i].y - boundary[i - 1].y);
        if (arc < 0.85)
          continue;
        WallPoint target = boundary[i];
        const auto before = boundary[i > 2 ? i - 2 : 0];
        const auto after = boundary[std::min(boundary.size() - 1, i + 2)];
        const double length =
            std::hypot(after.x - before.x, after.y - before.y);
        if (length > 1e-6) {
          const WallPoint shifted{
              target.x + 0.10 * (after.y - before.y) / length,
              target.y - 0.10 * (after.x - before.x) / length};
          if (map.traversable(shifted.x, shifted.y))
            target = shifted;
        }
        auto route = map.route({pose.x, pose.y}, target);
        if (route.size() < 2)
          continue;
        target_ = target;
        target_anchor_ = boundary[i];
        have_target_ = true;
        reference = std::move(route);
        break;
      }
    } else
      reference = map.route({pose.x, pose.y}, target_);
  }
  if (reference.size() < 2) {
    mode_ = LocalMode::blocked;
    return {};
  }
  reference_ = reference;
  std::vector<double> cumulative(reference.size(), 0);
  for (std::size_t i = 1; i < reference.size(); ++i)
    cumulative[i] =
        cumulative[i - 1] + std::hypot(reference[i].x - reference[i - 1].x,
                                       reference[i].y - reference[i - 1].y);
  auto projection = [&](double x, double y) {
    double best = 1e9, along = 0, heading = pose.yaw;
    for (std::size_t i = 1; i < reference.size(); ++i) {
      const auto a = reference[i - 1], b = reference[i];
      const double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
      if (l2 < 1e-10)
        continue;
      const double t =
          std::clamp(((x - a.x) * dx + (y - a.y) * dy) / l2, 0.0, 1.0);
      const double distance = (x - a.x - t * dx) * (x - a.x - t * dx) +
                              (y - a.y - t * dy) * (y - a.y - t * dy);
      if (distance < best) {
        best = distance;
        along = cumulative[i - 1] + t * std::sqrt(l2);
        heading = std::atan2(dy, dx);
      }
    }
    return std::array<double, 3>{std::sqrt(best), along, heading};
  };
  const auto look = std::min(
      reference.size() - 1,
      static_cast<std::size_t>(
          std::lower_bound(cumulative.begin(), cumulative.end(), 0.55) -
          cumulative.begin()));
  desired_yaw_ =
      std::atan2(reference[look].y - pose.y, reference[look].x - pose.x);
  const double heading_error =
      wrap_angle(projection(pose.x, pose.y)[2] - pose.yaw);
  dt = std::clamp(dt, 0.01, 0.1);
  const double upper_v =
      std::abs(heading_error) > 0.8
          ? 0
          : (recovering_ ? std::min(0.15, p_.max_speed) : p_.max_speed) *
                std::pow(std::cos(heading_error), 2);
  std::vector<double> rotations{
      0, std::clamp(1.8 * heading_error - 0.2 * pose.angular_speed,
                    -p_.max_angular, p_.max_angular)};
  rotations.push_back(
      std::clamp(wrap_angle(projection(pose.x, pose.y)[2] - pose.yaw) /
                     std::max(0.5, p_.horizon - p_.reaction_time),
                 -p_.max_angular, p_.max_angular));
  for (int i = 0; i <= 30; ++i)
    rotations.push_back(-p_.max_angular + 2 * p_.max_angular * i / 30);
  double best = 1e30;
  WallCommand chosen{};
  for (int vi = 0; vi <= 6; ++vi)
    for (const double w : rotations) {
      const double v = upper_v * vi / 6;
      WallPose predicted = pose;
      double clearance = 100, penalty = 0;
      bool valid = true;
      std::vector<WallPoint> path;
      constexpr double step = 0.04;
      // Include the complete stop from the candidate speed, even when
      // the measured velocity is still zero at the start of acceleration.
      const double duration =
          p_.horizon +
          std::max(std::max(v, std::max(0.0, pose.linear_speed)) /
                       p_.deceleration,
                   std::max(std::abs(w), std::abs(pose.angular_speed)) /
                       p_.angular_acceleration);
      for (double time = 0; time < duration; time += step) {
        const double desired_v = time < std::max(p_.reaction_time, dt)
                                     ? pose.linear_speed
                                     : (time < p_.horizon ? v : 0);
        const double desired_w = time < std::max(p_.reaction_time, dt)
                                     ? pose.angular_speed
                                     : (time < p_.horizon ? w : 0);
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
      const auto end = projection(predicted.x, predicted.y);
      const double score =
          -3 * std::min(end[1], 1.0) + 2 * end[0] +
          0.8 * std::abs(wrap_angle(end[2] - predicted.yaw)) +
          0.5 * penalty / path.size() +
          4.0 * std::exp(-40 * (clearance - map.hard_radius())) +
          0.5 * (p_.max_speed - v) + 0.04 * std::abs(w - pose.angular_speed);
      if (score < best) {
        best = score;
        chosen = {v, w};
        best_path_ = std::move(path);
        predicted_clearance_ = clearance;
      }
    }
  if (best == 1e30) {
    mode_ = LocalMode::blocked;
    return {};
  }
  return chosen;
}
} // namespace maze_mouse_navigation
