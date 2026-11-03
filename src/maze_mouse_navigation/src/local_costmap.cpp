#include "maze_mouse_navigation/local_costmap.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
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
  const std::array positive{p.resolution, p.size, p.robot_radius};
  if (!std::all_of(positive.begin(), positive.end(),
                   [](double v) { return std::isfinite(v) && v > 0; }) ||
      !std::isfinite(p.safety_margin) || p.safety_margin < 0 || p.size < 1 ||
      !std::isfinite(p.origin_x) || !std::isfinite(p.origin_y) ||
      p.size / p.resolution > 400 || !std::isfinite(p.laser_x) ||
      !std::isfinite(p.laser_y) || !std::isfinite(p.laser_yaw) ||
      !std::isfinite(p.soft_radius) || p.soft_radius < 0 ||
      (p.soft_radius > 0 && p.soft_radius <= hard_radius()) ||
      !(p.side_sector_degrees > 0 && p.side_sector_degrees <= 180) ||
      !(p.front_sector_degrees > 0 && p.front_sector_degrees <= 180) ||
      !(p.minimum_wall_length > 0 && std::isfinite(p.minimum_wall_length)) ||
      !(p.wall_fit_tolerance > 0 && std::isfinite(p.wall_fit_tolerance)))
    throw std::invalid_argument("Invalid local costmap parameters");
  n_ = static_cast<int>(std::ceil(p.size / p.resolution)) | 1;
  if (p.fixed) {
    ox_ = p.origin_x;
    oy_ = p.origin_y;
  }
  odds_.assign(n_ * n_, 0);
  seen_.assign(n_ * n_, false);
  distance_.assign(n_ * n_, 0);
}
void LocalCostmap::configure(CostmapParameters p) {
  LocalCostmap checked(p);
  const bool reset = p.resolution != p_.resolution || p.size != p_.size ||
                     p.fixed != p_.fixed || p.origin_x != p_.origin_x ||
                     p.origin_y != p_.origin_y || p.laser_x != p_.laser_x ||
                     p.laser_y != p_.laser_y || p.laser_yaw != p_.laser_yaw;
  if (reset)
    *this = std::move(checked);
  else
    p_ = p;
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
  if (p_.fixed) {
    if (index(pose.x, pose.y) < 0)
      return false;
  } else {
    shift(pose.x, pose.y);
  }
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
bool LocalCostmap::observed(double x, double y) const {
  const int cell = index(x, y);
  return ready_ && cell >= 0 && seen_[cell];
}
bool LocalCostmap::occupied(double x, double y) const {
  const int cell = index(x, y);
  return ready_ && cell >= 0 && odds_[cell] >= 2;
}
double LocalCostmap::tracking_clearance(double x, double y) const {
  const int cell = index(x, y);
  if (!ready_ || cell < 0 || !seen_[cell])
    return 0;
  const double coarse = clearance(x, y);
  const double radius = hard_radius() + .10;
  if (coarse > radius)
    return coarse;
  // Compute distance to occupied cell squares near the footprint directly.
  // The planning distance field subtracts two grid-error bounds; applying
  // those again while tracking rejected physically clear cached corners.
  double nearest = radius;
  const int lo_x = std::max(
      0, static_cast<int>(std::floor((x - radius - ox_) / p_.resolution)));
  const int hi_x = std::min(
      n_ - 1, static_cast<int>(std::floor((x + radius - ox_) / p_.resolution)));
  const int lo_y = std::max(
      0, static_cast<int>(std::floor((y - radius - oy_) / p_.resolution)));
  const int hi_y = std::min(
      n_ - 1, static_cast<int>(std::floor((y + radius - oy_) / p_.resolution)));
  for (int cy = lo_y; cy <= hi_y; ++cy)
    for (int cx = lo_x; cx <= hi_x; ++cx) {
      if (odds_[cy * n_ + cx] < 2)
        continue;
      const double dx =
          std::max(0.0, std::abs(x - (ox_ + (cx + .5) * p_.resolution)) -
                            p_.resolution / 2);
      const double dy =
          std::max(0.0, std::abs(y - (oy_ + (cy + .5) * p_.resolution)) -
                            p_.resolution / 2);
      nearest = std::min(nearest, std::hypot(dx, dy));
    }
  return nearest;
}
bool LocalCostmap::tracking_line_clear(WallPoint a, WallPoint b) const {
  const double length = std::hypot(b.x - a.x, b.y - a.y);
  const int steps =
      std::max(1, static_cast<int>(std::ceil(length / (p_.resolution * .25))));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    if (tracking_clearance(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)) <=
        tracking_radius())
      return false;
  }
  return true;
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
      out[i] = p_.soft_radius > 0 && d < p_.soft_radius
                   ? static_cast<std::int8_t>(
                         90 * std::exp(-8 * (d - hard_radius())))
                   : 0;
  }
  return out;
}
const char *local_mode_name(LocalMode m) {
  switch (m) {
  case LocalMode::following:
    return "FOLLOW_PATH";
  case LocalMode::complete:
    return "COMPLETE";
  case LocalMode::blocked:
    return "BLOCKED";
  }
  return "BLOCKED";
}
CostmapFollower::CostmapFollower(LocalPlannerParameters p) : p_(p) {
  const std::array values{p.max_speed,
                          p.max_angular,
                          p.acceleration,
                          p.deceleration,
                          p.angular_acceleration,
                          p.horizon,
                          p.goal_tolerance,
                          p.heading_stop_angle,
                          p.progress_weight,
                          p.path_weight,
                          p.heading_weight,
                          p.speed_weight,
                          p.pursuit_lookahead,
                          p.pursuit_lookahead_gain,
                          p.pursuit_lookahead_max,
                          p.path_preview_distance,
                          p.path_validation_distance,
                          p.path_deviation,
                          p.heading_kp,
                          p.curvature_feedforward,
                          p.max_lateral_acceleration,
                          p.angular_command_acceleration,
                          p.pursuit_weight};
  if (!std::all_of(values.begin(), values.end(),
                   [](double v) { return std::isfinite(v) && v > 0; }) ||
      !std::isfinite(p.clearance_weight) || p.clearance_weight < 0 ||
      !std::isfinite(p.reaction_time) || p.reaction_time < 0 ||
      p.horizon <= p.reaction_time ||
      p.pursuit_lookahead_max < p.pursuit_lookahead || p.linear_samples < 2 ||
      p.linear_samples > 40 || p.angular_samples < 8 ||
      p.angular_samples > 120 || p.heading_stop_angle >= pi / 2 ||
      !std::isfinite(p.goal_x) || !std::isfinite(p.goal_y))
    throw std::invalid_argument("Invalid local planner parameters");
}
void CostmapFollower::configure(LocalPlannerParameters p) {
  CostmapFollower checked(p);
  if (p.path_validation_distance != p_.path_validation_distance ||
      p.path_deviation != p_.path_deviation) {
    reference_.clear();
    last_reference_.clear();
  }
  previous_command_angular_ =
      std::clamp(previous_command_angular_, -p.max_angular, p.max_angular);
  filtered_target_angular_ =
      std::clamp(filtered_target_angular_, -p.max_angular, p.max_angular);
  p_ = p;
}
std::vector<WallPoint> CostmapFollower::refresh_reference(
    std::vector<WallPoint> candidate, const WallPose &pose,
    const LocalCostmap &map) {
  reference_update_code_ = 0;
  reference_remaining_ = 0;
  reference_candidate_points_ = candidate.size();
  if (candidate.size() < 2) {
    reference_update_code_ = 10;
    return {};
  }
  double cumulative = 0, best = 1e30;
  std::size_t best_segment = 1;
  WallPoint projected = candidate.front();
  for (std::size_t i = 1; i < candidate.size(); ++i) {
    if (cumulative > 2.0)
      break; // Only project near the robot; a self-crossing path must not
             // hijack the reference to a later branch.
    const auto a = candidate[i - 1], b = candidate[i];
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    const double length = std::sqrt(length2);
    if (length2 > 1e-12) {
      const double t = std::clamp(
          ((pose.x - a.x) * dx + (pose.y - a.y) * dy) / length2, 0.0, 1.0);
      const WallPoint q{a.x + t * dx, a.y + t * dy};
      const double d = std::hypot(pose.x - q.x, pose.y - q.y);
      if (d < best) {
        best = d;
        best_segment = i;
        projected = q;
      }
    }
    cumulative += length;
  }
  if (best > p_.path_deviation) {
    reference_update_code_ = 3;
    return {};
  }
  std::vector<WallPoint> active{projected};
  for (std::size_t i = best_segment; i < candidate.size(); ++i)
    if (std::hypot(candidate[i].x - active.back().x,
                   candidate[i].y - active.back().y) > .015)
      active.push_back(candidate[i]);
  if (active.size() < 2) {
    reference_update_code_ = 10;
    return {};
  }
  double checked = 0.0;
  std::size_t safe = 1;
  WallPoint previous = active.front();
  while (safe < active.size() && checked < p_.path_validation_distance) {
    if (!map.tracking_line_clear(previous, active[safe]))
      break;
    checked += std::hypot(active[safe].x - previous.x,
                          active[safe].y - previous.y);
    previous = active[safe];
    ++safe;
  }
  if (safe < 2) {
    reference_update_code_ = 4;
    return {};
  }
  if (safe < active.size()) {
    active.resize(safe);
    reference_update_code_ = 14;
  }
  double remaining = 0;
  for (std::size_t i = 1; i < active.size(); ++i)
    remaining += std::hypot(active[i].x - active[i - 1].x,
                            active[i].y - active[i - 1].y);
  reference_remaining_ = remaining;
  return active;
}
WallCommand CostmapFollower::step(const WallPose &pose, const Sectors &s,
                                  const LocalCostmap &map, double dt,
                                  bool fresh,
                                  const std::vector<WallPoint> &planned_path) {
  if (reference_.size() >= 2)
    last_reference_ = reference_;
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
  const double goal_distance =
      std::hypot(p_.goal_x - pose.x, p_.goal_y - pose.y);
  if (goal_distance < p_.goal_tolerance) {
    mode_ = LocalMode::complete;
    previous_command_angular_ = 0;
    filtered_target_angular_ = 0;
    heading_integral_ = 0;
    return {};
  }
  const double initial_clearance = map.tracking_clearance(pose.x, pose.y);
  if (initial_clearance <= map.robot_radius() - .05) {
    mode_ = LocalMode::blocked;
    previous_command_angular_ = 0;
    filtered_target_angular_ = 0;
    return {};
  }
  // A physical collision or residual momentum can leave the body slightly
  // inside the tracking radius. In that state only accept arcs whose final
  // clearance improves; never let one tick's small decrease become the next
  // tick's lower limit.
  const bool recovering_overlap = initial_clearance < map.tracking_radius();
  const double minimum_clearance =
      recovering_overlap
          ? std::max(map.robot_radius() - .04, initial_clearance - .005)
          : map.tracking_radius() - .002;
  if (!recovering_overlap)
    recovery_direction_ = 0;
  if (initial_clearance >= map.tracking_radius() + .02)
    recovery_direction_ = 0;
  // Latch the escape direction while the body is at the tracking floor.
  // Recomputing it from the left/right sectors every tick makes the robot
  // oscillate as it rotates and the sectors swap sides.
  auto recovery_angular = [&]() -> double {
    if (recovery_direction_ == 0) {
      const double left_gap = s.left.distance, right_gap = s.right.distance;
      if (std::isfinite(left_gap) && std::isfinite(right_gap) &&
          std::abs(left_gap - right_gap) > .05)
        recovery_direction_ = left_gap < right_gap ? -1 : 1;
    }
    if (recovery_direction_ == 0)
      return 0.0;
    const double delta =
        p_.angular_command_acceleration * std::clamp(dt, .01, .1);
    const double angular =
        std::clamp(recovery_direction_ * .9, previous_command_angular_ - delta,
                   previous_command_angular_ + delta);
    previous_command_angular_ = angular;
    return angular;
  };
  // Rotation alone cannot translate a differential drive out of a side wall.
  // Creep along the latched turn only when the predicted arc's clearance does
  // not decrease (and improves by the end); otherwise stay rotational.
  auto recovery_creep = [&](double angular) -> double {
    WallPose predicted = pose;
    double clearance = initial_clearance;
    constexpr double step = .04;
    constexpr double speed = .12;
    for (double time = 0; time < .30; time += step) {
      predicted.x += speed * std::cos(predicted.yaw) * step;
      predicted.y += speed * std::sin(predicted.yaw) * step;
      predicted.yaw = wrap_angle(predicted.yaw + angular * step);
      const double d = map.tracking_clearance(predicted.x, predicted.y);
      if (d <= clearance - .002)
        return 0.0;
      clearance = d;
    }
    return clearance > initial_clearance + .003 ? speed : 0.0;
  };
  std::vector<WallPoint> candidate = planned_path;
  if (candidate.size() < 2)
    candidate = last_reference_;
  reference_ = refresh_reference(std::move(candidate), pose, map);
  if (reference_.size() < 2) {
    mode_ = LocalMode::blocked;
    previous_command_angular_ = 0;
    filtered_target_angular_ = 0;
    return {};
  }
  mode_ = LocalMode::following;
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
        lateral = -std::sin(heading) * (px - x) + std::cos(heading) * (py - y);
      }
    }
    return std::array<double, 4>{std::sqrt(best), along, heading, lateral};
  };
  dt = std::clamp(dt, .01, .1);
  const auto projected = projection(pose.x, pose.y, .65);
  const double lookahead =
      std::clamp(p_.pursuit_lookahead +
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
    if (map.tracking_line_clear({pose.x, pose.y}, {candidate[0], candidate[1]}))
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
  // Pure pursuit already accounts for position and heading error. Mixing
  // the curvature of a later bend into steering made the car turn left while
  // the current cached arc was still turning right around the inner corner.
  // Preview geometry is used below to brake, never to change this tangent.
  const double curvature = pursuit_curvature;
  const double heading_error = pursuit_heading_error;
  tracking_error_ = pursuit_heading_error;
  const double speed_limit = p_.max_speed;
  double preview_speed =
      std::min(speed_limit, std::sqrt(p_.max_lateral_acceleration /
                                      std::max(.01, std::abs(curvature))));
  // Keep enough lateral room to brake for a corner. In a 0.88 m corridor the
  // unrestricted 1.0 m/s profile reaches a wall faster than the sampler can
  // stop; this governor is the physical stopping-distance limit.
  const double wall_margin =
      std::max(0.0, initial_clearance - map.tracking_radius());
  preview_speed =
      std::min(preview_speed,
               std::sqrt(2.0 * p_.deceleration * wall_margin) + .05);
  for (std::size_t i = 3; i + 3 < reference.size(); ++i) {
    const double ahead = cumulative[i] - projected[1];
    if (ahead < 0 || ahead > p_.path_preview_distance)
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
  target_angular_ = std::clamp(p_.curvature_feedforward * curvature * upper_v,
                               -p_.max_angular, p_.max_angular);
  // Slew is limited in the sampler; a second low-pass filter delays the
  // corner command and causes the car to cut inside the cached curve.
  filtered_target_angular_ = target_angular_;
  const double horizon = p_.horizon;
  const double stationary_error =
      std::abs(heading_error) > .08 ? heading_error : pursuit_heading_error;
  const double raw_stationary_angular =
      std::abs(target_angular_) > .3
          ? target_angular_
          : std::clamp(p_.heading_kp * stationary_error, -p_.max_angular,
                       p_.max_angular);
  if (std::abs(tracking_error_) <= .7)
    alignment_turn_direction_ = 0;
  else if (std::abs(tracking_error_) > p_.heading_stop_angle &&
           alignment_turn_direction_ == 0) {
    const double initial_turn =
        std::abs(target_angular_) > .05 ? target_angular_ : tracking_error_;
    alignment_turn_direction_ = initial_turn >= 0 ? 1 : -1;
  }
  const double stationary_angular =
      alignment_turn_direction_ != 0 &&
              std::abs(tracking_error_) > p_.heading_stop_angle
          ? alignment_turn_direction_ *
                std::clamp(std::abs(raw_stationary_angular), .45,
                           p_.max_angular)
          : raw_stationary_angular;
  // Large path-heading errors intentionally set upper_v to zero. In that
  // state a differential drive should rotate continuously in place; sending
  // the turn through the translational candidate sampler can alternate
  // FOLLOW/BLOCKED and reset angular acceleration on every rejected cycle.
  // The robot footprint is circular, so a stationary spin is safe whenever
  // the measured clearance already exceeds the hard inflated radius.
  if (upper_v < .005 && std::abs(tracking_error_) > p_.heading_stop_angle &&
      std::abs(pose.linear_speed) < .04 &&
      initial_clearance > map.tracking_radius() + .01) {
    const double command_delta = p_.angular_command_acceleration * dt;
    const double turn_magnitude =
        std::clamp(std::abs(stationary_angular), .45, p_.max_angular);
    const double continuous_turn = alignment_turn_direction_ * turn_magnitude;
    const double angular =
        std::clamp(continuous_turn, previous_command_angular_ - command_delta,
                   previous_command_angular_ + command_delta);
    previous_command_angular_ = angular;
    return {0, angular};
  }
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
        v < .005 ? stationary_angular
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
      double clearance = 100;
      double final_clearance = 0;
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
        const double d = map.tracking_clearance(predicted.x, predicted.y);
        if (d <= minimum_clearance) {
          valid = false;
          break;
        }
        clearance = std::min(clearance, d);
        final_clearance = d;
        path.push_back({predicted.x, predicted.y});
      }
      if (!valid || path.empty())
        continue;
      if (recovering_overlap &&
          final_clearance <= initial_clearance + .002)
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
          p_.clearance_weight *
              std::exp(-40 * (clearance - map.tracking_radius())) +
          p_.speed_weight * (p_.max_speed - v) +
          0.12 * std::abs(w - pose.angular_speed) +
          p_.pursuit_weight * std::abs(w - desired_w);
      if (v > .005 && end[1] - projected[1] > .005 &&
          std::abs(w - limited_w) < 1e-5 && v > tracking_speed &&
          clearance > map.tracking_radius()) {
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
    // If every forward arc collides, first rotate away from the nearer wall.
    // A tracking-floor stop leaves a differential drive with no way to
    // translate sideways; facing the open side lets the next sampler cycle
    // find an improving arc.
    if (std::abs(pose.linear_speed) < .04 &&
        initial_clearance > map.robot_radius() - .04) {
      const double angular = recovery_angular();
      if (angular != 0.0) {
        previous_command_angular_ = angular;
        mode_ = LocalMode::following;
        return {recovery_creep(angular), angular};
      }
    }
    // If every forward arc collides, keep a safe stationary turn toward the
    // path tangent. Zeroing the command here at every rejected scan cycle
    // used to make a corner turn pulse on/off forever even when the robot had
    // room to rotate in place.
    if (std::abs(pose.linear_speed) < .04 &&
        initial_clearance > map.tracking_radius() + .01 &&
        std::abs(stationary_angular) > .10) {
      const double command_delta = p_.angular_command_acceleration * dt;
      const double angular = std::clamp(
          stationary_angular, previous_command_angular_ - command_delta,
          previous_command_angular_ + command_delta);
      previous_command_angular_ = angular;
      mode_ = LocalMode::following;
      return {0, angular};
    }
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
  if (tracking_speed > .005) {
    chosen = tracking_command;
    best_path_ = std::move(tracking_path);
    predicted_clearance_ = tracking_clearance;
  }
  // A stationary local minimum at the tracking floor cannot translate out of
  // the wall; rotate and creep toward the more open side instead of parking.
  if (chosen.linear < .02 && std::abs(pose.linear_speed) < .04 &&
      initial_clearance < map.tracking_radius() + .03 &&
      initial_clearance > map.robot_radius() - .04) {
    const double angular = recovery_angular();
    if (angular != 0.0)
      return {recovery_creep(angular), angular};
  }
  if (chosen.linear > .05)
    recovery_direction_ = 0;
  // Hold a side buffer while driving. A pure-pursuit arc at speed cuts close
  // to inner corners; once the centre sits inside the 0.18 m hard inflation,
  // every downstream route, branch exit and dead-end probe is rejected and
  // the robot can only creep or stall. Nudge away from the nearer wall.
  if (chosen.linear > .05) {
    const double left_gap = s.left.distance, right_gap = s.right.distance;
    const double buffer = .24;
    if (right_gap < buffer && right_gap < left_gap)
      chosen.angular = std::clamp(chosen.angular + 1.2 * (buffer - right_gap),
                                  -p_.max_angular, p_.max_angular);
    else if (left_gap < buffer && left_gap < right_gap)
      chosen.angular = std::clamp(chosen.angular - 1.2 * (buffer - left_gap),
                                  -p_.max_angular, p_.max_angular);
  }
  previous_command_angular_ = chosen.angular;
  return chosen;
}
} // namespace maze_mouse_navigation
