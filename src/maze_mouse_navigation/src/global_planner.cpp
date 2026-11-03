#include "maze_mouse_navigation/global_planner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace maze_mouse_navigation {
namespace {
struct Projection {
  double deviation{1e30};
  double along{0.0};
  double remaining{0.0};
  std::size_t segment{0};
};

Projection project(const std::vector<WallPoint> &path, WallPoint p) {
  Projection result;
  if (path.empty())
    return result;
  if (path.size() == 1) {
    result.deviation = std::hypot(p.x - path[0].x, p.y - path[0].y);
    return result;
  }
  double cumulative = 0.0;
  double best = 1e30;
  double best_along = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto a = path[i - 1], b = path[i];
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    const double length = std::sqrt(length2);
    if (length2 > 1e-12) {
      const double t = std::clamp(
          ((p.x - a.x) * dx + (p.y - a.y) * dy) / length2, 0.0, 1.0);
      const double qx = a.x + t * dx, qy = a.y + t * dy;
      const double distance = std::hypot(p.x - qx, p.y - qy);
      if (distance < best) {
        best = distance;
        best_along = cumulative + t * length;
        result.segment = i;
      }
    }
    cumulative += length;
  }
  result.deviation = best;
  result.along = best_along;
  result.remaining = std::max(0.0, cumulative - best_along);
  return result;
}

bool segment_free(WallPoint a, WallPoint b,
                  const std::vector<std::uint8_t> &free,
                  const std::vector<float> &clearance,
                  double minimum_clearance, const LocalCostmap &map) {
  const int n = map.width();
  const double res = map.resolution();
  const double ox = map.origin_x(), oy = map.origin_y();
  int x = static_cast<int>(std::floor((a.x - ox) / res));
  int y = static_cast<int>(std::floor((a.y - oy) / res));
  const int end_x = static_cast<int>(std::floor((b.x - ox) / res));
  const int end_y = static_cast<int>(std::floor((b.y - oy) / res));
  if (x < 0 || y < 0 || x >= n || y >= n || end_x < 0 || end_y < 0 ||
      end_x >= n || end_y >= n)
    return false;
  if (!free[y * n + x] || clearance[y * n + x] < minimum_clearance ||
      !free[end_y * n + end_x] ||
      clearance[end_y * n + end_x] < minimum_clearance)
    return false;
  const double dx = b.x - a.x, dy = b.y - a.y;
  const int step_x = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
  const int step_y = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
  const double inf = std::numeric_limits<double>::infinity();
  double max_x = inf, max_y = inf, delta_x = inf, delta_y = inf;
  if (step_x > 0)
    max_x = ((ox + (x + 1) * res) - a.x) / dx;
  else if (step_x < 0)
    max_x = (a.x - (ox + x * res)) / -dx;
  if (step_y > 0)
    max_y = ((oy + (y + 1) * res) - a.y) / dy;
  else if (step_y < 0)
    max_y = (a.y - (oy + y * res)) / -dy;
  if (step_x != 0)
    delta_x = res / std::abs(dx);
  if (step_y != 0)
    delta_y = res / std::abs(dy);
  for (int guard = 0; guard <= 4 * n; ++guard) {
    if (x == end_x && y == end_y)
      return true;
    if (std::abs(max_x - max_y) < 1e-12) {
      // Exact grid-corner crossing: forbid cutting between two blocked
      // orthogonal cells.
      if (step_x != 0 && !free[y * n + x + step_x])
        return false;
      if (step_y != 0 && !free[(y + step_y) * n + x])
        return false;
      x += step_x;
      y += step_y;
      max_x += delta_x;
      max_y += delta_y;
    } else if (max_x < max_y) {
      x += step_x;
      max_x += delta_x;
    } else {
      y += step_y;
      max_y += delta_y;
    }
    if (x < 0 || y < 0 || x >= n || y >= n)
      return false;
    if (!free[y * n + x] || clearance[y * n + x] < minimum_clearance)
      return false;
  }
  return false;
}

int nearest_free_cell(const LocalCostmap &map,
                      const std::vector<std::uint8_t> &free, WallPoint p,
                      double radius) {
  const int n = map.width();
  const double res = map.resolution();
  const int cx = static_cast<int>(std::floor((p.x - map.origin_x()) / res));
  const int cy = static_cast<int>(std::floor((p.y - map.origin_y()) / res));
  if (cx < 0 || cy < 0 || cx >= n || cy >= n)
    return -1;
  const int reach = std::max(1, static_cast<int>(std::ceil(radius / res)));
  int best = -1;
  double best_distance = radius + 1e-9;
  for (int dy = -reach; dy <= reach; ++dy)
    for (int dx = -reach; dx <= reach; ++dx) {
      const int x = cx + dx, y = cy + dy;
      if (x < 0 || y < 0 || x >= n || y >= n || !free[y * n + x])
        continue;
      const auto center = map.cell_center(y * n + x);
      const double distance = std::hypot(center.x - p.x, center.y - p.y);
      if (distance <= best_distance) {
        best_distance = distance;
        best = y * n + x;
      }
    }
  return best;
}

} // namespace

GlobalPlanner::GlobalPlanner(GlobalPlannerParameters p) : p_(p) {
  const std::array values{p.replan_period,
                          p.frontier_weight,
                          p.frontier_min_distance,
                          p.frontier_switch_margin,
                          p.frontier_sticky_radius,
                          p.frontier_sticky_bonus,
                          p.start_snap_radius,
                          p.goal_snap_radius,
                          p.planning_margin,
                          p.clearance_cost_weight,
                          p.clearance_target,
                          p.path_min_remaining,
                          p.path_deviation,
                          p.path_validation_distance,
                          p.smoothing_span,
                          p.smoothing_clearance};
  if (!std::all_of(values.begin(), values.end(),
                   [](double v) { return std::isfinite(v) && v > 0; }) ||
      p.frontier_switch_margin < 0 || p.frontier_sticky_bonus < 0 ||
      p.clearance_cost_weight < 0)
    throw std::invalid_argument("Invalid global planner parameters");
}
void GlobalPlanner::configure(GlobalPlannerParameters p) {
  GlobalPlanner checked(p);
  p_ = p;
  dirty_ = true;
}

bool GlobalPlanner::update(const WallPose &pose, const LocalCostmap &map,
                           double dt, WallPoint goal, bool force) {
  if (!map.ready() || !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !std::isfinite(pose.yaw) || !std::isfinite(goal.x) ||
      !std::isfinite(goal.y) || !std::isfinite(dt))
    return false;
  dt = std::clamp(dt, 0.0, 0.5);
  since_plan_ += dt;
  bool replan = force || dirty_ || since_plan_ >= p_.replan_period;
  if (!replan && path_.size() >= 2) {
    const auto projected = project(path_, {pose.x, pose.y});
    bool ahead_clear = true;
    double checked = 0.0;
    for (std::size_t i = std::max<std::size_t>(1, projected.segment);
         i < path_.size() && checked < p_.path_validation_distance; ++i) {
      if (!map.tracking_line_clear(path_[i - 1], path_[i])) {
        ahead_clear = false;
        break;
      }
      checked += std::hypot(path_[i].x - path_[i - 1].x,
                            path_[i].y - path_[i - 1].y);
    }
    if (projected.deviation > p_.path_deviation ||
        projected.remaining < p_.path_min_remaining || !ahead_clear ||
        (target_ == PlannerTarget::frontier &&
         map.traversable(goal.x, goal.y)))
      replan = true;
  }
  if (!replan)
    return path_.size() >= 2;
  const auto started = std::chrono::steady_clock::now();
  dirty_ = false;
  since_plan_ = 0.0;
  ++replans_;
  const bool found = plan(pose, map, goal);
  plan_ms_ =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started)
          .count();
  return found && path_.size() >= 2;
}

double GlobalPlanner::path_remaining(const WallPose &pose) const {
  return project(path_, {pose.x, pose.y}).remaining;
}
double GlobalPlanner::lateral_deviation(const WallPose &pose) const {
  return project(path_, {pose.x, pose.y}).deviation;
}

bool GlobalPlanner::plan(const WallPose &pose, const LocalCostmap &map,
                         WallPoint goal) {
  path_.clear();
  target_ = PlannerTarget::none;
  target_point_ = WallPoint{};
  exhausted_ = false;
  frontier_candidates_ = 0;
  path_length_ = 0.0;
  const int n = map.width();
  if (n < 3)
    return false;
  const int count = map.cells();
  const double resolution = map.resolution();
  const double required = map.tracking_radius() + p_.planning_margin;
  std::vector<std::uint8_t> free(count, 0);
  std::vector<float> clearance(count, 0.0F);
  for (int i = 0; i < count; ++i) {
    const auto center = map.cell_center(i);
    // tracking_clearance is the exact distance to occupied cell squares and is
    // what the Pure Pursuit sampler enforces; using the coarser bilinear field
    // could create paths whose first segment fails tracking validation.
    const double distance = map.tracking_clearance(center.x, center.y);
    clearance[i] = static_cast<float>(distance);
    free[i] = distance > required;
  }

  const WallPoint start_point{pose.x, pose.y};
  const int pose_cell = map.cell_index(pose.x, pose.y);
  int start = pose_cell;
  if (start < 0 || !free[start])
    start = nearest_free_cell(map, free, start_point, p_.start_snap_radius);
  if (start < 0)
    return false;
  const bool exact_start = start == pose_cell && free[pose_cell];

  int goal_cell = map.cell_index(goal.x, goal.y);
  const bool exact_goal_cell = goal_cell >= 0 && free[goal_cell];
  if (!exact_goal_cell)
    goal_cell = nearest_free_cell(map, free, goal, p_.goal_snap_radius);
  const bool have_goal = goal_cell >= 0;

  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> costs(count, inf);
  std::vector<int> parents(count, -1);
  std::vector<std::uint8_t> closed(count, 0);
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  const auto heuristic = [&](int cell) {
    if (!have_goal)
      return 0.0;
    const auto a = map.cell_center(cell), b = map.cell_center(goal_cell);
    return std::hypot(a.x - b.x, a.y - b.y);
  };
  costs[start] = 0.0;
  open.push({heuristic(start), start});

  int best_frontier = -1;
  double best_biased = inf, best_raw = inf;
  int last_frontier_cell = -1;
  double last_raw = inf;
  bool reached_goal = false;
  while (!open.empty()) {
    const auto [priority, cell] = open.top();
    open.pop();
    if (closed[cell])
      continue;
    if (priority > costs[cell] + heuristic(cell) + 1e-9)
      continue;
    closed[cell] = 1;
    if (have_goal && cell == goal_cell) {
      reached_goal = true;
      break;
    }
    if (free[cell]) {
      const int cx = cell % n, cy = cell / n;
      const auto center = map.cell_center(cell);
      const double robot_distance =
          std::hypot(center.x - start_point.x, center.y - start_point.y);
      bool frontier = false;
      constexpr int dx4[4] = {1, -1, 0, 0};
      constexpr int dy4[4] = {0, 0, 1, -1};
      for (int k = 0; k < 4 && !frontier; ++k) {
        const int nx = cx + dx4[k], ny = cy + dy4[k];
        if (nx < 0 || ny < 0 || nx >= n || ny >= n)
          continue;
        const auto neighbor = map.cell_center(ny * n + nx);
        frontier = !map.observed(neighbor.x, neighbor.y);
      }
      if (frontier && robot_distance >= p_.frontier_min_distance) {
        ++frontier_candidates_;
        const double raw =
            costs[cell] + p_.frontier_weight * std::hypot(
                                                 center.x - goal.x,
                                                 center.y - goal.y);
        double biased = raw;
        if (have_last_frontier_ &&
            std::hypot(center.x - last_frontier_.x,
                       center.y - last_frontier_.y) <=
                p_.frontier_sticky_radius) {
          biased -= p_.frontier_sticky_bonus;
          if (raw < last_raw) {
            last_raw = raw;
            last_frontier_cell = cell;
          }
        }
        if (biased < best_biased) {
          best_biased = biased;
          best_raw = raw;
          best_frontier = cell;
        }
      }
    }
    const int cx = cell % n, cy = cell / n;
    for (int dy = -1; dy <= 1; ++dy)
      for (int dx = -1; dx <= 1; ++dx) {
        if (!dx && !dy)
          continue;
        const int nx = cx + dx, ny = cy + dy;
        if (nx < 0 || ny < 0 || nx >= n || ny >= n)
          continue;
        const int next = ny * n + nx;
        if (!free[next])
          continue;
        if (dx && dy && (!free[cy * n + nx] || !free[ny * n + cx]))
          continue; // never cut a corner
        const double step = resolution * std::hypot(dx, dy);
        const double minimum =
            std::min<double>(clearance[cell], clearance[next]);
        const double penalty =
            p_.clearance_cost_weight *
            std::clamp((p_.clearance_target - minimum) /
                           p_.clearance_target,
                       0.0, 1.0);
        const double candidate = costs[cell] + step * (1.0 + penalty);
        if (candidate + 1e-9 < costs[next]) {
          costs[next] = candidate;
          parents[next] = cell;
          open.push({candidate + heuristic(next), next});
        }
      }
  }

  int chosen = -1;
  PlannerTarget kind = PlannerTarget::none;
  if (reached_goal) {
    chosen = goal_cell;
    kind = PlannerTarget::goal;
  } else if (best_frontier >= 0) {
    chosen = best_frontier;
    if (last_frontier_cell >= 0 &&
        best_raw + p_.frontier_switch_margin >= last_raw)
      chosen = last_frontier_cell;
    kind = PlannerTarget::frontier;
  } else {
    exhausted_ = true;
    return false;
  }

  std::vector<int> cells;
  for (int i = chosen; i >= 0; i = parents[i]) {
    cells.push_back(i);
    if (i == start)
      break;
  }
  if (cells.empty() || cells.back() != start) {
    exhausted_ = true;
    return false;
  }
  std::reverse(cells.begin(), cells.end());
  std::vector<WallPoint> raw;
  if (exact_start)
    raw.push_back(start_point);
  for (const int cell : cells) {
    const auto center = map.cell_center(cell);
    if (raw.empty() ||
        std::hypot(center.x - raw.back().x, center.y - raw.back().y) > 0.01)
      raw.push_back(center);
  }
  if (kind == PlannerTarget::goal && exact_goal_cell &&
      map.traversable(goal.x, goal.y)) {
    if (std::hypot(goal.x - raw.back().x, goal.y - raw.back().y) > 0.01)
      raw.push_back(goal);
    target_point_ = goal;
  } else {
    target_point_ = map.cell_center(chosen);
  }
  target_ = kind;

  // Greedy line-of-sight shortcut on the precomputed free mask. The span limit
  // keeps the shortcut local so a U-shaped corridor is not cut through.
  std::vector<WallPoint> smooth;
  smooth.push_back(raw.front());
  std::size_t i = 0;
  while (i + 1 < raw.size()) {
    std::size_t best = i + 1;
    double length = 0.0;
    for (std::size_t j = i + 1; j < raw.size(); ++j) {
      length += std::hypot(raw[j].x - raw[j - 1].x, raw[j].y - raw[j - 1].y);
      if (length > p_.smoothing_span)
        break;
      if (segment_free(raw[i], raw[j], free, clearance,
                       p_.smoothing_clearance, map))
        best = j;
    }
    smooth.push_back(raw[best]);
    i = best;
  }
  bool smooth_clear = true;
  for (std::size_t k = 1; k < smooth.size(); ++k)
    if (!map.tracking_line_clear(smooth[k - 1], smooth[k])) {
      smooth_clear = false;
      break;
    }
  path_ = smooth_clear ? std::move(smooth) : std::move(raw);
  for (std::size_t k = 1; k < path_.size(); ++k)
    path_length_ += std::hypot(path_[k].x - path_[k - 1].x,
                               path_[k].y - path_[k - 1].y);
  if (kind == PlannerTarget::frontier) {
    last_frontier_ = target_point_;
    have_last_frontier_ = true;
  }
  return path_.size() >= 2;
}
} // namespace maze_mouse_navigation
