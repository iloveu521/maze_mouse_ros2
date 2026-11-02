#include "maze_mouse_navigation/left_wall.hpp"
#include "maze_mouse_navigation/local_costmap.hpp"
#include <algorithm>
#include <array>
#include <stdexcept>
namespace maze_mouse_navigation {
const char *wall_mode_name(WallMode mode) {
  switch (mode) {
  case WallMode::following:
    return "FOLLOW_LEFT_WALL";
  case WallMode::approaching:
    return "APPROACH_JUNCTION";
  case WallMode::left_turn:
    return "TURN_LEFT";
  case WallMode::right_turn:
    return "TURN_RIGHT";
  case WallMode::u_turn:
    return "U_TURN";
  case WallMode::exiting:
    return "EXITING";
  case WallMode::complete:
    return "COMPLETE";
  case WallMode::fault:
    return "FAULT";
  }
  return "FAULT";
}
LeftWallFollower::LeftWallFollower(WallParameters p) : p_(p) {
  const std::array positive{p.speed,
                            p.approach_speed,
                            p.cell_size,
                            p.wall_distance,
                            p.opening_distance,
                            p.front_turn_distance,
                            p.front_stop_distance,
                            p.side_stop_distance,
                            p.max_angular_speed,
                            p.acceleration,
                            p.deceleration,
                            p.heading_gain,
                            p.wall_gain,
                            p.goal_tolerance};
  if (!std::all_of(positive.begin(), positive.end(),
                   [](double v) { return std::isfinite(v) && v > 0; }) ||
      !std::isfinite(p.goal_x) || !std::isfinite(p.goal_y) ||
      !std::isfinite(p.goal_yaw) || !std::isfinite(p.angular_damping) ||
      p.angular_damping < 0 || p.opening_scans < 1 ||
      p.approach_speed > p.speed ||
      p.front_stop_distance >= p.front_turn_distance ||
      p.wall_distance >= p.opening_distance) {
    throw std::invalid_argument("Invalid left-wall parameters");
  }
}
WallCommand LeftWallFollower::stop() {
  previous_speed_ = 0;
  return {};
}
WallCommand LeftWallFollower::fail(const std::string &s) {
  mode_ = WallMode::fault;
  reason_ = s;
  return stop();
}
WallCommand LeftWallFollower::limit(double speed, double omega, double dt) {
  previous_speed_ =
      std::clamp(speed, std::max(0.0, previous_speed_ - p_.deceleration * dt),
                 std::min(p_.speed, previous_speed_ + p_.acceleration * dt));
  return {previous_speed_,
          std::clamp(omega, -p_.max_angular_speed, p_.max_angular_speed)};
}
void LeftWallFollower::request_turn(const WallPose &pose, int turns) {
  decision_x_ = static_cast<int>(std::floor(pose.x / p_.cell_size));
  decision_y_ = static_cast<int>(std::floor(pose.y / p_.cell_size));
  decision_locked_ = true;
  target_ = {(decision_x_ + 0.5) * p_.cell_size,
             (decision_y_ + 0.5) * p_.cell_size};
  path_ = {{pose.x, pose.y}, target_};
  turn_heading_ = wrap_angle(heading_ + turns * pi / 2);
  pending_turn_ = turns == 1
                      ? WallMode::left_turn
                      : (turns == -1 ? WallMode::right_turn : WallMode::u_turn);
  if (turns == 1)
    ++left_turns_;
  else if (turns == -1)
    ++right_turns_;
  else
    ++u_turns_;
  mode_ = WallMode::approaching;
  stable_ticks_ = 0;
  open_scans_ = 0;
  action_seconds_ = 0;
}
WallCommand LeftWallFollower::step(const WallPose &pose, const Sectors &s,
                                   std::uint64_t id, double dt, bool fresh) {
  if (!fresh || !s.usable() || !std::isfinite(pose.x) ||
      !std::isfinite(pose.y) || !std::isfinite(pose.yaw) ||
      !std::isfinite(pose.angular_speed) || !std::isfinite(pose.linear_speed)) {
    stable_ticks_ = 0;
    open_scans_ = 0;
    last_scan_ = id;
    return stop();
  }
  dt = std::clamp(dt, 0.001, 0.1);
  if (mode_ == WallMode::complete || mode_ == WallMode::fault)
    return stop();
  if (!initialized_) {
    heading_ = std::round(pose.yaw / (pi / 2)) * (pi / 2);
    initialized_ = true;
  }
  const bool new_scan = id != last_scan_;
  last_scan_ = id;
  if (awaiting_scan_) {
    if (id == required_scan_)
      return stop();
    awaiting_scan_ = false;
  }
  const double goal_distance =
      std::hypot(p_.goal_x - pose.x, p_.goal_y - pose.y);
  if (goal_distance <= p_.goal_tolerance &&
      std::abs(wrap_angle(pose.yaw - p_.goal_yaw)) < 0.5) {
    mode_ = WallMode::complete;
    reason_ = "Reached exit target";
    return stop();
  }
  const double goal_lateral = -(p_.goal_x - pose.x) * std::sin(p_.goal_yaw) +
                              (p_.goal_y - pose.y) * std::cos(p_.goal_yaw);
  if (mode_ == WallMode::following && goal_distance < 0.75 &&
      std::abs(goal_lateral) < 0.25 &&
      std::abs(wrap_angle(pose.yaw - p_.goal_yaw)) < 0.25) {
    mode_ = WallMode::exiting;
    path_.clear();
  }
  if (s.front.minimum_range < p_.front_stop_distance ||
      s.left.minimum_range < p_.side_stop_distance ||
      s.right.minimum_range < p_.side_stop_distance)
    return fail("Measured obstacle clearance below stop distance");
  if (mode_ == WallMode::approaching) {
    action_seconds_ += dt;
    if (action_seconds_ > 15)
      return fail("Junction approach timeout");
    const double dx = target_.x - pose.x, dy = target_.y - pose.y;
    const double along = dx * std::cos(heading_) + dy * std::sin(heading_);
    const double lateral = -dx * std::sin(heading_) + dy * std::cos(heading_);
    if (std::abs(lateral) > 0.22 || along < -0.15)
      return fail("Junction alignment error too large");
    if (along <= 0.045) {
      if (std::abs(pose.linear_speed) < 0.025 &&
          std::abs(pose.angular_speed) < 0.05) {
        if (++stable_ticks_ >= 5) {
          mode_ = pending_turn_;
          stable_ticks_ = 0;
          action_seconds_ = 0;
        }
      } else
        stable_ticks_ = 0;
      return stop();
    }
    stable_ticks_ = 0;
    return limit(std::min(p_.approach_speed, along),
                 p_.heading_gain * wrap_angle(heading_ - pose.yaw) +
                     1.8 * lateral - p_.angular_damping * pose.angular_speed,
                 dt);
  }
  if (mode_ == WallMode::left_turn || mode_ == WallMode::right_turn ||
      mode_ == WallMode::u_turn) {
    action_seconds_ += dt;
    if (action_seconds_ > 25)
      return fail("Junction turn timeout");
    const double error = wrap_angle(turn_heading_ - pose.yaw);
    if (std::abs(error) < 0.04 && std::abs(pose.angular_speed) < 0.05 &&
        std::abs(pose.linear_speed) < 0.025) {
      if (++stable_ticks_ >= 5) {
        heading_ = turn_heading_;
        mode_ = WallMode::following;
        path_.clear();
        awaiting_scan_ = true;
        required_scan_ = id;
      }
      return stop();
    }
    stable_ticks_ = 0;
    previous_speed_ = 0;
    return {0, std::clamp(p_.heading_gain * error -
                              p_.angular_damping * pose.angular_speed,
                          -p_.max_angular_speed, p_.max_angular_speed)};
  }
  if (mode_ == WallMode::exiting) {
    const double alpha = wrap_angle(
        std::atan2(p_.goal_y - pose.y, p_.goal_x - pose.x) - pose.yaw);
    return limit(std::min(p_.speed, goal_distance),
                 1.6 * alpha - p_.angular_damping * pose.angular_speed, dt);
  }
  const int x = static_cast<int>(std::floor(pose.x / p_.cell_size)),
            y = static_cast<int>(std::floor(pose.y / p_.cell_size));
  if (decision_locked_ && (x != decision_x_ || y != decision_y_)) {
    decision_locked_ = false;
    open_scans_ = 0;
  }
  const bool left_open = s.left.distance > p_.opening_distance;
  if (new_scan)
    open_scans_ = left_open && !decision_locked_ ? open_scans_ + 1 : 0;
  if (!decision_locked_ && open_scans_ >= p_.opening_scans) {
    request_turn(pose, 1);
    return limit(p_.approach_speed, 0, dt);
  }
  const bool front_blocked = s.front.distance < p_.front_turn_distance;
  if (!decision_locked_ && !left_open && front_blocked) {
    if (s.right.distance > p_.opening_distance) {
      request_turn(pose, -1);
      return limit(p_.approach_speed, 0, dt);
    }
    const double remaining =
        ((x + 0.5) * p_.cell_size - pose.x) * std::cos(heading_) +
        ((y + 0.5) * p_.cell_size - pose.y) * std::sin(heading_);
    if (remaining < 0.20) {
      request_turn(pose, 2);
      return limit(p_.approach_speed, 0, dt);
    }
  }
  const double error = wrap_angle(heading_ - pose.yaw);
  const double wall_error = left_open ? 0 : s.left.distance - p_.wall_distance;
  const double omega = p_.heading_gain * error +
                       std::clamp(p_.wall_gain * wall_error, -0.25, 0.25) -
                       p_.angular_damping * pose.angular_speed;
  double speed = (left_open || front_blocked) ? p_.approach_speed : p_.speed;
  if (std::abs(error) > 0.25)
    speed = std::min(speed, 0.10);
  speed = std::min(speed,
                   std::sqrt(2 * p_.deceleration *
                             std::max(0.0, s.front.distance -
                                               p_.front_stop_distance - 0.15)));
  return limit(speed, omega, dt);
}
} // namespace maze_mouse_navigation

namespace maze_mouse_navigation {
namespace {
double distance(WallPoint a, WallPoint b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}
WallPoint local_to_world(const WallPose &pose, double x, double y) {
  return {pose.x + std::cos(pose.yaw) * x - std::sin(pose.yaw) * y,
          pose.y + std::sin(pose.yaw) * x + std::cos(pose.yaw) * y};
}
} // namespace

const char *wall_path_mode_name(WallPathMode mode) {
  switch (mode) {
  case WallPathMode::waiting:
    return "WAITING";
  case WallPathMode::straight:
    return "FOLLOW_STRAIGHT";
  case WallPathMode::left_arc:
    return "FOLLOW_LEFT_ARC";
  case WallPathMode::right_arc:
    return "FOLLOW_RIGHT_ARC";
  case WallPathMode::u_turn_arc:
    return "FOLLOW_U_TURN_ARC";
  case WallPathMode::spin_uturn:
    return "SPIN_U_TURN";
  case WallPathMode::blocked:
    return "BLOCKED";
  case WallPathMode::exiting:
    return "EXITING";
  case WallPathMode::complete:
    return "COMPLETE";
  }
  return "BLOCKED";
}

WallPathFollower::WallPathFollower(WallPathParameters p) : p_(p) {
  const std::array positive{
      p.speed, p.wall_distance, p.lookahead, p.lookahead_gain,
      p.max_lookahead, p.turn_speed, p.min_turn_radius, p.max_turn_radius,
      p.max_lateral_acceleration, p.linear_acceleration, p.linear_deceleration,
      p.max_angular_speed, p.angular_acceleration, p.heading_gain,
      p.lateral_gain, p.spin_clearance, p.goal_tolerance};
  if (!std::all_of(positive.begin(), positive.end(), [](double v) {
        return std::isfinite(v) && v > 0;
      }) ||
      !std::isfinite(p.angular_damping) || p.angular_damping < 0 ||
      p.max_lookahead < p.lookahead ||
      p.max_turn_radius < p.min_turn_radius || !std::isfinite(p.goal_x) ||
      !std::isfinite(p.goal_y))
    throw std::invalid_argument("Invalid left-wall path parameters");
}

bool WallPathFollower::left_wall_continues(const WallPose &pose,
                                           const LocalCostmap &map) const {
  for (const auto &wall : map.guide_walls()) {
    const double dx = wall.b.x - wall.a.x, dy = wall.b.y - wall.a.y;
    const double length = std::hypot(dx, dy);
    if (length < 0.25)
      continue;
    double ux = dx / length, uy = dy / length;
    double alignment = ux * std::cos(pose.yaw) + uy * std::sin(pose.yaw);
    if (alignment < 0) {
      ux = -ux;
      uy = -uy;
      alignment = -alignment;
    }
    if (alignment < std::cos(0.35))
      continue;
    const double lateral =
        -(wall.a.x - pose.x) * std::sin(pose.yaw) +
        (wall.a.y - pose.y) * std::cos(pose.yaw);
    const double ta = (wall.a.x - pose.x) * std::cos(pose.yaw) +
                      (wall.a.y - pose.y) * std::sin(pose.yaw);
    const double tb = (wall.b.x - pose.x) * std::cos(pose.yaw) +
                      (wall.b.y - pose.y) * std::sin(pose.yaw);
    if (lateral > p_.wall_distance + 0.12 && lateral < 1.0 &&
        std::min(ta, tb) <= 0.15 && std::max(ta, tb) >= 0.65)
      return true;
  }
  return false;
}

bool WallPathFollower::collision_free(const std::vector<WallPoint> &path,
                                     const LocalCostmap &map) const {
  if (path.size() < 2)
    return false;
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (!map.traversable(path[i].x, path[i].y) ||
        (i > 0 && !map.line_clear(path[i - 1], path[i])))
      return false;
  }
  return true;
}

std::vector<WallPoint>
WallPathFollower::make_straight(const WallPose &pose,
                               const LocalCostmap &map) const {
  // With no continuous left wall, preserve the current corridor track. Do not
  // invent a large lateral shift at openings.
  double lateral = 0.0;
  double best = 1e9;
  for (const auto &wall : map.guide_walls()) {
    const double dx = wall.b.x - wall.a.x, dy = wall.b.y - wall.a.y;
    const double length = std::hypot(dx, dy);
    if (length < 0.30)
      continue;
    double ux = dx / length, uy = dy / length;
    double align = ux * std::cos(pose.yaw) + uy * std::sin(pose.yaw);
    if (align < 0) {
      ux = -ux;
      uy = -uy;
      align = -align;
    }
    if (align < std::cos(0.35))
      continue;
    const double side = -(wall.a.x - pose.x) * std::sin(pose.yaw) +
                        (wall.a.y - pose.y) * std::cos(pose.yaw);
    const double ta = (wall.a.x - pose.x) * std::cos(pose.yaw) +
                      (wall.a.y - pose.y) * std::sin(pose.yaw);
    const double tb = (wall.b.x - pose.x) * std::cos(pose.yaw) +
                      (wall.b.y - pose.y) * std::sin(pose.yaw);
    if (side <= map.hard_radius() + 0.04 || side > 1.0 ||
        std::min(ta, tb) > 0.15 || std::max(ta, tb) < 0.55)
      continue;
    if (side < best) {
      best = side;
      lateral = side - p_.wall_distance;
    }
  }
  // A long preview can extend through a nearby partition even when the next
  // corridor section is open. Keep the longest collision-free local segment.
  for (double horizon : {1.6, 1.2, 0.8, 0.55}) {
    std::vector<WallPoint> path;
    for (double x = 0; x <= horizon; x += 0.04) {
      const double blend = std::clamp(x / 0.45, 0.0, 1.0);
      const double smooth = blend * blend * (3.0 - 2.0 * blend);
      path.push_back(local_to_world(pose, x, lateral * smooth));
    }
    if (collision_free(path, map))
      return path;
  }
  return {};
}

bool WallPathFollower::make_arc(const WallPose &pose, const LocalCostmap &map,
                                int direction, double angle,
                                std::vector<WallPoint> &path) const {
  path.clear();
  const int intervals = std::max(16, static_cast<int>(std::ceil(angle * 0.32 / 0.035)));
  for (double radius = p_.max_turn_radius; radius >= p_.min_turn_radius - 1e-9;
       radius -= 0.025) {
    std::vector<WallPoint> candidate;
    for (int i = 0; i <= intervals; ++i) {
      const double a = angle * i / intervals;
      const double x = radius * std::sin(a);
      const double y = direction * radius * (1 - std::cos(a));
      candidate.push_back(local_to_world(pose, x, y));
    }
    const double end_yaw = pose.yaw + direction * angle;
    for (double d = 0.035; d <= 0.21; d += 0.035)
      candidate.push_back({candidate.back().x + d * std::cos(end_yaw),
                           candidate.back().y + d * std::sin(end_yaw)});
    if (collision_free(candidate, map)) {
      path = std::move(candidate);
      return true;
    }
  }
  return false;
}

bool WallPathFollower::choose_maneuver(const WallPose &pose,
                                       const LocalCostmap &map) {
  const WallPoint start{pose.x, pose.y};
  WallPoint forward{pose.x + 0.42 * std::cos(pose.yaw),
                    pose.y + 0.42 * std::sin(pose.yaw)};
  const bool forward_open = map.line_clear(start, forward);
  const double left_range = map.scan_geometry().sectors.left.distance;
  const bool left_opening =
      std::isfinite(left_range) && left_range > p_.wall_distance + 0.32;
  std::vector<WallPoint> left, right;
  const bool left_open = make_arc(pose, map, 1, pi / 2, left);
  const bool right_open = make_arc(pose, map, -1, pi / 2, right);
  left_opening_ = left_opening;
  left_arc_available_ = left_open;
  forward_available_ = forward_open;

  // A long-range reading in the 60-degree left sector is the junction cue.
  // Use it before the wall-segment heuristic so the robot does not pass an
  // opening simply because the fitted wall has a small noisy continuation.
  if (left_opening && left_open) {
    path_ = std::move(left);
    mode_ = WallPathMode::left_arc;
    ++left_turns_;
    return true;
  }
  if (left_opening && !left_open) {
    // The side sector sees the opening, but the full curved primitive is not
    // collision-checked yet. Advance in a short, checked segment to expose the
    // corner from a better scan viewpoint, then classify the opening again.
    if (map.scan_geometry().sectors.front.distance > map.hard_radius() + 0.12) {
      for (double distance_ahead : {0.20, 0.14, 0.08}) {
        path_ = {start,
                 {pose.x + distance_ahead * std::cos(pose.yaw),
                  pose.y + distance_ahead * std::sin(pose.yaw)}};
        if (collision_free(path_, map)) {
          mode_ = WallPathMode::straight;
          return true;
        }
      }
    }
    mode_ = WallPathMode::waiting;
    path_.clear();
    return false;
  }
  if (forward_open && left_wall_continues(pose, map)) {
    path_ = make_straight(pose, map);
    mode_ = path_.empty() ? WallPathMode::blocked : WallPathMode::straight;
    return !path_.empty();
  }

  if (!left_wall_continues(pose, map) && left_open) {
    path_ = std::move(left);
    mode_ = WallPathMode::left_arc;
    ++left_turns_;
    return true;
  }
  if (forward_open) {
    path_ = make_straight(pose, map);
    mode_ = path_.empty() ? WallPathMode::blocked : WallPathMode::straight;
    return !path_.empty();
  }
  if (right_open) {
    path_ = std::move(right);
    mode_ = WallPathMode::right_arc;
    ++right_turns_;
    return true;
  }

  std::vector<WallPoint> uturn;
  if (make_arc(pose, map, 1, pi, uturn) ||
      make_arc(pose, map, -1, pi, uturn)) {
    path_ = std::move(uturn);
    mode_ = WallPathMode::u_turn_arc;
    ++arc_uturns_;
    return true;
  }
  if (map.clearance(pose.x, pose.y) >
      map.hard_radius() + p_.spin_clearance) {
    spin_target_yaw_ = wrap_angle(pose.yaw + pi);
    path_ = {start, local_to_world(pose, 0.35, 0)};
    mode_ = WallPathMode::spin_uturn;
    ++spin_uturns_;
    return true;
  }
  mode_ = WallPathMode::blocked;
  path_.clear();
  return false;
}

WallCommand WallPathFollower::spin_in_place(const WallPose &pose,
                                            const LocalCostmap &map,
                                            double dt) {
  target_speed_ = 0;
  if (map.clearance(pose.x, pose.y) <= map.hard_radius()) {
    mode_ = WallPathMode::blocked;
    path_.clear();
    previous_angular_ = 0;
    previous_speed_ = 0;
    target_speed_ = 0;
    return {};
  }
  const double error = wrap_angle(spin_target_yaw_ - pose.yaw);
  if (std::abs(error) < 0.035 && std::abs(pose.angular_speed) < 0.06) {
    mode_ = WallPathMode::waiting;
    path_.clear();
    previous_angular_ = 0;
    previous_speed_ = 0;
    return {};
  }
  const double desired = std::clamp(p_.heading_gain * error -
                                        p_.angular_damping * pose.angular_speed,
                                    -p_.max_angular_speed,
                                    p_.max_angular_speed);
  target_angular_ = std::clamp(
      desired, previous_angular_ - p_.angular_acceleration * dt,
      previous_angular_ + p_.angular_acceleration * dt);
  previous_angular_ = target_angular_;
  return {0, target_angular_};
}

WallCommand WallPathFollower::track_path(const WallPose &pose,
                                        const LocalCostmap &map, double dt) {
  if (path_.size() < 2)
    return {};
  bool remaining_path_clear =
      map.clearance(pose.x, pose.y) > map.hard_radius();
  const std::size_t validation_end =
      std::min(path_.size(), progress_index_ + 7);
  for (std::size_t i = progress_index_; remaining_path_clear &&
                                      i < validation_end;
       ++i) {
    remaining_path_clear = map.traversable(path_[i].x, path_[i].y) &&
                           (i == progress_index_ ||
                            map.line_clear(path_[i - 1], path_[i]));
  }
  if (!remaining_path_clear) {
    mode_ = WallPathMode::blocked;
    path_.clear();
    previous_speed_ = previous_angular_ = 0;
    return {};
  }
  const std::size_t begin = progress_index_ > 2 ? progress_index_ - 2 : 0;
  const std::size_t end = std::min(path_.size(), progress_index_ + 19);
  std::size_t nearest = progress_index_;
  double nearest_distance = 1e9;
  for (std::size_t i = begin; i < end; ++i) {
    const double d = distance({pose.x, pose.y}, path_[i]);
    if (d < nearest_distance) {
      nearest_distance = d;
      nearest = i;
    }
  }
  progress_index_ = std::max(progress_index_, nearest);
  cross_track_error_ = nearest_distance;
  if (nearest_distance > 0.38) {
    mode_ = WallPathMode::blocked;
    path_.clear();
    return {};
  }
  double remaining = 0;
  for (std::size_t i = progress_index_ + 1; i < path_.size(); ++i)
    remaining += distance(path_[i - 1], path_[i]);
  if (remaining < 0.10 && nearest_distance < 0.14) {
    path_.clear();
    progress_index_ = 0;
    mode_ = WallPathMode::waiting;
    previous_angular_ = 0;
    previous_speed_ = 0;
    return {};
  }

  const double lookahead = std::clamp(p_.lookahead +
                                          p_.lookahead_gain *
                                              std::max(0.0, pose.linear_speed),
                                      p_.lookahead, p_.max_lookahead);
  const double tracking_lookahead =
      mode_ == WallPathMode::left_arc || mode_ == WallPathMode::right_arc ||
              mode_ == WallPathMode::u_turn_arc
          ? std::min(lookahead, std::clamp(remaining * 0.40, 0.16, 0.32))
          : lookahead;
  std::size_t target_index = path_.size() - 1;
  double accumulated = 0;
  for (std::size_t i = progress_index_ + 1; i < path_.size(); ++i) {
    const double segment = distance(path_[i - 1], path_[i]);
    if (accumulated + segment >= tracking_lookahead) {
      target_index = i;
      break;
    }
    accumulated += segment;
  }
  const std::size_t tangent_index = std::clamp<std::size_t>(
      target_index, 1, path_.size() - 2);
  const WallPoint before = path_[tangent_index - 1];
  const WallPoint center = path_[tangent_index];
  const WallPoint after = path_[tangent_index + 1];
  const double before_yaw = std::atan2(center.y - before.y, center.x - before.x);
  const double after_yaw = std::atan2(after.y - center.y, after.x - center.x);
  const double ds = 0.5 * (distance(before, center) + distance(center, after));
  const double curvature = wrap_angle(after_yaw - before_yaw) /
                           std::max(ds, 1e-4);
  const double tangent_yaw = wrap_angle(
      before_yaw + 0.5 * wrap_angle(after_yaw - before_yaw));
  const double cross_track =
      -std::sin(tangent_yaw) * (pose.x - path_[nearest].x) +
      std::cos(tangent_yaw) * (pose.y - path_[nearest].y);
  const double heading_error = wrap_angle(tangent_yaw - pose.yaw);

  double speed = mode_ == WallPathMode::left_arc ||
                         mode_ == WallPathMode::right_arc ||
                         mode_ == WallPathMode::u_turn_arc
                     ? p_.turn_speed
                     : p_.speed;
  speed = std::min(speed, std::sqrt(p_.max_lateral_acceleration /
                                   std::max(0.02, std::abs(curvature))));
  speed = std::min(speed, std::sqrt(2 * p_.linear_deceleration *
                                    std::max(0.0, remaining - 0.10)));
  speed *= std::clamp(std::cos(heading_error), 0.3, 1.0);
  if (nearest_distance > 0.20)
    speed = std::min(speed, 0.25);

  const double desired_angular = std::clamp(
      speed * curvature + p_.heading_gain * heading_error -
          p_.lateral_gain * cross_track -
          p_.angular_damping * pose.angular_speed,
      -p_.max_angular_speed, p_.max_angular_speed);
  target_speed_ = speed;
  target_angular_ = std::clamp(
      desired_angular, previous_angular_ - p_.angular_acceleration * dt,
      previous_angular_ + p_.angular_acceleration * dt);
  const double acceleration = speed < previous_speed_
                                  ? p_.linear_deceleration
                                  : p_.linear_acceleration;
  const double commanded_speed = std::clamp(
      speed, std::max(0.0, previous_speed_ - acceleration * dt),
      previous_speed_ + acceleration * dt);
  // Track the already collision-checked path directly. Re-rollout with a
  // constant curvature predicted an unplanned continuation through the next
  // junction and repeatedly vetoed otherwise valid turn primitives.
  previous_speed_ = commanded_speed;
  previous_angular_ = target_angular_;
  return {commanded_speed, target_angular_};
}

WallCommand WallPathFollower::step(const WallPose &pose,
                                   const LocalCostmap &map, double dt,
                                   bool fresh) {
  if (!fresh || !map.ready() || !std::isfinite(pose.x) ||
      !std::isfinite(pose.y) || !std::isfinite(pose.yaw) ||
      !std::isfinite(pose.linear_speed) || !std::isfinite(pose.angular_speed)) {
    target_speed_ = target_angular_ = previous_angular_ = previous_speed_ = 0;
    if (mode_ != WallPathMode::complete && path_.empty())
      mode_ = WallPathMode::waiting;
    return {};
  }
  dt = std::clamp(dt, 0.01, 0.10);
  if (mode_ == WallPathMode::complete)
    return {};
  if (std::hypot(p_.goal_x - pose.x, p_.goal_y - pose.y) <= p_.goal_tolerance) {
    mode_ = WallPathMode::complete;
    path_.clear();
    target_speed_ = target_angular_ = 0;
    return {};
  }
  if (mode_ == WallPathMode::spin_uturn)
    return spin_in_place(pose, map, dt);
  if (mode_ == WallPathMode::blocked) {
    path_.clear();
    progress_index_ = 0;
    mode_ = WallPathMode::waiting;
  }
  if (path_.empty() && !choose_maneuver(pose, map))
    return {};
  if (mode_ == WallPathMode::spin_uturn)
    return spin_in_place(pose, map, dt);
  return track_path(pose, map, dt);
}
} // namespace maze_mouse_navigation
