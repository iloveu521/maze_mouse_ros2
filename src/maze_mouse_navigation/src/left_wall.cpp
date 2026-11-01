#include "maze_mouse_navigation/left_wall.hpp"
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
