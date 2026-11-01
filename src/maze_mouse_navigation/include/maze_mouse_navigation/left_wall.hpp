#pragma once
#include "maze_mouse_navigation/core.hpp"
#include <cstdint>
#include <string>
#include <vector>
namespace maze_mouse_navigation {
struct WallPose {
  double x{}, y{}, yaw{}, angular_speed{}, linear_speed{};
};
struct WallPoint {
  double x{}, y{};
};
struct WallParameters {
  double speed{0.5}, approach_speed{0.20}, cell_size{1.0};
  double wall_distance{0.40}, opening_distance{0.78}, front_turn_distance{0.82};
  double front_stop_distance{0.20}, side_stop_distance{0.14};
  double max_angular_speed{0.6}, acceleration{0.8}, deceleration{1.2};
  double heading_gain{1.6}, wall_gain{1.5}, angular_damping{0.25};
  double goal_x{6.45}, goal_y{5.5}, goal_yaw{0.0}, goal_tolerance{0.14};
  int opening_scans{2};
};
enum class WallMode {
  following,
  approaching,
  left_turn,
  right_turn,
  u_turn,
  exiting,
  complete,
  fault
};
const char *wall_mode_name(WallMode mode);
struct WallCommand {
  double linear{}, angular{};
};
class LeftWallFollower {
public:
  explicit LeftWallFollower(WallParameters parameters = {});
  WallCommand step(const WallPose &, const Sectors &, std::uint64_t scan_id,
                   double dt, bool fresh);
  WallMode mode() const { return mode_; }
  const std::vector<WallPoint> &local_path() const { return path_; }
  const std::string &reason() const { return reason_; }
  int left_turns() const { return left_turns_; }
  int right_turns() const { return right_turns_; }
  int u_turns() const { return u_turns_; }

private:
  void request_turn(const WallPose &, int quarter_turns);
  WallCommand limit(double speed, double omega, double dt);
  WallCommand stop();
  WallCommand fail(const std::string &);
  WallParameters p_;
  WallMode mode_{WallMode::following}, pending_turn_{WallMode::left_turn};
  bool initialized_{}, decision_locked_{}, awaiting_scan_{};
  double heading_{}, turn_heading_{}, previous_speed_{}, action_seconds_{};
  int decision_x_{}, decision_y_{}, open_scans_{}, stable_ticks_{};
  int left_turns_{}, right_turns_{}, u_turns_{};
  std::uint64_t last_scan_{}, required_scan_{};
  WallPoint target_{};
  std::vector<WallPoint> path_;
  std::string reason_;
};
} // namespace maze_mouse_navigation
