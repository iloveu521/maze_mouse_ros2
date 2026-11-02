#pragma once
#include "maze_mouse_navigation/core.hpp"
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
namespace maze_mouse_navigation {
class LocalCostmap;
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

// New left-wall strategy: junction decisions are made from the unified local
// costmap and executed as a latched path primitive. This is independent of the
// legacy cell-centre controller above.
struct WallPathParameters {
  double speed{0.5};
  double wall_distance{0.44};
  double lookahead{0.55};
  double lookahead_gain{0.35};
  double max_lookahead{0.90};
  double turn_speed{0.18};
  double min_turn_radius{0.14};
  double max_turn_radius{0.42};
  double max_lateral_acceleration{0.55};
  double linear_acceleration{0.8};
  double linear_deceleration{1.0};
  double max_angular_speed{1.8};
  double angular_acceleration{2.5};
  double heading_gain{1.2};
  double lateral_gain{2.0};
  double angular_damping{0.25};
  double spin_clearance{0.035};
  double goal_x{6.45}, goal_y{5.5}, goal_tolerance{0.14};
};
enum class WallPathMode {
  waiting,
  straight,
  left_arc,
  right_arc,
  u_turn_arc,
  spin_uturn,
  blocked,
  exiting,
  complete
};
const char *wall_path_mode_name(WallPathMode mode);

class WallPathFollower {
public:
  explicit WallPathFollower(WallPathParameters parameters = {});
  WallCommand step(const WallPose &pose, const LocalCostmap &map, double dt,
                  bool fresh);
  const std::vector<WallPoint> &path() const { return path_; }
  WallPathMode mode() const { return mode_; }
  double cross_track_error() const { return cross_track_error_; }
  double target_speed() const { return target_speed_; }
  double target_angular() const { return target_angular_; }
  bool left_opening() const { return left_opening_; }
  bool left_arc_available() const { return left_arc_available_; }
  bool forward_available() const { return forward_available_; }
  int left_turns() const { return left_turns_; }
  int right_turns() const { return right_turns_; }
  int arc_uturns() const { return arc_uturns_; }
  int spin_uturns() const { return spin_uturns_; }

private:
  bool left_wall_continues(const WallPose &, const LocalCostmap &) const;
  bool make_arc(const WallPose &, const LocalCostmap &, int direction,
                double angle, std::vector<WallPoint> &) const;
  std::vector<WallPoint> make_straight(const WallPose &,
                                       const LocalCostmap &) const;
  bool collision_free(const std::vector<WallPoint> &,
                      const LocalCostmap &) const;
  bool choose_maneuver(const WallPose &, const LocalCostmap &);
  WallCommand track_path(const WallPose &, const LocalCostmap &, double dt);
  WallCommand spin_in_place(const WallPose &, const LocalCostmap &, double dt);
  WallPathParameters p_;
  WallPathMode mode_{WallPathMode::waiting};
  std::vector<WallPoint> path_;
  std::size_t progress_index_{};
  double spin_target_yaw_{}, previous_angular_{}, previous_speed_{};
  double cross_track_error_{}, target_speed_{}, target_angular_{};
  bool left_opening_{}, left_arc_available_{}, forward_available_{};
  int left_turns_{}, right_turns_{}, arc_uturns_{}, spin_uturns_{};
};
} // namespace maze_mouse_navigation
