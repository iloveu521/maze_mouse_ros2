#pragma once
#include "maze_mouse_navigation/left_wall.hpp"
#include "maze_mouse_navigation/scan_geometry.hpp"
#include <cstdint>
#include <span>
#include <vector>

namespace maze_mouse_navigation {
struct CostmapParameters {
  double resolution{0.05}, size{4.0};
  double robot_radius{0.14}, safety_margin{0.10}, soft_radius{0.55};
  double laser_x{-0.032}, laser_y{0}, laser_yaw{0};
  double side_sector_degrees{60}, front_sector_degrees{90};
  double minimum_wall_length{0.24}, wall_fit_tolerance{0.025};
};

class LocalCostmap {
public:
  explicit LocalCostmap(CostmapParameters parameters = {});
  void configure(CostmapParameters parameters);
  bool update(const WallPose &pose, std::span<const float> ranges,
              double angle_min, double increment, double range_min,
              double range_max);
  bool ready() const { return ready_; }
  double clearance(double x, double y) const;
  bool traversable(double x, double y) const;
  bool line_clear(WallPoint a, WallPoint b) const;
  std::vector<WallPoint> route(WallPoint start, WallPoint goal) const;
  std::vector<WallPoint> boundary_path(const WallPose &pose, double offset,
                                       WallPoint &anchor,
                                       bool &have_anchor) const;
  std::vector<std::int8_t> raw_data() const;
  std::vector<std::int8_t> cost_data() const;
  int width() const { return n_; }
  double resolution() const { return p_.resolution; }
  double origin_x() const { return ox_; }
  double origin_y() const { return oy_; }
  double hard_radius() const { return p_.robot_radius + p_.safety_margin; }
  const ScanGeometry &scan_geometry() const { return scan_geometry_; }
  const std::vector<WallSegment> &guide_walls() const { return guide_walls_; }

private:
  int index(double x, double y) const;
  void shift(double x, double y);
  void trace(double ax, double ay, double bx, double by,
             std::vector<bool> &mask) const;
  void inflate();
  CostmapParameters p_;
  int n_{};
  double ox_{}, oy_{};
  bool ready_{};
  std::vector<std::int8_t> odds_;
  std::vector<bool> seen_;
  std::vector<double> distance_;
  std::vector<double> guide_distance_;
  ScanGeometry scan_geometry_;
  std::vector<WallSegment> guide_walls_;
};

struct LocalPlannerParameters {
  double max_speed{0.5}, max_angular{0.9};
  int linear_samples{6}, angular_samples{30};
  double acceleration{0.8}, deceleration{1.0}, angular_acceleration{1.5};
  double horizon{1.5}, reaction_time{0.20}, wall_distance{0.30};
  double target_lookahead{0.85}, target_tolerance{0.18}, wall_offset{0.10};
  double recovery_enter{0.04}, recovery_exit{0.075}, recovery_speed{0.15};
  double heading_stop_angle{0.8};
  double progress_weight{3.0}, path_weight{2.0}, heading_weight{0.8};
  double clearance_weight{4.0}, speed_weight{0.5};
  double pursuit_lookahead{0.60}, pursuit_lookahead_gain{0.35};
  double pursuit_lookahead_max{0.90}, path_preview_distance{1.20};
  double path_refresh_distance{0.90}, path_validation_distance{1.50};
  double heading_kp{0.75}, heading_ki{0.02}, heading_kd{0.16};
  double lateral_gain{0.40}, curvature_feedforward{1.0};
  double max_lateral_acceleration{0.65}, angular_command_acceleration{2.5};
  double angular_filter_time_constant{0.18};
  double pursuit_weight{4.0};
  double dead_end_distance{1.15}, dead_end_front_gap{0.45};
  double u_turn_speed{0.22}, u_turn_lookahead{0.20}, u_turn_horizon{0.50};
  double goal_x{6.45}, goal_y{5.5}, goal_tolerance{0.14};
};
enum class LocalMode {
  following,
  recovering,
  u_turn,
  exiting,
  complete,
  blocked
};
const char *local_mode_name(LocalMode mode);
class CostmapFollower {
public:
  explicit CostmapFollower(LocalPlannerParameters parameters = {});
  void configure(LocalPlannerParameters parameters);
  WallCommand step(const WallPose &pose, const Sectors &sectors,
                   const LocalCostmap &map, double dt, bool fresh);
  const std::vector<WallPoint> &trajectory() const { return best_path_; }
  LocalMode mode() const { return mode_; }
  double desired_yaw() const { return desired_yaw_; }
  double predicted_clearance() const { return predicted_clearance_; }
  const std::vector<WallPoint> &reference() const { return reference_; }
  int moving_candidates() const { return moving_candidates_; }
  int dead_ends() const { return dead_ends_; }
  double dead_end_front_distance() const { return dead_end_front_distance_; }
  double navigation_heading() const { return guide_yaw_; }
  double tracking_error() const { return tracking_error_; }
  double control_lookahead() const { return control_lookahead_; }
  double target_angular() const { return target_angular_; }

private:
  std::vector<WallPoint> wall_reference(const WallPose &, const LocalCostmap &);
  std::vector<WallPoint> dead_end_path(const WallPose &, const LocalCostmap &);
  std::vector<WallPoint> refresh_reference(std::vector<WallPoint> candidate,
                                           const WallPose &pose,
                                           const LocalCostmap &map);
  LocalPlannerParameters p_;
  LocalMode mode_{LocalMode::following};
  double desired_yaw_{}, predicted_clearance_{};
  WallPoint anchor_{};
  bool have_anchor_{};
  bool recovering_{};
  std::vector<WallPoint> best_path_;
  std::vector<WallPoint> reference_;
  std::vector<WallPoint> cached_reference_;
  LocalMode cached_mode_{LocalMode::blocked};
  int moving_candidates_{};
  std::vector<WallPoint> turn_path_;
  std::size_t turn_index_{};
  int dead_ends_{};
  double dead_end_front_distance_{};
  double guide_yaw_{};
  bool have_guide_yaw_{};
  double tracking_error_{}, control_lookahead_{}, target_angular_{};
  double heading_integral_{}, previous_tracking_error_{}, filtered_derivative_{};
  double previous_command_angular_{};
  double filtered_target_angular_{};
  bool have_control_error_{};
};
} // namespace maze_mouse_navigation
