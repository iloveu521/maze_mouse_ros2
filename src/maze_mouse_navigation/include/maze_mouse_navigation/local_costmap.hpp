#pragma once
#include "maze_mouse_navigation/left_wall.hpp"
#include "maze_mouse_navigation/scan_geometry.hpp"
#include <algorithm>
#include <cstdint>
#include <span>
#include <vector>

namespace maze_mouse_navigation {
struct CostmapParameters {
  double resolution{0.05}, size{4.0};
  double origin_x{0.0}, origin_y{0.0};
  bool fixed{false};
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
  bool occupied(double x, double y) const;
  bool observed(double x, double y) const;
  double clearance(double x, double y) const;
  double tracking_clearance(double x, double y) const;
  bool traversable(double x, double y) const;
  bool line_clear(WallPoint a, WallPoint b) const;
  bool tracking_line_clear(WallPoint a, WallPoint b) const;
  double tracking_radius() const {
    return p_.robot_radius + std::min(0.01, p_.safety_margin);
  }
  std::vector<std::int8_t> raw_data() const;
  std::vector<std::int8_t> cost_data() const;
  int width() const { return n_; }
  int cells() const { return n_ * n_; }
  bool contains(double x, double y) const { return index(x, y) >= 0; }
  int cell_index(double x, double y) const { return index(x, y); }
  WallPoint cell_center(int cell) const {
    return {ox_ + (cell % n_ + 0.5) * p_.resolution,
            oy_ + (cell / static_cast<double>(n_) + 0.5) * p_.resolution};
  }
  double resolution() const { return p_.resolution; }
  double robot_radius() const { return p_.robot_radius; }
  double origin_x() const { return ox_; }
  double origin_y() const { return oy_; }
  double hard_radius() const { return p_.robot_radius + p_.safety_margin; }
  const ScanGeometry &scan_geometry() const { return scan_geometry_; }

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
  ScanGeometry scan_geometry_;
};

struct LocalPlannerParameters {
  // Speed and motion model limits for the sampler/predictor.
  double max_speed{0.5}, max_angular{0.9};        // command limits
  int linear_samples{6}, angular_samples{30};     // sample counts
  double acceleration{0.8}, deceleration{1.0},    // predicted accel/decel
      angular_acceleration{1.5};                  // predicted angular accel
  double horizon{1.5}, reaction_time{0.20};       // preview / latency
  double heading_stop_angle{0.8};
  double progress_weight{3.0}, path_weight{2.0}, heading_weight{0.8};
  double clearance_weight{4.0}, speed_weight{0.5}; // speed preference weight
  // Pure Pursuit lookahead: base (m), speed gain (s), lower/upper limits.
  double pursuit_lookahead{0.35}, pursuit_lookahead_gain{0.15};
  double pursuit_lookahead_max{0.55}, path_preview_distance{1.20};
  double path_validation_distance{1.50}, path_deviation{0.35};
  double heading_kp{0.75};
  double curvature_feedforward{1.0};
  // Lateral acceleration caps corner speed; angular slew caps steering changes.
  double max_lateral_acceleration{0.65}, angular_command_acceleration{2.5};
  double pursuit_weight{4.0};
  double goal_x{6.45}, goal_y{5.5}, goal_tolerance{0.14};
};
enum class LocalMode { following, complete, blocked };
const char *local_mode_name(LocalMode mode);
class CostmapFollower {
public:
  explicit CostmapFollower(LocalPlannerParameters parameters = {});
  void configure(LocalPlannerParameters parameters);
  WallCommand step(const WallPose &pose, const Sectors &sectors,
                   const LocalCostmap &map, double dt, bool fresh,
                   const std::vector<WallPoint> &planned_path);
  const std::vector<WallPoint> &trajectory() const { return best_path_; }
  LocalMode mode() const { return mode_; }
  double desired_yaw() const { return desired_yaw_; }
  double predicted_clearance() const { return predicted_clearance_; }
  const std::vector<WallPoint> &reference() const { return reference_; }
  int moving_candidates() const { return moving_candidates_; }
  double tracking_error() const { return tracking_error_; }
  double control_lookahead() const { return control_lookahead_; }
  double target_angular() const { return target_angular_; }
  int reference_update_code() const { return reference_update_code_; }
  double reference_remaining() const { return reference_remaining_; }
  std::size_t reference_points() const { return reference_.size(); }
  std::size_t candidate_points() const { return reference_candidate_points_; }

private:
  std::vector<WallPoint> refresh_reference(std::vector<WallPoint> candidate,
                                           const WallPose &pose,
                                           const LocalCostmap &map);
  LocalPlannerParameters p_;
  LocalMode mode_{LocalMode::following};
  double desired_yaw_{}, predicted_clearance_{};
  std::vector<WallPoint> best_path_;
  std::vector<WallPoint> reference_;
  std::vector<WallPoint> last_reference_;
  int moving_candidates_{};
  double tracking_error_{}, control_lookahead_{}, target_angular_{};
  double reference_remaining_{};
  std::size_t reference_candidate_points_{};
  int reference_update_code_{};
  double heading_integral_{}, previous_tracking_error_{},
      filtered_derivative_{};
  double previous_command_angular_{};
  double filtered_target_angular_{};
  bool have_control_error_{};
  int alignment_turn_direction_{};
  int recovery_direction_{};
};
} // namespace maze_mouse_navigation
