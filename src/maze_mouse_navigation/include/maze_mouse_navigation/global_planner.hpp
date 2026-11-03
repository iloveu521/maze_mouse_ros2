#pragma once
#include "maze_mouse_navigation/local_costmap.hpp"
#include <vector>

namespace maze_mouse_navigation {
struct GlobalPlannerParameters {
  double replan_period{0.5};
  double frontier_weight{2.5};
  double frontier_min_distance{0.4};
  double frontier_switch_margin{0.3};
  double frontier_sticky_radius{0.6};
  double frontier_sticky_bonus{0.5};
  double start_snap_radius{0.4};
  double goal_snap_radius{0.12};
  double planning_margin{0.05};
  double clearance_cost_weight{5.0};
  double clearance_target{0.45};
  double path_min_remaining{0.4};
  double path_deviation{0.45};
  double path_validation_distance{1.0};
  double smoothing_span{1.5};
  double smoothing_clearance{0.32};
};
enum class PlannerTarget { none, goal, frontier };

class GlobalPlanner {
public:
  GlobalPlanner() = default;
  explicit GlobalPlanner(GlobalPlannerParameters p);
  void configure(GlobalPlannerParameters p);
  void invalidate() { dirty_ = true; }

  bool update(const WallPose &pose, const LocalCostmap &map, double dt,
              WallPoint goal, bool force = false);
  const std::vector<WallPoint> &path() const { return path_; }
  PlannerTarget target() const { return target_; }
  WallPoint target_point() const { return target_point_; }
  bool exhausted() const { return exhausted_; }
  int replans() const { return replans_; }
  int frontier_candidates() const { return frontier_candidates_; }
  double plan_ms() const { return plan_ms_; }
  double path_length() const { return path_length_; }
  double path_remaining(const WallPose &pose) const;
  double lateral_deviation(const WallPose &pose) const;

private:
  bool plan(const WallPose &pose, const LocalCostmap &map, WallPoint goal);
  GlobalPlannerParameters p_;
  std::vector<WallPoint> path_;
  PlannerTarget target_{PlannerTarget::none};
  WallPoint target_point_{};
  bool exhausted_{}, dirty_{true};
  double since_plan_{1e9}, plan_ms_{}, path_length_{};
  int replans_{}, frontier_candidates_{};
  WallPoint last_frontier_{};
  bool have_last_frontier_{};
};
} // namespace maze_mouse_navigation
