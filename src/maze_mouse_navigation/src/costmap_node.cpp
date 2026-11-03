#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "maze_mouse_navigation/global_planner.hpp"
#include "maze_mouse_navigation/local_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <csignal>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>

namespace maze_mouse_navigation {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace {
double yaw_of(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z));
}
std::string number(double v) {
  if (!std::isfinite(v))
    return "null";
  std::ostringstream s;
  s << std::setprecision(9) << v;
  return s.str();
}
const char *target_name(PlannerTarget target) {
  switch (target) {
  case PlannerTarget::goal:
    return "goal";
  case PlannerTarget::frontier:
    return "frontier";
  case PlannerTarget::none:
    return "none";
  }
  return "none";
}
} // namespace
class CostmapNode final : public rclcpp::Node {
public:
  CostmapNode() : Node("maze_navigator") {
    auto real = [this](const char *n, double d) {
      return declare_parameter<double>(n, d);
    };
    LocalPlannerParameters p;
    for (const auto &[name, field] : planner_fields_)
      p.*field = real(name.c_str(), p.*field);
    p.linear_samples =
        declare_parameter<int>("linear_samples", p.linear_samples);
    p.angular_samples =
        declare_parameter<int>("angular_samples", p.angular_samples);
    start_x_ = real("start_x", 0.5);
    start_y_ = real("start_y", 0.5);
    start_yaw_ = real("start_yaw", 0);
    timeout_ = real("data_timeout", 0.7);

    CostmapParameters m;
    m.size = real("costmap_size", m.size);
    m.resolution = real("costmap_resolution", m.resolution);
    const double default_origin_x =
        0.5 * (start_x_ + p.goal_x) - 0.5 * m.size;
    const double default_origin_y =
        0.5 * (start_y_ + p.goal_y) - 0.5 * m.size;
    for (const auto &[name, field] : map_fields_) {
      if (name == "costmap_size" || name == "costmap_resolution")
        continue;
      if (name == "costmap_origin_x")
        m.origin_x = real(name.c_str(), default_origin_x);
      else if (name == "costmap_origin_y")
        m.origin_y = real(name.c_str(), default_origin_y);
      else
        m.*field = real(name.c_str(), m.*field);
    }
    m.fixed = declare_parameter<bool>("costmap_fixed", false);
    map_ = std::make_unique<LocalCostmap>(m);
    if (m.fixed &&
        (!map_->contains(start_x_, start_y_) ||
         !map_->contains(p.goal_x, p.goal_y)))
      throw std::invalid_argument(
          "costmap_size/costmap_origin must cover start and goal");
    robot_radius_ = m.robot_radius;
    margin_ = m.safety_margin;

    GlobalPlannerParameters g;
    for (const auto &[name, field] : global_fields_)
      g.*field = real(name.c_str(), g.*field);
    global_ = std::make_unique<GlobalPlanner>(g);

    planner_ = std::make_unique<CostmapFollower>(p);
    map_parameters_ = m;
    planner_parameters_ = p;
    global_parameters_ = g;
    if (!std::isfinite(start_x_) || !std::isfinite(start_y_) ||
        !std::isfinite(start_yaw_) || !(timeout_ > 0) ||
        !std::isfinite(timeout_))
      throw std::invalid_argument("Invalid frame/scan parameters");
    parameter_callback_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          try {
            auto m = map_parameters_;
            auto p = planner_parameters_;
            auto g = global_parameters_;
            for (const auto &parameter : parameters) {
              const auto &name = parameter.get_name();
              if (name == "costmap_fixed")
                m.fixed = parameter.as_bool();
              else if (auto field = map_fields_.find(name);
                       field != map_fields_.end())
                m.*(field->second) = parameter.as_double();
              else if (auto field = planner_fields_.find(name);
                       field != planner_fields_.end())
                p.*(field->second) = parameter.as_double();
              else if (auto field = global_fields_.find(name);
                       field != global_fields_.end())
                g.*(field->second) = parameter.as_double();
              else if (name == "linear_samples")
                p.linear_samples = parameter.as_int();
              else if (name == "angular_samples")
                p.angular_samples = parameter.as_int();
              else if (has_parameter(name) &&
                       parameter.get_type() == get_parameter(name).get_type() &&
                       parameter.value_to_string() ==
                           get_parameter(name).value_to_string())
                continue;
              else
                throw std::invalid_argument(
                    "Parameter requires restarting node: " + name);
            }
            LocalCostmap checked_map(m);
            CostmapFollower checked_planner(p);
            GlobalPlanner checked_global(g);
            if (m.fixed &&
                (!checked_map.contains(start_x_, start_y_) ||
                 !checked_map.contains(p.goal_x, p.goal_y)))
              throw std::invalid_argument(
                  "costmap_size/costmap_origin must cover start and goal");
            const bool map_reset =
                m.resolution != map_parameters_.resolution ||
                m.size != map_parameters_.size || m.fixed != map_parameters_.fixed ||
                m.origin_x != map_parameters_.origin_x ||
                m.origin_y != map_parameters_.origin_y ||
                m.laser_x != map_parameters_.laser_x ||
                m.laser_y != map_parameters_.laser_y ||
                m.laser_yaw != map_parameters_.laser_yaw;
            map_->configure(m);
            valid_map_ = map_->ready();
            planner_->configure(p);
            global_->configure(g);
            map_parameters_ = m;
            planner_parameters_ = p;
            global_parameters_ = g;
            robot_radius_ = m.robot_radius;
            margin_ = m.safety_margin;
            if (map_reset)
              global_->invalidate();
            ++parameter_updates_;
            RCLCPP_INFO(
                get_logger(),
                "Parameters updated: v=%.2f w=%.2f margin=%.3f horizon=%.2f",
                p.max_speed, p.max_angular, m.safety_margin, p.horizon);
            result.successful = true;
          } catch (const std::exception &error) {
            result.successful = false;
            result.reason = error.what();
          }
          return result;
        });
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    sectors_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "maze_mouse/sectors", 10);
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    status_pub_ =
        create_publisher<std_msgs::msg::String>("maze_mouse/status", qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("maze_mouse/path", qos);
    plan_pub_ = create_publisher<nav_msgs::msg::Path>("maze_mouse/plan", qos);
    reference_pub_ =
        create_publisher<nav_msgs::msg::Path>("maze_mouse/reference", qos);
    trajectory_pub_ =
        create_publisher<nav_msgs::msg::Path>("maze_mouse/trajectory", qos);
    frontier_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "maze_mouse/frontier", qos);
    raw_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "maze_mouse/obstacles", qos);
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "maze_mouse/costmap", qos);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          const auto &v = msg->pose.pose.position;
          const double yaw = yaw_of(msg->pose.pose.orientation),
                       linear = msg->twist.twist.linear.x,
                       angular = msg->twist.twist.angular.z;
          if (!std::isfinite(v.x) || !std::isfinite(v.y) ||
              !std::isfinite(yaw) || !std::isfinite(linear) ||
              !std::isfinite(angular))
            return;
          if (!have_odom_) {
            anchor_x_ = v.x;
            anchor_y_ = v.y;
            rotation_ = wrap_angle(start_yaw_ - yaw);
          } else
            distance_ += std::hypot(v.x - odom_x_, v.y - odom_y_);
          odom_x_ = v.x;
          odom_y_ = v.y;
          const double dx = v.x - anchor_x_, dy = v.y - anchor_y_;
          pose_ = {
              start_x_ + std::cos(rotation_) * dx - std::sin(rotation_) * dy,
              start_y_ + std::sin(rotation_) * dx + std::cos(rotation_) * dy,
              wrap_angle(yaw + rotation_), angular, linear};
          max_measured_ = std::max(max_measured_, std::abs(linear));
          frame_ = msg->header.frame_id.empty() ? "odom" : msg->header.frame_id;
          have_odom_ = true;
          odom_time_ = Clock::now();
        });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
          scan_time_ = Clock::now();
          have_scan_ = true;
          valid_map_ =
              have_odom_ && map_->update(pose_, scan->ranges, scan->angle_min,
                                         scan->angle_increment, scan->range_min,
                                         scan->range_max);
          sectors_ = valid_map_ ? map_->scan_geometry().sectors : Sectors{};
          for (float v : scan->ranges)
            if (std::isfinite(v) && v >= scan->range_min &&
                v <= scan->range_max)
              min_scan_ = std::min(min_scan_, static_cast<double>(v));
          geometry_msgs::msg::Vector3Stamped d;
          d.header = scan->header;
          d.vector.x = sectors_.front.distance;
          d.vector.y = sectors_.left.distance;
          d.vector.z = sectors_.right.distance;
          sectors_pub_->publish(d);
          if (valid_map_)
            publish_maps();
        });
    last_time_ = now().seconds();
    timer_ = create_wall_timer(50ms, [this] { tick(); });
    RCLCPP_INFO(get_logger(),
                "C++20 global A* costmap: body radius %.3f + margin %.3f = "
                "%.3f m; target %.2f m/s; fixed map %s",
                m.robot_radius, m.safety_margin, map_->hard_radius(),
                p.max_speed, m.fixed ? "on" : "off");
  }
  void stop_robot() { publish_command({}); }

private:
  geometry_msgs::msg::PoseStamped point(double x, double y) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = frame_;
    p.header.stamp = now();
    const double dx = x - start_x_, dy = y - start_y_;
    p.pose.position.x =
        anchor_x_ + std::cos(rotation_) * dx + std::sin(rotation_) * dy;
    p.pose.position.y =
        anchor_y_ - std::sin(rotation_) * dx + std::cos(rotation_) * dy;
    p.pose.orientation.w = 1;
    return p;
  }
  void publish_maps() {
    nav_msgs::msg::OccupancyGrid m;
    m.header.frame_id = frame_;
    m.header.stamp = now();
    m.info.resolution = map_->resolution();
    m.info.width = map_->width();
    m.info.height = map_->width();
    m.info.origin = point(map_->origin_x(), map_->origin_y()).pose;
    m.info.origin.orientation.z = std::sin(-rotation_ / 2);
    m.info.origin.orientation.w = std::cos(-rotation_ / 2);
    m.data = map_->raw_data();
    raw_pub_->publish(m);
    m.data = map_->cost_data();
    map_pub_->publish(m);
  }
  void publish_command(WallCommand c) {
    last_command_ = c;
    geometry_msgs::msg::Twist t;
    t.linear.x = c.linear;
    t.angular.z = c.angular;
    command_pub_->publish(t);
  }
  void tick() {
    const double time = now().seconds();
    const auto wall = Clock::now();
    const double dt = std::clamp(time - last_time_, 0.01, 0.1);
    last_time_ = time;
    const bool ready =
        have_odom_ && have_scan_ && valid_map_ && sectors_.usable() &&
        std::chrono::duration<double>(wall - odom_time_).count() < timeout_ &&
        std::chrono::duration<double>(wall - scan_time_).count() < timeout_;
    if (!running_ && ready) {
      running_ = true;
      started_ = time;
      progress_time_ = time;
      progress_ = {pose_.x, pose_.y};
      forced_replan_ = false;
    }
    if (running_ &&
        std::hypot(pose_.x - progress_.x, pose_.y - progress_.y) > 0.10) {
      progress_ = {pose_.x, pose_.y};
      progress_time_ = time;
      forced_replan_ = false;
    }
    const bool no_progress = running_ && ready && time - progress_time_ > 15;
    const bool force_replan =
        running_ && ready && !forced_replan_ && time - progress_time_ > 5;
    if (force_replan) {
      forced_replan_ = true;
      global_->invalidate();
    }
    if (no_progress && !global_->exhausted() &&
        planner_->mode() != LocalMode::complete)
      fault_ = "No translational progress for 15 seconds";
    const auto before = Clock::now();
    if (running_ && ready) {
      global_->update(pose_, *map_, dt, {planner_parameters_.goal_x,
                                         planner_parameters_.goal_y},
                      false);
      max_global_planner_ms_ =
          std::max(max_global_planner_ms_, global_->plan_ms());
    }
    auto command = planner_->step(
        pose_, sectors_, *map_, dt, ready && fault_.empty(), global_->path());
    computation_ms_ =
        std::chrono::duration<double, std::milli>(Clock::now() - before)
            .count();
    max_computation_ms_ = std::max(max_computation_ms_, computation_ms_);
    publish_command(command);
    max_command_ = std::max(max_command_, command.linear);
    if (running_ && planner_->mode() != LocalMode::complete &&
        std::abs(pose_.linear_speed) < 0.03)
      stopped_seconds_ += dt;
    if (running_ && ready && global_->exhausted() &&
        planner_->mode() == LocalMode::blocked)
      exhausted_seconds_ += dt;
    else
      exhausted_seconds_ = 0;
    if (exhausted_seconds_ > 1.5)
      no_frontier_ = true;
    else if (global_->path().size() >= 2)
      no_frontier_ = false;
    const std::string state =
        !fault_.empty()
            ? "FAULT"
            : (no_frontier_ ? "NO_FRONTIER"
                            : (!running_ ? "WAITING"
                                         : (!ready ? "WAIT_FOR_DATA"
                                                   : local_mode_name(
                                                         planner_->mode()))));
    if (state != previous_state_) {
      RCLCPP_INFO(get_logger(),
                  "%s pose=(%.3f,%.3f,%.3f), costmap clearance=%.3f",
                  state.c_str(), pose_.x, pose_.y, pose_.yaw,
                  map_->clearance(pose_.x, pose_.y));
      previous_state_ = state;
    }
    if (elapsed_ < 0 &&
        (state == "COMPLETE" || state == "FAULT" || state == "NO_FRONTIER"))
      elapsed_ = time - started_;
    if (++ticks_ % 5 != 0)
      return;
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_;
    path.header.stamp = now();
    for (auto p : planner_->trajectory())
      path.poses.push_back(point(p.x, p.y));
    path_pub_->publish(path);
    path.poses.clear();
    for (auto p : global_->path())
      path.poses.push_back(point(p.x, p.y));
    plan_pub_->publish(path);
    path.poses.clear();
    for (auto p : planner_->reference())
      path.poses.push_back(point(p.x, p.y));
    reference_pub_->publish(path);
    geometry_msgs::msg::PointStamped frontier;
    frontier.header.frame_id = frame_;
    frontier.header.stamp = now();
    if (global_->target() == PlannerTarget::frontier) {
      frontier.point.x = global_->target_point().x;
      frontier.point.y = global_->target_point().y;
    } else {
      frontier.point.x = std::numeric_limits<double>::quiet_NaN();
      frontier.point.y = std::numeric_limits<double>::quiet_NaN();
    }
    frontier_pub_->publish(frontier);
    if (have_odom_) {
      auto p = point(pose_.x, pose_.y);
      if (trajectory_.poses.empty() ||
          std::hypot(
              p.pose.position.x - trajectory_.poses.back().pose.position.x,
              p.pose.position.y - trajectory_.poses.back().pose.position.y) >
              0.025) {
        if (trajectory_.poses.size() >= 15000)
          trajectory_.poses.erase(trajectory_.poses.begin());
        trajectory_.poses.push_back(p);
      }
      trajectory_.header = path.header;
      trajectory_pub_->publish(trajectory_);
    }
    const double goal_distance = std::hypot(
        planner_parameters_.goal_x - pose_.x,
        planner_parameters_.goal_y - pose_.y);
    std::ostringstream out;
    out << "{\"algorithm\":\"occupancy_astar\",\"state\":\"" << state
        << "\",\"data_ready\":" << (ready ? "true" : "false") << ",\"pose\":["
        << number(pose_.x) << ',' << number(pose_.y) << ',' << number(pose_.yaw)
        << "],\"command_speed\":" << number(command.linear)
        << ",\"command_angular\":" << number(command.angular)
        << ",\"max_command_speed\":" << number(max_command_)
        << ",\"max_measured_speed\":" << number(max_measured_)
        << ",\"configured_max_speed\":" << number(planner_parameters_.max_speed)
        << ",\"configured_max_angular\":"
        << number(planner_parameters_.max_angular)
        << ",\"parameter_updates\":" << parameter_updates_
        << ",\"stopped_sim_s\":" << number(stopped_seconds_)
        << ",\"robot_radius\":" << number(robot_radius_)
        << ",\"inflation_margin\":" << number(margin_)
        << ",\"hard_inflation_radius\":" << number(map_->hard_radius())
        << ",\"costmap_clearance\":"
        << number(map_->clearance(pose_.x, pose_.y))
        << ",\"tracking_clearance\":"
        << number(map_->tracking_clearance(pose_.x, pose_.y))
        << ",\"predicted_clearance\":"
        << number(planner_->predicted_clearance())
        << ",\"tracking_error\":" << number(planner_->tracking_error())
        << ",\"control_lookahead\":"
        << number(planner_->control_lookahead())
        << ",\"reference_update_code\":"
        << planner_->reference_update_code()
        << ",\"reference_remaining\":"
        << number(planner_->reference_remaining())
        << ",\"reference_points\":" << planner_->reference_points()
        << ",\"candidate_points\":" << planner_->candidate_points()
        << ",\"moving_candidates\":" << planner_->moving_candidates()
        << ",\"target_angular\":" << number(planner_->target_angular())
        << ",\"front\":" << number(sectors_.front.distance)
        << ",\"front_minimum_range\":"
        << number(sectors_.front.minimum_range)
        << ",\"front_center_minimum_range\":"
        << number(sectors_.front.center_minimum_range)
        << ",\"left\":" << number(sectors_.left.distance)
        << ",\"left_minimum_range\":" << number(sectors_.left.minimum_range)
        << ",\"right\":" << number(sectors_.right.distance)
        << ",\"right_minimum_range\":" << number(sectors_.right.minimum_range)
        << ",\"side_sector_width_deg\":"
        << number(map_parameters_.side_sector_degrees)
        << ",\"front_sector_width_deg\":"
        << number(map_parameters_.front_sector_degrees)
        << ",\"goal_distance\":" << number(goal_distance)
        << ",\"planner_target\":\"" << target_name(global_->target()) << "\""
        << ",\"frontier_x\":"
        << (global_->target() == PlannerTarget::frontier
                ? number(global_->target_point().x)
                : "null")
        << ",\"frontier_y\":"
        << (global_->target() == PlannerTarget::frontier
                ? number(global_->target_point().y)
                : "null")
        << ",\"replans\":" << global_->replans()
        << ",\"frontier_candidates\":" << global_->frontier_candidates()
        << ",\"exploration_exhausted\":"
        << (global_->exhausted() ? "true" : "false")
        << ",\"global_planner_ms\":" << number(global_->plan_ms())
        << ",\"max_global_planner_ms\":" << number(max_global_planner_ms_)
        << ",\"path_points\":" << global_->path().size()
        << ",\"path_length_m\":" << number(global_->path_length())
        << ",\"path_remaining_m\":"
        << number(global_->path_remaining(pose_))
        << ",\"odom_distance_m\":" << number(distance_)
        << ",\"minimum_laser_range_m\":" << number(min_scan_)
        << ",\"elapsed_sim_s\":"
        << number(elapsed_ >= 0 ? elapsed_ : (running_ ? time - started_ : 0))
        << ",\"planner_ms\":" << number(computation_ms_)
        << ",\"max_planner_ms\":" << number(max_computation_ms_)
        << ",\"reason\":\"" << fault_ << "\"}";
    std_msgs::msg::String s;
    s.data = out.str();
    status_pub_->publish(s);
  }
  const std::unordered_map<std::string, double CostmapParameters::*>
      map_fields_{
          {"costmap_resolution", &CostmapParameters::resolution},
          {"costmap_size", &CostmapParameters::size},
          {"costmap_origin_x", &CostmapParameters::origin_x},
          {"costmap_origin_y", &CostmapParameters::origin_y},
          {"robot_radius", &CostmapParameters::robot_radius},
          {"inflation_margin", &CostmapParameters::safety_margin},
          {"soft_cost_radius", &CostmapParameters::soft_radius},
          {"laser_offset_x", &CostmapParameters::laser_x},
          {"laser_offset_y", &CostmapParameters::laser_y},
          {"laser_yaw", &CostmapParameters::laser_yaw},
          {"side_sector_width_deg", &CostmapParameters::side_sector_degrees},
          {"front_sector_width_deg", &CostmapParameters::front_sector_degrees},
          {"minimum_wall_length", &CostmapParameters::minimum_wall_length},
          {"wall_fit_tolerance", &CostmapParameters::wall_fit_tolerance}};
  const std::unordered_map<std::string, double LocalPlannerParameters::*>
      planner_fields_{
          {"max_linear_speed", &LocalPlannerParameters::max_speed},
          {"max_angular_speed", &LocalPlannerParameters::max_angular},
          {"linear_acceleration", &LocalPlannerParameters::acceleration},
          {"linear_deceleration", &LocalPlannerParameters::deceleration},
          {"angular_acceleration",
           &LocalPlannerParameters::angular_acceleration},
          {"prediction_horizon", &LocalPlannerParameters::horizon},
          {"reaction_time", &LocalPlannerParameters::reaction_time},
          {"heading_stop_angle", &LocalPlannerParameters::heading_stop_angle},
          {"progress_weight", &LocalPlannerParameters::progress_weight},
          {"path_weight", &LocalPlannerParameters::path_weight},
          {"heading_weight", &LocalPlannerParameters::heading_weight},
          {"clearance_weight", &LocalPlannerParameters::clearance_weight},
          {"speed_weight", &LocalPlannerParameters::speed_weight},
          {"pursuit_lookahead", &LocalPlannerParameters::pursuit_lookahead},
          {"pursuit_lookahead_gain",
           &LocalPlannerParameters::pursuit_lookahead_gain},
          {"pursuit_lookahead_max",
           &LocalPlannerParameters::pursuit_lookahead_max},
          {"path_preview_distance",
           &LocalPlannerParameters::path_preview_distance},
          {"path_validation_distance",
           &LocalPlannerParameters::path_validation_distance},
          {"path_deviation", &LocalPlannerParameters::path_deviation},
          {"heading_kp", &LocalPlannerParameters::heading_kp},
          {"curvature_feedforward",
           &LocalPlannerParameters::curvature_feedforward},
          {"max_lateral_acceleration",
           &LocalPlannerParameters::max_lateral_acceleration},
          {"angular_command_acceleration",
           &LocalPlannerParameters::angular_command_acceleration},
          {"pursuit_weight", &LocalPlannerParameters::pursuit_weight},
          {"goal_x", &LocalPlannerParameters::goal_x},
          {"goal_y", &LocalPlannerParameters::goal_y},
          {"goal_tolerance", &LocalPlannerParameters::goal_tolerance}};
  const std::unordered_map<std::string, double GlobalPlannerParameters::*>
      global_fields_{{"replan_period", &GlobalPlannerParameters::replan_period},
                     {"frontier_weight",
                      &GlobalPlannerParameters::frontier_weight},
                     {"frontier_min_distance",
                      &GlobalPlannerParameters::frontier_min_distance},
                     {"frontier_switch_margin",
                      &GlobalPlannerParameters::frontier_switch_margin},
                     {"frontier_sticky_radius",
                      &GlobalPlannerParameters::frontier_sticky_radius},
                     {"frontier_sticky_bonus",
                      &GlobalPlannerParameters::frontier_sticky_bonus},
                     {"start_snap_radius",
                      &GlobalPlannerParameters::start_snap_radius},
                     {"goal_snap_radius",
                      &GlobalPlannerParameters::goal_snap_radius},
                     {"planning_margin",
                      &GlobalPlannerParameters::planning_margin},
                     {"clearance_cost_weight",
                      &GlobalPlannerParameters::clearance_cost_weight},
                     {"clearance_target",
                      &GlobalPlannerParameters::clearance_target},
                     {"planner_path_min_remaining",
                      &GlobalPlannerParameters::path_min_remaining},
                     {"planner_path_deviation",
                      &GlobalPlannerParameters::path_deviation},
                     {"planner_path_validation_distance",
                      &GlobalPlannerParameters::path_validation_distance},
                     {"smoothing_span",
                      &GlobalPlannerParameters::smoothing_span},
                     {"smoothing_clearance",
                      &GlobalPlannerParameters::smoothing_clearance}};
  CostmapParameters map_parameters_;
  LocalPlannerParameters planner_parameters_;
  GlobalPlannerParameters global_parameters_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_;
  std::uint64_t parameter_updates_{};
  double stopped_seconds_{}, exhausted_seconds_{};
  std::unique_ptr<LocalCostmap> map_;
  std::unique_ptr<CostmapFollower> planner_;
  std::unique_ptr<GlobalPlanner> global_;
  WallPose pose_{};
  Sectors sectors_{};
  WallCommand last_command_{};
  WallPoint progress_{};
  double start_x_{}, start_y_{}, start_yaw_{}, anchor_x_{}, anchor_y_{},
      rotation_{}, odom_x_{}, odom_y_{};
  double timeout_{}, robot_radius_{}, margin_{}, last_time_{}, started_{},
      progress_time_{}, elapsed_{-1};
  double distance_{}, max_command_{}, max_measured_{}, min_scan_{1e9},
      computation_ms_{}, max_computation_ms_{}, max_global_planner_ms_{};
  bool have_odom_{}, have_scan_{}, valid_map_{}, running_{}, forced_replan_{},
      no_frontier_{};
  std::uint64_t ticks_{};
  std::string frame_{"odom"}, previous_state_, fault_;
  Clock::time_point odom_time_{}, scan_time_{};
  nav_msgs::msg::Path trajectory_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr sectors_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, plan_pub_,
      reference_pub_, trajectory_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr frontier_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_pub_, map_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};
} // namespace maze_mouse_navigation
namespace {
volatile std::sig_atomic_t stopping = 0;
void request_stop(int) { stopping = 1; }
} // namespace
int main(int argc, char **argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions{},
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  std::shared_ptr<maze_mouse_navigation::CostmapNode> node;
  try {
    node = std::make_shared<maze_mouse_navigation::CostmapNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !stopping)
      executor.spin_once(std::chrono::milliseconds(50));
    for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
      node->stop_robot();
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  } catch (const std::exception &e) {
    if (node && rclcpp::ok()) {
      try {
        node->stop_robot();
      } catch (...) {
      }
    }
    RCLCPP_ERROR(rclcpp::get_logger("costmap_follower"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
