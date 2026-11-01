#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
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
#include <memory>
#include <sstream>
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
} // namespace
class CostmapNode final : public rclcpp::Node {
public:
  CostmapNode() : Node("maze_navigator") {
    auto real = [this](const char *n, double d) {
      return declare_parameter<double>(n, d);
    };
    CostmapParameters m;
    for (const auto &[name, field] : map_fields_)
      m.*field = real(name.c_str(), m.*field);
    map_ = std::make_unique<LocalCostmap>(m);
    robot_radius_ = m.robot_radius;
    margin_ = m.safety_margin;
    LocalPlannerParameters p;
    for (const auto &[name, field] : planner_fields_)
      p.*field = real(name.c_str(), p.*field);
    p.linear_samples =
        declare_parameter<int>("linear_samples", p.linear_samples);
    p.angular_samples =
        declare_parameter<int>("angular_samples", p.angular_samples);
    if (p.wall_distance <= m.robot_radius + m.safety_margin)
      throw std::invalid_argument(
          "Wall contour distance must exceed hard inflation radius");
    planner_ = std::make_unique<CostmapFollower>(p);
    map_parameters_ = m;
    planner_parameters_ = p;
    start_x_ = real("start_x", 0.5);
    start_y_ = real("start_y", 0.5);
    start_yaw_ = real("start_yaw", 0);
    timeout_ = real("data_timeout", 0.7);
    width_ = real("sector_width_deg", 30) * pi / 180;
    if (!std::isfinite(start_x_) || !std::isfinite(start_y_) ||
        !std::isfinite(start_yaw_) || !(timeout_ > 0) ||
        !std::isfinite(timeout_) || !(width_ > 0 && width_ <= pi / 2))
      throw std::invalid_argument("Invalid frame/scan parameters");
    parameter_callback_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          try {
            auto m = map_parameters_;
            auto p = planner_parameters_;
            for (const auto &parameter : parameters) {
              const auto &name = parameter.get_name();
              if (auto field = map_fields_.find(name);
                  field != map_fields_.end())
                m.*(field->second) = parameter.as_double();
              else if (auto field = planner_fields_.find(name);
                       field != planner_fields_.end())
                p.*(field->second) = parameter.as_double();
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
            if (p.wall_distance <= m.robot_radius + m.safety_margin)
              throw std::invalid_argument(
                  "Wall contour distance must exceed hard inflation radius");
            map_->configure(m);
            valid_map_ = map_->ready();
            planner_->configure(p);
            map_parameters_ = m;
            planner_parameters_ = p;
            robot_radius_ = m.robot_radius;
            margin_ = m.safety_margin;
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
    trajectory_pub_ =
        create_publisher<nav_msgs::msg::Path>("maze_mouse/trajectory", qos);
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
          auto extract = [&](double center) {
            return extract_sector(scan->ranges, scan->angle_min,
                                  scan->angle_increment, scan->range_min,
                                  scan->range_max, center, width_);
          };
          sectors_ = {extract(0), extract(pi / 2), extract(-pi / 2)};
          scan_time_ = Clock::now();
          have_scan_ = true;
          valid_map_ =
              have_odom_ && map_->update(pose_, scan->ranges, scan->angle_min,
                                         scan->angle_increment, scan->range_min,
                                         scan->range_max);
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
                "C++20 laser costmap: body radius %.3f + margin %.3f = %.3f m; "
                "target %.2f m/s; no maze-cell centres",
                m.robot_radius, m.safety_margin, map_->hard_radius(),
                p.max_speed);
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
    }
    if (running_ &&
        std::hypot(pose_.x - progress_.x, pose_.y - progress_.y) > 0.10) {
      progress_ = {pose_.x, pose_.y};
      progress_time_ = time;
    }
    if (running_ && ready && planner_->mode() != LocalMode::complete &&
        time - progress_time_ > 15) {
      fault_ = "No translational progress for 15 seconds";
    }
    const auto before = Clock::now();
    auto command =
        planner_->step(pose_, sectors_, *map_, dt, ready && fault_.empty());
    computation_ms_ =
        std::chrono::duration<double, std::milli>(Clock::now() - before)
            .count();
    max_computation_ms_ = std::max(max_computation_ms_, computation_ms_);
    publish_command(command);
    max_command_ = std::max(max_command_, command.linear);
    if (running_ && planner_->mode() != LocalMode::complete &&
        std::abs(pose_.linear_speed) < 0.03)
      stopped_seconds_ += dt;
    const std::string state =
        !fault_.empty()
            ? "FAULT"
            : (!running_ ? "WAITING"
                         : (!ready ? "WAIT_FOR_DATA"
                                   : local_mode_name(planner_->mode())));
    if (state != previous_state_) {
      RCLCPP_INFO(get_logger(),
                  "%s pose=(%.3f,%.3f,%.3f), costmap clearance=%.3f",
                  state.c_str(), pose_.x, pose_.y, pose_.yaw,
                  map_->clearance(pose_.x, pose_.y));
      previous_state_ = state;
    }
    if (elapsed_ < 0 && (state == "COMPLETE" || state == "FAULT"))
      elapsed_ = time - started_;
    if (++ticks_ % 5 != 0)
      return;
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_;
    path.header.stamp = now();
    for (auto p : planner_->trajectory())
      path.poses.push_back(point(p.x, p.y));
    path_pub_->publish(path);
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
    std::ostringstream out;
    out << "{\"algorithm\":\"laser_costmap\",\"state\":\"" << state
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
        << ",\"predicted_clearance\":"
        << number(planner_->predicted_clearance())
        << ",\"front\":" << number(sectors_.front.distance)
        << ",\"left\":" << number(sectors_.left.distance)
        << ",\"right\":" << number(sectors_.right.distance)
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
      map_fields_{{"costmap_resolution", &CostmapParameters::resolution},
                  {"costmap_size", &CostmapParameters::size},
                  {"robot_radius", &CostmapParameters::robot_radius},
                  {"inflation_margin", &CostmapParameters::safety_margin},
                  {"soft_cost_radius", &CostmapParameters::soft_radius},
                  {"laser_offset_x", &CostmapParameters::laser_x},
                  {"laser_offset_y", &CostmapParameters::laser_y},
                  {"laser_yaw", &CostmapParameters::laser_yaw}};
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
          {"desired_left_distance", &LocalPlannerParameters::wall_distance},
          {"target_lookahead", &LocalPlannerParameters::target_lookahead},
          {"target_tolerance", &LocalPlannerParameters::target_tolerance},
          {"wall_target_offset", &LocalPlannerParameters::wall_offset},
          {"recovery_enter_margin", &LocalPlannerParameters::recovery_enter},
          {"recovery_exit_margin", &LocalPlannerParameters::recovery_exit},
          {"recovery_speed", &LocalPlannerParameters::recovery_speed},
          {"heading_stop_angle", &LocalPlannerParameters::heading_stop_angle},
          {"progress_weight", &LocalPlannerParameters::progress_weight},
          {"path_weight", &LocalPlannerParameters::path_weight},
          {"heading_weight", &LocalPlannerParameters::heading_weight},
          {"clearance_weight", &LocalPlannerParameters::clearance_weight},
          {"speed_weight", &LocalPlannerParameters::speed_weight},
          {"goal_x", &LocalPlannerParameters::goal_x},
          {"goal_y", &LocalPlannerParameters::goal_y},
          {"goal_tolerance", &LocalPlannerParameters::goal_tolerance}};
  CostmapParameters map_parameters_;
  LocalPlannerParameters planner_parameters_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_;
  std::uint64_t parameter_updates_{};
  double stopped_seconds_{};
  std::unique_ptr<LocalCostmap> map_;
  std::unique_ptr<CostmapFollower> planner_;
  WallPose pose_{};
  Sectors sectors_{};
  WallCommand last_command_{};
  WallPoint progress_{};
  double start_x_{}, start_y_{}, start_yaw_{}, anchor_x_{}, anchor_y_{},
      rotation_{}, odom_x_{}, odom_y_{};
  double timeout_{}, width_{}, robot_radius_{}, margin_{}, last_time_{},
      started_{}, progress_time_{}, elapsed_{-1};
  double distance_{}, max_command_{}, max_measured_{}, min_scan_{1e9},
      computation_ms_{}, max_computation_ms_{};
  bool have_odom_{}, have_scan_{}, valid_map_{}, running_{};
  std::uint64_t ticks_{};
  std::string frame_{"odom"}, previous_state_, fault_;
  Clock::time_point odom_time_{}, scan_time_{};
  nav_msgs::msg::Path trajectory_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr sectors_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, trajectory_pub_;
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
