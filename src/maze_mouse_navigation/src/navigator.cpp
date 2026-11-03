#include "maze_mouse_navigation/core.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

namespace maze_mouse_navigation {
using SteadyClock = std::chrono::steady_clock;
using namespace std::chrono_literals;

namespace {
struct Pose {
  double x{}, y{}, yaw{};
};
enum class State { waiting, observing, turning, driving, complete, fault };

double yaw_of(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

const char *state_name(State state) {
  switch (state) {
  case State::waiting:
    return "WAITING";
  case State::observing:
    return "OBSERVING";
  case State::turning:
    return "TURNING";
  case State::driving:
    return "DRIVING";
  case State::complete:
    return "COMPLETE";
  case State::fault:
    return "FAULT";
  }
  return "FAULT";
}

std::string number(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream out;
  out << std::setprecision(8) << value;
  return out.str();
}
} // namespace

class Navigator final : public rclcpp::Node {
public:
  Navigator() : Node("maze_navigator") {
    const auto integer = [this](const char *name, int value) -> int {
      const auto parameter = declare_parameter<std::int64_t>(name, value);
      if (parameter < std::numeric_limits<int>::min() ||
          parameter > std::numeric_limits<int>::max()) {
        throw std::invalid_argument(
            std::string("Integer parameter out of range: ") + name);
      }
      return static_cast<int>(parameter);
    };
    const auto real = [this](const char *name, double value) {
      return declare_parameter<double>(name, value);
    };
    columns_ = integer("columns", 6);
    rows_ = integer("rows", 6);
    grid_ = std::make_unique<GridMaze>(columns_, rows_);
    cell_size_ = real("cell_size", 1.0);
    const double thickness = real("wall_thickness", 0.12);
    cell_ = {integer("start_x", 0), integer("start_y", 0)};
    goal_ = {integer("goal_x", 5), integer("goal_y", 5)};
    const int start_heading = integer("start_heading", 0);
    const int exit_heading = integer("exit_heading", 0);
    if (!grid_->contains(cell_) || !grid_->contains(goal_) ||
        start_heading < 0 || start_heading > 3 || exit_heading < 0 ||
        exit_heading > 3 ||
        !(cell_size_ > 0.0 && thickness > 0.0 && thickness < cell_size_)) {
      throw std::invalid_argument("Invalid grid/start/goal/heading parameters");
    }
    heading_ = static_cast<Direction>(start_heading);
    exit_heading_ = static_cast<Direction>(exit_heading);
    if (grid_->contains(adjacent(goal_, exit_heading_))) {
      throw std::invalid_argument(
          "Exit direction must point out of the grid at the goal cell");
    }
    exit_distance_ = real("exit_distance", 0.95);
    sector_width_ = real("sector_width_deg", 30.0) * pi / 180.0;
    sector_centers_ = {real("front_center_deg", 0.0) * pi / 180.0,
                       real("left_center_deg", 90.0) * pi / 180.0,
                       real("right_center_deg", -90.0) * pi / 180.0};
    wall_distance_ = real("wall_detection_distance", 0.68);
    stop_distance_ = real("front_stop_distance", 0.20);
    observation_scans_ = integer("observation_scans", 3);
    max_speed_ = real("max_linear_speed", 0.20);
    acceleration_ = real("linear_acceleration", 0.35);
    position_tolerance_ = real("position_tolerance", 0.055);
    heading_tolerance_ = real("heading_tolerance", 0.04);
    timeout_ = real("data_timeout", 1.0);
    follow_.heading_gain = real("heading_gain", 1.6);
    follow_.heading_damping = real("heading_damping", 0.25);
    follow_.lateral_gain = real("lateral_gain", 1.8);
    follow_.centering_gain = real("centering_gain", 0.8);
    follow_.side_wall_max = real("side_wall_max", 0.70);
    follow_.max_centering_correction = real("max_centering_correction", 0.15);
    follow_.max_angular_speed = real("max_angular_speed", 0.6);
    follow_.desired_side_distance = (cell_size_ - thickness) / 2.0;
    if (!(sector_width_ > 0.0 && sector_width_ <= pi / 2.0) ||
        observation_scans_ < 1 || !(max_speed_ > 0.0 && max_speed_ <= 0.22) ||
        !(follow_.max_angular_speed >= 0.12 &&
          follow_.max_angular_speed <= 2.84) ||
        !(acceleration_ > 0.0) ||
        !(stop_distance_ > 0.0 && stop_distance_ < wall_distance_) ||
        !(wall_distance_ < cell_size_ && wall_distance_ > cell_size_ / 2.0) ||
        !(position_tolerance_ > 0.0 &&
          position_tolerance_ < cell_size_ / 5.0) ||
        !(heading_tolerance_ > 0.0 && heading_tolerance_ < 0.2) ||
        !(timeout_ > 0.0) || !(exit_distance_ > cell_size_ / 2.0)) {
      throw std::invalid_argument(
          "Invalid sector/controller parameters or Burger speed limits");
    }
    const std::array values{cell_size_,
                            thickness,
                            exit_distance_,
                            sector_width_,
                            wall_distance_,
                            stop_distance_,
                            max_speed_,
                            acceleration_,
                            position_tolerance_,
                            heading_tolerance_,
                            timeout_,
                            follow_.heading_gain,
                            follow_.heading_damping,
                            follow_.lateral_gain,
                            follow_.centering_gain,
                            follow_.side_wall_max,
                            follow_.max_centering_correction,
                            follow_.max_angular_speed,
                            sector_centers_[0],
                            sector_centers_[1],
                            sector_centers_[2]};
    if (!std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); }) ||
        follow_.heading_gain <= 0.0 || follow_.heading_damping < 0.0 ||
        follow_.lateral_gain < 0.0 || follow_.centering_gain < 0.0 ||
        follow_.max_centering_correction < 0.0 ||
        follow_.side_wall_max <= 0.0) {
      throw std::invalid_argument(
          "Control parameters must be finite with nonnegative gains");
    }
    velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    sectors_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "maze_mouse/sectors", 10);
    auto latched = rclcpp::QoS(1).reliable().transient_local();
    status_pub_ =
        create_publisher<std_msgs::msg::String>("maze_mouse/status", latched);
    path_pub_ =
        create_publisher<nav_msgs::msg::Path>("maze_mouse/path", latched);
    trajectory_pub_ =
        create_publisher<nav_msgs::msg::Path>("maze_mouse/trajectory", latched);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
          const auto extract = [&](double center) {
            return extract_sector(scan->ranges, scan->angle_min,
                                  scan->angle_increment, scan->range_min,
                                  scan->range_max, center, sector_width_);
          };
          sectors_ = {extract(sector_centers_[0]), extract(sector_centers_[1]),
                      extract(sector_centers_[2])};
          scan_received_ = SteadyClock::now();
          have_scan_ = true;
          ++scan_id_;
          geometry_msgs::msg::Vector3Stamped output;
          output.header = scan->header;
          output.vector.x = sectors_.front.distance;
          output.vector.y = sectors_.left.distance;
          output.vector.z = sectors_.right.distance;
          sectors_pub_->publish(output);
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          const auto &p = message->pose.pose.position;
          const double yaw = yaw_of(message->pose.pose.orientation);
          if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
              !std::isfinite(yaw)) {
            return;
          }
          if (have_odom_) {
            odom_distance_ += std::hypot(p.x - odom_.x, p.y - odom_.y);
          }
          odom_ = {p.x, p.y, yaw};
          odom_angular_speed_ = message->twist.twist.angular.z;
          odom_linear_speed_ = message->twist.twist.linear.x;
          if (!std::isfinite(odom_angular_speed_) ||
              !std::isfinite(odom_linear_speed_)) {
            have_odom_ = false;
            return;
          }
          odom_frame_ = message->header.frame_id;
          if (odom_frame_.empty()) {
            odom_frame_ = "odom";
          }
          odom_received_ = SteadyClock::now();
          have_odom_ = true;
        });
    previous_tick_ = SteadyClock::now();
    timer_ = create_wall_timer(50ms, [this] { tick(); });
    RCLCPP_INFO(
        get_logger(),
        "C++20 navigator: unknown %dx%d maze, goal=(%d,%d), sectors=%.0f deg",
        columns_, rows_, goal_.x, goal_.y, sector_width_ * 180.0 / pi);
  }

  void stop_robot() { send(0.0, 0.0); }

private:
  void send(double linear, double angular) {
    geometry_msgs::msg::Twist command;
    command.linear.x = linear;
    command.angular.z = angular;
    velocity_pub_->publish(command);
    commanded_speed_ = linear;
  }

  Pose grid_pose() const {
    const double dx = odom_.x - anchor_odom_.x;
    const double dy = odom_.y - anchor_odom_.y;
    return {anchor_grid_.x + std::cos(frame_rotation_) * dx -
                std::sin(frame_rotation_) * dy,
            anchor_grid_.y + std::sin(frame_rotation_) * dx +
                std::cos(frame_rotation_) * dy,
            wrap_angle(odom_.yaw + frame_rotation_)};
  }

  std::pair<double, double> center(Cell cell) const {
    return {(cell.x + 0.5) * cell_size_, (cell.y + 0.5) * cell_size_};
  }

  void transition(State state) {
    state_ = state;
    stable_ticks_ = 0;
    state_started_ = SteadyClock::now();
  }

  void observe() {
    stop_robot();
    observations_ = 0;
    votes_ = {};
    observed_scan_ = scan_id_; // Require new scans after stopping/alignment.
    transition(State::observing);
  }

  void fault(const std::string &message) {
    reason_ = message;
    stop_robot();
    transition(State::fault);
    RCLCPP_ERROR(get_logger(), "%s", reason_.c_str());
    publish_status();
  }

  void update_edge(Direction direction, Edge edge) {
    const auto next = adjacent(cell_, direction);
    if (!grid_->contains(next)) {
      return;
    } // Boundary gates are handled only on final exit.
    const int a = grid_->index(cell_);
    const int b = grid_->index(next);
    if (traversed_.contains({std::min(a, b), std::max(a, b)})) {
      edge = Edge::open;
    }
    grid_->set_edge(cell_, direction, edge);
  }

  void plan() {
    if (cell_ == goal_) {
      exiting_ = true;
      desired_heading_ = exit_heading_;
      const auto [x, y] = center(goal_);
      const double yaw = static_cast<int>(exit_heading_) * pi / 2.0;
      target_ = {x + exit_distance_ * std::cos(yaw),
                 y + exit_distance_ * std::sin(yaw), yaw};
    } else {
      const auto route = grid_->route(cell_, goal_, heading_);
      ++replans_;
      if (route.size() < 2) {
        fault("Goal unreachable in observed maze");
        return;
      }
      target_cell_ = route[1];
      desired_heading_ = direction_to(cell_, target_cell_);
      const auto [x, y] = center(target_cell_);
      target_ = {x, y, static_cast<int>(desired_heading_) * pi / 2.0};
    }
    RCLCPP_INFO(get_logger(),
                "cell=(%d,%d), known_edges=%d, next=(%.2f,%.2f), heading=%d%s",
                cell_.x, cell_.y, grid_->known_edges(), target_.x, target_.y,
                static_cast<int>(desired_heading_), exiting_ ? " EXIT" : "");
    transition(State::turning);
    publish_path();
  }

  void tick() {
    const auto current_time = SteadyClock::now();
    const double dt = std::clamp(
        std::chrono::duration<double>(current_time - previous_tick_).count(),
        0.001, 0.1);
    previous_tick_ = current_time;
    ready_ =
        have_scan_ && have_odom_ && sectors_.usable() &&
        std::chrono::duration<double>(current_time - scan_received_).count() <=
            timeout_ &&
        std::chrono::duration<double>(current_time - odom_received_).count() <=
            timeout_;
    if (!ready_ || state_ == State::fault || state_ == State::complete) {
      stop_robot();
      // Discard votes separated by invalid/missing data; never fuse stale
      // scans.
      if (!ready_ && state_ == State::observing) {
        observations_ = 0;
        votes_ = {};
        observed_scan_ = scan_id_;
      }
    } else if (state_ == State::waiting) {
      anchor_odom_ = odom_;
      const auto [x, y] = center(cell_);
      anchor_grid_ = {x, y, static_cast<int>(heading_) * pi / 2.0};
      frame_rotation_ = wrap_angle(anchor_grid_.yaw - anchor_odom_.yaw);
      initialized_ = true;
      grid_->visit(cell_);
      run_start_sim_ = now().seconds();
      observe();
    } else if (state_ == State::observing) {
      stop_robot();
      if (observed_scan_ != scan_id_) {
        observed_scan_ = scan_id_;
        const std::array<double, 3> distances{sectors_.front.distance,
                                              sectors_.left.distance,
                                              sectors_.right.distance};
        for (std::size_t i = 0; i < distances.size(); ++i) {
          votes_[i] += distances[i] < wall_distance_ ? 1 : 0;
        }
        if (++observations_ >= observation_scans_) {
          for (int i = 0; i < 3; ++i) {
            const int turns = i == 0 ? 0 : (i == 1 ? 1 : -1);
            update_edge(rotate(heading_, turns), votes_[i] * 2 > observations_
                                                     ? Edge::wall
                                                     : Edge::open);
          }
          plan();
        }
      }
    } else if (state_ == State::turning) {
      const double error = wrap_angle(target_.yaw - grid_pose().yaw);
      if (std::abs(error) <= heading_tolerance_) {
        stop_robot();
        // A momentary heading crossing is not a settled turn. Wait for the body
        // and wheel response to stop before enabling forward motion.
        if (std::abs(odom_angular_speed_) < 0.05 &&
            std::abs(odom_linear_speed_) < 0.025) {
          if (++stable_ticks_ >= 5) {
            heading_ = desired_heading_;
            if (aligning_) {
              aligning_ = false;
              observe();
            } else if (!exiting_ &&
                       grid_->edge(cell_, heading_) == Edge::unknown) {
              // Only the initial rear edge can be unseen: inspect it after
              // turning, instead of driving into an optimistic unknown passage.
              RCLCPP_INFO(get_logger(),
                          "Observe unknown edge before driving from (%d,%d)",
                          cell_.x, cell_.y);
              observe();
            } else {
              transition(State::driving);
            }
          }
        } else {
          stable_ticks_ = 0;
        }
      } else if (std::chrono::duration<double>(current_time - state_started_)
                     .count() > 25.0) {
        fault("Turn timeout; check odometry and wheel control");
      } else {
        stable_ticks_ = 0;
        const double omega = follow_.heading_gain * error -
                             follow_.heading_damping * odom_angular_speed_;
        send(0.0, std::clamp(omega, -follow_.max_angular_speed,
                             follow_.max_angular_speed));
      }
    } else if (state_ == State::driving) {
      const auto pose = grid_pose();
      const double dx = target_.x - pose.x;
      const double dy = target_.y - pose.y;
      const double distance = std::hypot(dx, dy);
      const double along =
          dx * std::cos(target_.yaw) + dy * std::sin(target_.yaw);
      const double lateral =
          -dx * std::sin(target_.yaw) + dy * std::cos(target_.yaw);
      const double heading_error = wrap_angle(target_.yaw - pose.yaw);
      if (distance <= position_tolerance_) {
        stop_robot();
        if (exiting_) {
          completed_sim_ = now().seconds() - run_start_sim_;
          transition(State::complete);
          reason_ = "Reached exit target";
          RCLCPP_INFO(
              get_logger(),
              "COMPLETE: %.2f simulation seconds, %d moves, %.2f m odometry",
              completed_sim_, moves_, odom_distance_);
          publish_status();
        } else {
          grid_->set_edge(cell_, desired_heading_, Edge::open);
          const int a = grid_->index(cell_);
          const int b = grid_->index(target_cell_);
          traversed_.insert({std::min(a, b), std::max(a, b)});
          cell_ = target_cell_;
          grid_->visit(cell_);
          ++moves_;
          // Align to the cardinal direction before sampling walls at the new
          // cell.
          target_.yaw = static_cast<int>(heading_) * pi / 2.0;
          if (std::abs(heading_error) > heading_tolerance_) {
            aligning_ = true;
            transition(State::turning);
          } else {
            observe();
          }
        }
      } else if (along < -0.10 || std::abs(lateral) > cell_size_ * 0.28) {
        fault("Tracking error too large; stopped before leaving the planned "
              "corridor");
      } else if (std::chrono::duration<double>(current_time - state_started_)
                     .count() > 40.0) {
        fault("Drive timeout; an obstacle or motion failure blocked the active "
              "passage");
      } else if (sectors_.front.minimum_range <= stop_distance_) {
        stop_robot();
      } else if (std::abs(heading_error) > 0.35) {
        send(0.0,
             std::clamp(follow_.heading_gain * heading_error -
                            follow_.heading_damping * odom_angular_speed_,
                        -follow_.max_angular_speed, follow_.max_angular_speed));
      } else {
        double speed = std::min(max_speed_, std::max(0.025, along));
        speed *= std::max(0.35, std::cos(heading_error));
        speed = std::min(speed, commanded_speed_ + acceleration_ * dt);
        send(speed, follow_angular(heading_error, lateral, sectors_, follow_,
                                   odom_angular_speed_));
      }
    }
    if (++ticks_ % 10 == 0) {
      publish_status();
      publish_path();
    }
    if (ticks_ % 5 == 0 && initialized_) {
      publish_trajectory();
    }
  }

  geometry_msgs::msg::PoseStamped odom_point(double x, double y,
                                             double yaw) const {
    geometry_msgs::msg::PoseStamped result;
    result.header.frame_id = odom_frame_;
    result.header.stamp = now();
    const double dx = x - anchor_grid_.x;
    const double dy = y - anchor_grid_.y;
    result.pose.position.x = anchor_odom_.x + std::cos(frame_rotation_) * dx +
                             std::sin(frame_rotation_) * dy;
    result.pose.position.y = anchor_odom_.y - std::sin(frame_rotation_) * dx +
                             std::cos(frame_rotation_) * dy;
    result.pose.orientation.z = std::sin((yaw - frame_rotation_) / 2.0);
    result.pose.orientation.w = std::cos((yaw - frame_rotation_) / 2.0);
    return result;
  }

  void publish_path() {
    if (!initialized_) {
      return;
    }
    nav_msgs::msg::Path message;
    message.header.frame_id = odom_frame_;
    message.header.stamp = now();
    for (const auto cell : grid_->route(cell_, goal_, heading_)) {
      const auto [x, y] = center(cell);
      message.poses.push_back(odom_point(x, y, 0.0));
    }
    const auto [x, y] = center(goal_);
    const double yaw = static_cast<int>(exit_heading_) * pi / 2.0;
    if (!message.poses.empty()) {
      message.poses.push_back(odom_point(x + exit_distance_ * std::cos(yaw),
                                         y + exit_distance_ * std::sin(yaw),
                                         yaw));
    }
    path_pub_->publish(message);
  }

  void publish_trajectory() {
    trajectory_.header.frame_id = odom_frame_;
    trajectory_.header.stamp = now();
    auto pose = odom_point(grid_pose().x, grid_pose().y, grid_pose().yaw);
    if (trajectory_.poses.empty() ||
        std::hypot(
            pose.pose.position.x - trajectory_.poses.back().pose.position.x,
            pose.pose.position.y - trajectory_.poses.back().pose.position.y) >
            0.02) {
      if (trajectory_.poses.size() >= 15000) {
        trajectory_.poses.erase(trajectory_.poses.begin());
      }
      trajectory_.poses.push_back(pose);
    }
    trajectory_pub_->publish(trajectory_);
  }

  void publish_status() {
    const auto pose = initialized_ ? grid_pose() : Pose{};
    std::ostringstream out;
    out << "{\"state\":\"" << state_name(state_)
        << "\",\"data_ready\":" << (ready_ ? "true" : "false") << ",\"cell\":["
        << cell_.x << ',' << cell_.y << "],\"pose\":[" << number(pose.x) << ','
        << number(pose.y) << ',' << number(pose.yaw) << "],\"target\":["
        << number(target_.x) << ',' << number(target_.y)
        << "],\"front\":" << number(sectors_.front.distance)
        << ",\"left\":" << number(sectors_.left.distance)
        << ",\"right\":" << number(sectors_.right.distance)
        << ",\"front_min\":" << number(sectors_.front.minimum_range)
        << ",\"angular_speed\":" << number(odom_angular_speed_)
        << ",\"command_speed\":" << number(commanded_speed_)
        << ",\"known_edges\":" << grid_->known_edges()
        << ",\"moves\":" << moves_ << ",\"replans\":" << replans_
        << ",\"odom_distance_m\":" << number(odom_distance_)
        << ",\"elapsed_sim_s\":"
        << number(state_ == State::complete
                      ? completed_sim_
                      : (initialized_ ? now().seconds() - run_start_sim_ : 0.0))
        << ",\"reason\":\"" << reason_ << "\",\"flood\":[";
    const auto values = grid_->flood(goal_);
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (i != 0) {
        out << ',';
      }
      out << (values[i] == GridMaze::unreachable ? -1 : values[i]);
    }
    out << "]}";
    std_msgs::msg::String message;
    message.data = out.str();
    status_pub_->publish(message);
  }

  int columns_{}, rows_{}, observation_scans_{}, observations_{}, moves_{},
      replans_{}, stable_ticks_{};
  std::unique_ptr<GridMaze> grid_;
  Cell cell_{}, goal_{}, target_cell_{};
  Direction heading_{}, desired_heading_{}, exit_heading_{};
  State state_{State::waiting};
  bool have_scan_{}, have_odom_{}, initialized_{}, ready_{}, exiting_{},
      aligning_{};
  std::uint64_t scan_id_{}, observed_scan_{}, ticks_{};
  std::array<int, 3> votes_{};
  std::array<double, 3> sector_centers_{};
  std::set<std::pair<int, int>> traversed_;
  Pose odom_{}, anchor_odom_{}, anchor_grid_{}, target_{};
  Sectors sectors_{};
  FollowParameters follow_{};
  double cell_size_{}, exit_distance_{}, sector_width_{}, wall_distance_{},
      stop_distance_{};
  double max_speed_{}, acceleration_{}, position_tolerance_{},
      heading_tolerance_{}, timeout_{};
  double frame_rotation_{}, commanded_speed_{}, odom_distance_{},
      run_start_sim_{}, completed_sim_{};
  double odom_angular_speed_{}, odom_linear_speed_{};
  SteadyClock::time_point scan_received_{}, odom_received_{}, previous_tick_{},
      state_started_{};
  std::string odom_frame_{"odom"}, reason_{};
  nav_msgs::msg::Path trajectory_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr sectors_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, trajectory_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace maze_mouse_navigation

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) { stop_requested = 1; }
} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions{},
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  std::shared_ptr<maze_mouse_navigation::Navigator> node;
  try {
    node = std::make_shared<maze_mouse_navigation::Navigator>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !stop_requested) {
      executor.spin_once(std::chrono::milliseconds(50));
    }
    for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
      node->stop_robot();
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    executor.remove_node(node);
  } catch (const std::exception &error) {
    if (node && rclcpp::ok()) {
      try {
        node->stop_robot();
      } catch (...) {
      } // Preserve the original failure.
    }
    RCLCPP_ERROR(rclcpp::get_logger("maze_navigator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
