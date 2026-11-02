#include "maze_mouse_navigation/left_wall.hpp"
#include <chrono>
#include <csignal>
#include <iomanip>
#include <memory>
#include <sstream>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

namespace maze_mouse_navigation {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace {
double yaw_of(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z));
}
std::string number(double v) {
  if (!std::isfinite(v)) {
    return "null";
  }
  std::ostringstream out;
  out << std::setprecision(9) << v;
  return out.str();
}
} // namespace

class WallNode final : public rclcpp::Node {
public:
  WallNode() : Node("maze_navigator") {
    WallParameters p;
    const auto real = [this](const char *name, double value) {
      return declare_parameter<double>(name, value);
    };
    p.speed = real("max_linear_speed", p.speed);
    p.approach_speed = real("approach_speed", p.approach_speed);
    p.cell_size = real("cell_size", p.cell_size);
    p.wall_distance = real("desired_left_distance", p.wall_distance);
    p.opening_distance = real("opening_distance", p.opening_distance);
    p.front_turn_distance = real("front_turn_distance", p.front_turn_distance);
    p.front_stop_distance = real("front_stop_distance", p.front_stop_distance);
    p.side_stop_distance = real("side_stop_distance", p.side_stop_distance);
    p.max_angular_speed = real("max_angular_speed", p.max_angular_speed);
    p.acceleration = real("linear_acceleration", p.acceleration);
    p.deceleration = real("linear_deceleration", p.deceleration);
    p.heading_gain = real("heading_gain", p.heading_gain);
    p.wall_gain = real("wall_gain", p.wall_gain);
    p.angular_damping = real("angular_damping", p.angular_damping);
    p.goal_x = real("goal_x", p.goal_x);
    p.goal_y = real("goal_y", p.goal_y);
    p.goal_yaw = real("goal_yaw", p.goal_yaw);
    p.goal_tolerance = real("goal_tolerance", p.goal_tolerance);
    p.opening_scans = declare_parameter<int>("opening_scans", p.opening_scans);
    follower_ = std::make_unique<LeftWallFollower>(p);
    start_x_ = real("start_x", 0.5);
    start_y_ = real("start_y", 0.5);
    start_yaw_ = real("start_yaw", 0.0);
    width_ = real("sector_width_deg", 30) * pi / 180.0;
    timeout_ = real("data_timeout", 0.7);
    if (!(width_ > 0 && width_ <= pi / 2) || !(timeout_ > 0) ||
        !std::isfinite(timeout_) || !std::isfinite(start_x_) ||
        !std::isfinite(start_y_) || !std::isfinite(start_yaw_)) {
      throw std::invalid_argument("Invalid scan/frame parameters");
    }
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    sectors_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "maze_mouse/sectors", 10);
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    status_pub_ =
        create_publisher<std_msgs::msg::String>("maze_mouse/status", qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("maze_mouse/path", qos);
    trajectory_pub_ =
        create_publisher<nav_msgs::msg::Path>("maze_mouse/trajectory", qos);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
          const auto sector = [&](double center) {
            return extract_sector(scan->ranges, scan->angle_min,
                                  scan->angle_increment, scan->range_min,
                                  scan->range_max, center, width_);
          };
          sectors_ = {sector(0), sector(pi / 2), sector(-pi / 2)};
          ++scan_id_;
          scan_received_ = Clock::now();
          have_scan_ = true;
          for (const float v : scan->ranges) {
            if (std::isfinite(v) && v >= scan->range_min &&
                v <= scan->range_max) {
              minimum_scan_ = std::min(minimum_scan_, static_cast<double>(v));
            }
          }
          geometry_msgs::msg::Vector3Stamped distances;
          distances.header = scan->header;
          distances.vector.x = sectors_.front.distance;
          distances.vector.y = sectors_.left.distance;
          distances.vector.z = sectors_.right.distance;
          sectors_pub_->publish(distances);
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr odom) {
          const auto &position = odom->pose.pose.position;
          const double yaw = yaw_of(odom->pose.pose.orientation);
          const double omega = odom->twist.twist.angular.z;
          if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
              !std::isfinite(yaw) || !std::isfinite(omega)) {
            return;
          }
          if (std::isfinite(odom->twist.twist.linear.x)) {
            max_measured_speed_ = std::max(
                max_measured_speed_, std::abs(odom->twist.twist.linear.x));
          }
          if (!have_odom_) {
            anchor_x_ = position.x;
            anchor_y_ = position.y;
            rotation_ = wrap_angle(start_yaw_ - yaw);
          } else {
            distance_ += std::hypot(position.x - odom_x_, position.y - odom_y_);
          }
          odom_x_ = position.x;
          odom_y_ = position.y;
          odom_yaw_ = yaw;
          const double dx = position.x - anchor_x_, dy = position.y - anchor_y_;
          pose_ = {
              start_x_ + std::cos(rotation_) * dx - std::sin(rotation_) * dy,
              start_y_ + std::sin(rotation_) * dx + std::cos(rotation_) * dy,
              wrap_angle(yaw + rotation_), omega, odom->twist.twist.linear.x};
          frame_ =
              odom->header.frame_id.empty() ? "odom" : odom->header.frame_id;
          odom_received_ = Clock::now();
          have_odom_ = true;
        });
    last_time_ = now().seconds();
    timer_ = create_wall_timer(50ms, [this] { tick(); });
    RCLCPP_INFO(
        get_logger(),
        "C++20 left-wall follower: straight target %.2f m/s, approach %.2f "
        "m/s, no flood planning",
        p.speed, p.approach_speed);
  }

  void stop_robot() { publish_command({}); }

private:
  void publish_command(WallCommand command) {
    last_command_ = command;
    geometry_msgs::msg::Twist message;
    message.linear.x = command.linear;
    message.angular.z = command.angular;
    command_pub_->publish(message);
  }
  geometry_msgs::msg::PoseStamped odom_point(double x, double y) {
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = now();
    p.header.frame_id = frame_;
    const double dx = x - start_x_, dy = y - start_y_;
    p.pose.position.x =
        anchor_x_ + std::cos(rotation_) * dx + std::sin(rotation_) * dy;
    p.pose.position.y =
        anchor_y_ - std::sin(rotation_) * dx + std::cos(rotation_) * dy;
    p.pose.orientation.w = 1;
    return p;
  }
  void tick() {
    const auto wall_time = Clock::now();
    const double time = now().seconds();
    const double dt = time > last_time_ ? time - last_time_ : 0.05;
    last_time_ = time;
    ready_ =
        have_odom_ && have_scan_ && sectors_.usable() &&
        std::chrono::duration<double>(wall_time - scan_received_).count() <=
            timeout_ &&
        std::chrono::duration<double>(wall_time - odom_received_).count() <=
            timeout_;
    if (!running_ && ready_) {
      running_ = true;
      started_ = time;
    }
    if (!running_) {
      stop_robot();
    } else {
      publish_command(follower_->step(pose_, sectors_, scan_id_, dt, ready_));
    }
    const auto mode = follower_->mode();
    if (mode != previous_mode_) {
      RCLCPP_INFO(get_logger(),
                  "%s: pose=(%.3f,%.3f,%.3f), left=%.3f front=%.3f right=%.3f",
                  wall_mode_name(mode), pose_.x, pose_.y, pose_.yaw,
                  sectors_.left.distance, sectors_.front.distance,
                  sectors_.right.distance);
      previous_mode_ = mode;
      if (mode == WallMode::complete || mode == WallMode::fault) {
        elapsed_ = time - started_;
      }
    }
    max_command_speed_ = std::max(max_command_speed_, last_command_.linear);
    if (++ticks_ % 5 != 0) {
      return;
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_;
    path.header.stamp = now();
    for (const auto p : follower_->local_path()) {
      path.poses.push_back(odom_point(p.x, p.y));
    }
    path_pub_->publish(path);
    if (have_odom_) {
      auto point = odom_point(pose_.x, pose_.y);
      if (trajectory_.poses.empty() ||
          std::hypot(point.pose.position.x -
                         trajectory_.poses.back().pose.position.x,
                     point.pose.position.y -
                         trajectory_.poses.back().pose.position.y) > 0.025) {
        if (trajectory_.poses.size() >= 15000) {
          trajectory_.poses.erase(trajectory_.poses.begin());
        }
        trajectory_.poses.push_back(point);
      }
      trajectory_.header = path.header;
      trajectory_pub_->publish(trajectory_);
    }
    std::ostringstream out;
    out << "{\"algorithm\":\"left_wall\",\"state\":\""
        << (running_ ? wall_mode_name(mode) : "WAITING")
        << "\",\"data_ready\":" << (ready_ ? "true" : "false") << ",\"pose\":["
        << number(pose_.x) << ',' << number(pose_.y) << ',' << number(pose_.yaw)
        << "],\"front\":" << number(sectors_.front.distance)
        << ",\"left\":" << number(sectors_.left.distance)
        << ",\"right\":" << number(sectors_.right.distance)
        << ",\"command_speed\":" << number(last_command_.linear)
        << ",\"max_command_speed\":" << number(max_command_speed_)
        << ",\"max_measured_speed\":" << number(max_measured_speed_)
        << ",\"left_turns\":" << follower_->left_turns()
        << ",\"right_turns\":" << follower_->right_turns()
        << ",\"u_turns\":" << follower_->u_turns()
        << ",\"odom_distance_m\":" << number(distance_)
        << ",\"minimum_laser_range_m\":" << number(minimum_scan_)
        << ",\"elapsed_sim_s\":"
        << number(elapsed_ >= 0 ? elapsed_ : (running_ ? time - started_ : 0))
        << ",\"reason\":\"" << follower_->reason() << "\"}";
    std_msgs::msg::String status;
    status.data = out.str();
    status_pub_->publish(status);
  }

  std::unique_ptr<LeftWallFollower> follower_;
  WallPose pose_{};
  Sectors sectors_{};
  WallCommand last_command_{};
  WallMode previous_mode_{WallMode::following};
  double start_x_{}, start_y_{}, start_yaw_{}, rotation_{}, anchor_x_{},
      anchor_y_{};
  double odom_x_{}, odom_y_{}, odom_yaw_{}, width_{}, timeout_{}, last_time_{},
      started_{};
  double distance_{}, minimum_scan_{std::numeric_limits<double>::infinity()},
      max_command_speed_{}, max_measured_speed_{}, elapsed_{-1};
  bool have_scan_{}, have_odom_{}, ready_{}, running_{};
  std::uint64_t scan_id_{}, ticks_{};
  std::string frame_{"odom"};
  Clock::time_point scan_received_{}, odom_received_{};
  nav_msgs::msg::Path trajectory_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr sectors_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};
} // namespace maze_mouse_navigation

namespace {
volatile std::sig_atomic_t stopping = 0;
void stop_signal(int) { stopping = 1; }
} // namespace
int main(int argc, char **argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions{},
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, stop_signal);
  std::signal(SIGTERM, stop_signal);
  std::shared_ptr<maze_mouse_navigation::WallNode> node;
  try {
    node = std::make_shared<maze_mouse_navigation::WallNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !stopping) {
      executor.spin_once(std::chrono::milliseconds(50));
    }
    for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
      node->stop_robot();
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  } catch (const std::exception &error) {
    if (node && rclcpp::ok()) {
      try {
        node->stop_robot();
      } catch (...) {
      }
    }
    RCLCPP_ERROR(rclcpp::get_logger("left_wall_follower"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
