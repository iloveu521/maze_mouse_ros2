#include "maze_mouse_navigation/left_wall.hpp"
#include "tinyxml2.h"
#include "gtest/gtest.h"
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace maze_mouse_navigation;
namespace {
Sector reading(double distance) {
  Sector s;
  s.distance = distance;
  s.minimum_range = distance;
  s.usable = true;
  return s;
}
Sectors readings(double front, double left, double right) {
  return {reading(front), reading(left), reading(right)};
}

struct Shape {
  double x, y, a, b;
  bool circle;
};
std::vector<Shape> load_world() {
  tinyxml2::XMLDocument document;
  if (document.LoadFile(MAZE_WORLD_PATH) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("World unavailable");
  }
  std::vector<Shape> shapes;
  for (auto *model = document.FirstChildElement("sdf")
                         ->FirstChildElement("world")
                         ->FirstChildElement("model");
       model != nullptr; model = model->NextSiblingElement("model")) {
    const std::string name = model->Attribute("name");
    if (!name.starts_with("wall_") && !name.starts_with("pillar_")) {
      continue;
    }
    double x{}, y{}, z{}, roll{}, pitch{}, yaw{};
    std::sscanf(model->FirstChildElement("pose")->GetText(),
                "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &roll, &pitch, &yaw);
    if (std::abs(yaw) > 1e-6) {
      throw std::runtime_error("Expected axis-aligned maze walls");
    }
    auto *geometry = model->FirstChildElement("link")
                         ->FirstChildElement("collision")
                         ->FirstChildElement("geometry");
    if (auto *box = geometry->FirstChildElement("box")) {
      double a{}, b{}, h{};
      std::sscanf(box->FirstChildElement("size")->GetText(), "%lf %lf %lf", &a,
                  &b, &h);
      shapes.push_back({x, y, a / 2, b / 2, false});
    } else {
      double radius{};
      geometry->FirstChildElement("cylinder")
          ->FirstChildElement("radius")
          ->QueryDoubleText(&radius);
      shapes.push_back({x, y, radius, radius, true});
    }
  }
  return shapes;
}
double ray(const Shape &s, double x, double y, double dx, double dy) {
  if (s.circle) {
    const double b = (x - s.x) * dx + (y - s.y) * dy;
    const double c = (x - s.x) * (x - s.x) + (y - s.y) * (y - s.y) - s.a * s.a;
    const double discriminant = b * b - c;
    if (discriminant < 0) {
      return 100;
    }
    const double t = -b - std::sqrt(discriminant);
    return t > 0 ? t : 100;
  }
  double minimum = 0, maximum = 100;
  for (int axis = 0; axis < 2; ++axis) {
    const double o = axis == 0 ? x - s.x : y - s.y;
    const double d = axis == 0 ? dx : dy;
    const double a = axis == 0 ? s.a : s.b;
    if (std::abs(d) < 1e-10) {
      if (std::abs(o) > a) {
        return 100;
      }
      continue;
    }
    const double t1 = (-a - o) / d, t2 = (a - o) / d;
    minimum = std::max(minimum, std::min(t1, t2));
    maximum = std::min(maximum, std::max(t1, t2));
    if (minimum > maximum) {
      return 100;
    }
  }
  return minimum > 0 ? minimum : 100;
}
} // namespace

#include "maze_mouse_navigation/local_costmap.hpp"

namespace {
std::vector<float> wall_scan(const WallPose &pose, double wall_x) {
  std::vector<float> ranges(360, std::numeric_limits<float>::infinity());
  const double sx = pose.x - 0.032 * std::cos(pose.yaw);
  for (int i = 0; i < 360; ++i) {
    const double c = std::cos(pose.yaw + i * pi / 180.0);
    if (c > 1e-6) {
      const double r = (wall_x - sx) / c;
      if (r >= 0.12 && r <= 3.5)
        ranges[i] = r;
    }
  }
  return ranges;
}
} // namespace
TEST(Costmap, BodyRadiusAndExtraTenCentimetresAreBothInflated) {
  LocalCostmap padded;
  CostmapParameters parameters;
  parameters.safety_margin = 0;
  LocalCostmap body_only(parameters);
  const WallPose pose{};
  const auto scan = wall_scan(pose, 1.0);
  ASSERT_TRUE(padded.update(pose, scan, 0, pi / 180, 0.12, 3.5));
  ASSERT_TRUE(body_only.update(pose, scan, 0, pi / 180, 0.12, 3.5));
  EXPECT_NEAR(padded.hard_radius(), 0.24, 1e-9);
  EXPECT_FALSE(padded.traversable(0.76, 0));
  EXPECT_TRUE(body_only.traversable(0.76, 0));
  EXPECT_TRUE(padded.traversable(0.60, 0));
  EXPECT_FALSE(padded.line_clear({0, 0}, {1.1, 0}));
  const auto costs = padded.cost_data();
  EXPECT_NE(std::find(costs.begin(), costs.end(), 100), costs.end());
}
TEST(Costmap, InvalidScanDoesNotCreateFreeSpaceAndUnknownIsNotTraversable) {
  LocalCostmap map;
  std::vector<float> invalid(360, std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(map.update({}, invalid, 0, pi / 180, 0.12, 3.5));
  EXPECT_FALSE(map.ready());
  EXPECT_FALSE(map.traversable(0, 0));
  const auto raw = map.raw_data();
  EXPECT_TRUE(
      std::all_of(raw.begin(), raw.end(), [](auto c) { return c == -1; }));
}
TEST(Costmap, RollingOriginAndLaserOffsetKeepWallInWorldCoordinates) {
  LocalCostmap map;
  WallPose p{0.13, 0.17, 0, 0, 0};
  auto scan = wall_scan(p, 1.0);
  ASSERT_TRUE(map.update(p, scan, 0, pi / 180, 0.12, 3.5));
  EXPECT_FALSE(map.traversable(0.9, 0.17));
  p.x = 0.37;
  scan = wall_scan(p, 1.0);
  ASSERT_TRUE(map.update(p, scan, 0, pi / 180, 0.12, 3.5));
  EXPECT_FALSE(map.traversable(0.9, 0.17));
  EXPECT_TRUE(map.traversable(0.55, 0.17));
}
TEST(CostmapPlanner, MissingDataAndCompletedGoalOutputZero) {
  LocalCostmap map;
  CostmapFollower controller;
  auto command = controller.step({}, readings(3.5, 0.4, 0.4), map, 0.05, false);
  EXPECT_DOUBLE_EQ(command.linear, 0);
  EXPECT_DOUBLE_EQ(command.angular, 0);
  WallPose pose{6.4, 5.5, 0, 0, 0};
  std::vector<float> ranges(360, std::numeric_limits<float>::infinity());
  ASSERT_TRUE(map.update(pose, ranges, 0, pi / 180, 0.12, 3.5));
  command = controller.step(pose, readings(3.5, 3.5, 3.5), map, 0.05, true);
  EXPECT_EQ(controller.mode(), LocalMode::complete);
  EXPECT_DOUBLE_EQ(command.linear, 0);
}
class CostmapWorld : public ::testing::TestWithParam<bool> {};
TEST_P(CostmapWorld, FullWorldWithFiveHertzLaserAndActuatorLag) {
  const auto shapes = load_world();
  ASSERT_FALSE(shapes.empty());
  LocalCostmap map;
  CostmapFollower controller;
  const bool noisy = GetParam();
  WallPose pose =
      noisy ? WallPose{0.56, 0.47, 0.07, 0, 0} : WallPose{0.5, 0.5, 0, 0, 0};
  std::mt19937 random(20261101);
  std::normal_distribution<double> noise(0, 0.01);
  Sectors sectors;
  double minimum = 100, maximum_speed = 0;
  int moving_turns = 0;
  LocalMode previous = LocalMode::blocked;
  WallPoint progress{pose.x, pose.y};
  int last_progress = 0;
  for (int tick = 0; tick < 10000; ++tick) {
    if (tick % 4 == 0) {
      std::vector<float> ranges(360, std::numeric_limits<float>::infinity());
      const double x = pose.x - 0.032 * std::cos(pose.yaw),
                   y = pose.y - 0.032 * std::sin(pose.yaw);
      for (int i = 0; i < 360; ++i) {
        const double angle = pose.yaw + i * pi / 180;
        double distance = 100;
        for (const auto &shape : shapes)
          distance = std::min(
              distance, ray(shape, x, y, std::cos(angle), std::sin(angle)));
        if (noisy)
          distance += noise(random);
        if (distance >= 0.12 && distance <= 3.5)
          ranges[i] = distance;
      }
      ASSERT_TRUE(map.update(pose, ranges, 0, pi / 180, 0.12, 3.5));
      auto extract = [&](double angle) {
        return extract_sector(ranges, 0, pi / 180, 0.12, 3.5, angle, pi / 6);
      };
      sectors = {extract(0), extract(pi / 2), extract(-pi / 2)};
    }
    const auto command = controller.step(pose, sectors, map, 0.05, true);
    maximum_speed = std::max(maximum_speed, command.linear);
    if (command.linear > 0.1 && std::abs(command.angular) > 0.2)
      ++moving_turns;
    if (controller.mode() != previous || tick % 200 == 0) {
      std::cout << tick * 0.05 << " " << local_mode_name(controller.mode())
                << " xy=" << pose.x << "," << pose.y << " yaw=" << pose.yaw
                << " v/w=" << command.linear << "," << command.angular
                << " desired=" << controller.desired_yaw()
                << " d=" << map.clearance(pose.x, pose.y) << "\n";
      previous = controller.mode();
    }
    if (controller.mode() == LocalMode::complete) {
      EXPECT_GT(maximum_speed, 0.4);
      EXPECT_GT(moving_turns, 100);
      EXPECT_GT(minimum, 0.23);
      std::cout << "COSTMAP_COMPLETE sim_s=" << tick * 0.05
                << " max_v=" << maximum_speed
                << " min_physical_clearance=" << minimum << "\n";
      return;
    }
    if (std::hypot(pose.x - progress.x, pose.y - progress.y) > 0.10) {
      progress = {pose.x, pose.y};
      last_progress = tick;
    }
    ASSERT_LT(tick - last_progress, 400)
        << "No progress; pose=" << pose.x << "," << pose.y << "," << pose.yaw;
    for (int sub = 0; sub < 5; ++sub) {
      pose.linear_speed += std::clamp(
          (command.linear - pose.linear_speed) * 0.01 / 0.12, -0.01, 0.008);
      pose.angular_speed += std::clamp(
          (command.angular - pose.angular_speed) * 0.01 / 0.15, -0.015, 0.015);
      pose.x += pose.linear_speed * std::cos(pose.yaw) * 0.01;
      pose.y += pose.linear_speed * std::sin(pose.yaw) * 0.01;
      pose.yaw = wrap_angle(pose.yaw + pose.angular_speed * 0.01);
      ASSERT_GT(pose.x, 0.0) << "Must not bypass maze through the entrance";
      ASSERT_GT(pose.y, 0.0);
      ASSERT_LT(pose.y, 6.0);
      for (const auto &s : shapes) {
        const double distance =
            s.circle ? std::hypot(pose.x - s.x, pose.y - s.y) - s.a
                     : std::hypot(std::max(0.0, std::abs(pose.x - s.x) - s.a),
                                  std::max(0.0, std::abs(pose.y - s.y) - s.b));
        minimum = std::min(minimum, distance);
        ASSERT_GT(distance, 0.23)
            << "Inflated body margin crossed: xy=" << pose.x << "," << pose.y
            << " time=" << tick * 0.05;
      }
    }
  }
  FAIL() << "Did not reach exit";
}

TEST(CostmapPlanner, ReadyMapStillRejectsUnknownRobotPosition) {
  LocalCostmap map;
  const WallPose origin{};
  ASSERT_TRUE(
      map.update(origin, wall_scan(origin, 1.0), 0, pi / 180, 0.12, 3.5));
  CostmapFollower controller;
  auto command = controller.step({8, 8, 0, 0, 0}, readings(3.5, 3.5, 3.5), map,
                                 0.05, true);
  EXPECT_EQ(controller.mode(), LocalMode::blocked);
  EXPECT_DOUBLE_EQ(command.linear, 0);
  EXPECT_DOUBLE_EQ(command.angular, 0);
}

INSTANTIATE_TEST_SUITE_P(CleanAndNoisyOffsetStart, CostmapWorld,
                         ::testing::Bool());

TEST(CostmapLiveParameters, InflationUpdatePreservesObservedMap) {
  LocalCostmap map;
  const WallPose pose{};
  ASSERT_TRUE(map.update(pose, wall_scan(pose, 1.0), 0, pi / 180, 0.12, 3.5));
  const auto observed = map.raw_data();
  EXPECT_FALSE(map.traversable(0.74, 0));
  CostmapParameters smaller;
  smaller.safety_margin = 0.02;
  map.configure(smaller);
  EXPECT_TRUE(map.ready());
  EXPECT_EQ(map.raw_data(), observed);
  EXPECT_NEAR(map.hard_radius(), 0.16, 1e-9);
  EXPECT_TRUE(map.traversable(0.74, 0));
  smaller.safety_margin = -0.1;
  EXPECT_THROW(map.configure(smaller), std::invalid_argument);
  EXPECT_NEAR(map.hard_radius(), 0.16, 1e-9);
  EXPECT_EQ(map.raw_data(), observed);
}

TEST(CostmapLiveParameters, SpeedUpdateDoesNotRestartCompletedNavigation) {
  LocalCostmap map;
  WallPose pose{6.4, 5.5, 0, 0, 0};
  std::vector<float> scan(360, std::numeric_limits<float>::infinity());
  ASSERT_TRUE(map.update(pose, scan, 0, pi / 180, 0.12, 3.5));
  CostmapFollower controller;
  controller.step(pose, readings(3.5, 3.5, 3.5), map, 0.05, true);
  ASSERT_EQ(controller.mode(), LocalMode::complete);
  LocalPlannerParameters faster;
  faster.max_speed = 1.2;
  faster.max_angular = 2.0;
  controller.configure(faster);
  const auto command =
      controller.step(pose, readings(3.5, 3.5, 3.5), map, 0.05, true);
  EXPECT_EQ(controller.mode(), LocalMode::complete);
  EXPECT_DOUBLE_EQ(command.linear, 0);
  faster.heading_stop_angle = pi;
  EXPECT_THROW(controller.configure(faster), std::invalid_argument);
  EXPECT_EQ(controller.mode(), LocalMode::complete);
}

TEST(CostmapPlanner, ReplacesTargetDisconnectedByNewLaserObstacle) {
  const WallPose pose{0.3, 0, -pi / 2, 0, 0};
  LocalCostmap map;
  CostmapFollower controller;
  ASSERT_TRUE(map.update(pose, wall_scan(pose, 1.0), 0, pi / 180, 0.12, 3.5));
  const auto sectors = readings(3.5, 0.7, 3.5);
  controller.step(pose, sectors, map, 0.05, true);
  ASSERT_GE(controller.reference().size(), 2u);
  const auto previous_target = controller.reference().back();
  ASSERT_LT(previous_target.y, -0.7);
  const std::vector<Shape> obstacles{{1, 0, 0.03, 5, false},
                                     {0, -0.45, 3, 0.025, false}};
  std::vector<float> ranges(360, std::numeric_limits<float>::infinity());
  for (int i = 0; i < 360; ++i) {
    const double angle = pose.yaw + i * pi / 180;
    double distance = 100;
    for (const auto &shape : obstacles)
      distance =
          std::min(distance, ray(shape, pose.x - 0.032 * std::cos(pose.yaw),
                                 pose.y - 0.032 * std::sin(pose.yaw),
                                 std::cos(angle), std::sin(angle)));
    if (distance >= 0.12 && distance <= 3.5)
      ranges[i] = distance;
  }
  for (int frame = 0; frame < 3; ++frame)
    ASSERT_TRUE(map.update(pose, ranges, 0, pi / 180, 0.12, 3.5));
  ASSERT_TRUE(map.traversable(previous_target.x, previous_target.y));
  ASSERT_TRUE(map.route({pose.x, pose.y}, previous_target).empty());
  controller.step(pose, sectors, map, 0.05, true);
  EXPECT_NE(controller.mode(), LocalMode::blocked);
  EXPECT_FALSE(controller.trajectory().empty());
}
