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
std::vector<Shape> load_world(const char *path = MAZE_WORLD_PATH) {
  tinyxml2::XMLDocument document;
  if (document.LoadFile(path) != tinyxml2::XML_SUCCESS) {
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
TEST(Costmap, ZeroSoftRadiusPublishesOnlyHardObstacles) {
  CostmapParameters parameters;
  parameters.soft_radius = 0;
  LocalCostmap map(parameters);
  const WallPose pose{};
  const auto scan = wall_scan(pose, 1.0);
  ASSERT_TRUE(map.update(pose, scan, 0, pi / 180, 0.12, 3.5));
  const auto costs = map.cost_data();
  EXPECT_NE(std::find(costs.begin(), costs.end(), 100), costs.end());
  EXPECT_TRUE(std::all_of(costs.begin(), costs.end(), [](std::int8_t value) {
    return value == -1 || value == 0 || value == 100;
  }));
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

#include "maze_mouse_navigation/global_planner.hpp"

namespace {
std::vector<float> shape_scan(const WallPose &pose,
                              const std::vector<Shape> &shapes) {
  std::vector<float> ranges(360, std::numeric_limits<float>::infinity());
  const double x = pose.x - .032 * std::cos(pose.yaw),
               y = pose.y - .032 * std::sin(pose.yaw);
  for (int i = 0; i < 360; ++i) {
    const double a = pose.yaw + i * pi / 180;
    double d = 100;
    for (const auto &shape : shapes)
      d = std::min(d, ray(shape, x, y, std::cos(a), std::sin(a)));
    if (d >= .12 && d <= 3.5)
      ranges[i] = d;
  }
  return ranges;
}
CostmapParameters fixed_map(double size, double origin_x, double origin_y,
                            double resolution = .05) {
  CostmapParameters p;
  p.resolution = resolution;
  p.size = size;
  p.fixed = true;
  p.origin_x = origin_x;
  p.origin_y = origin_y;
  p.safety_margin = .04;
  p.soft_radius = 0;
  return p;
}
std::vector<float> open_scan(int beams = 360) {
  return std::vector<float>(beams, std::numeric_limits<float>::infinity());
}
std::vector<WallPoint> straight(double x0, double y0, double x1, double y1,
                                double step = .1) {
  std::vector<WallPoint> path;
  const double length = std::hypot(x1 - x0, y1 - y0);
  const int count = std::max(1, static_cast<int>(std::ceil(length / step)));
  for (int i = 0; i <= count; ++i) {
    const double t = static_cast<double>(i) / count;
    path.push_back({x0 + t * (x1 - x0), y0 + t * (y1 - y0)});
  }
  return path;
}
} // namespace

TEST(Costmap, FixedOriginKeepsWorldCoordinates) {
  LocalCostmap map(fixed_map(8, -4, -4, .04));
  WallPose pose{.13, .17, 0, 0, 0};
  ASSERT_TRUE(map.update(pose, wall_scan(pose, 1.0), 0, pi / 180, .12, 3.5));
  EXPECT_FALSE(map.traversable(.9, .17));
  pose.x = .37;
  ASSERT_TRUE(map.update(pose, wall_scan(pose, 1.0), 0, pi / 180, .12, 3.5));
  EXPECT_NEAR(map.origin_x(), -4, 1e-9);
  EXPECT_NEAR(map.origin_y(), -4, 1e-9);
  EXPECT_FALSE(map.traversable(.9, .17));
  EXPECT_TRUE(map.traversable(.55, .17));
}

TEST(ScanGeometry, IndependentSixtyAndNinetyDegreeCoverage) {
  std::vector<float> scan(360, std::numeric_limits<float>::infinity());
  const auto geometry = analyse_scan(scan, 0, pi / 180, .12, 3.5);
  EXPECT_EQ(geometry.sectors.front.total, 91u);
  EXPECT_EQ(geometry.sectors.left.total, 61u);
  EXPECT_EQ(geometry.sectors.right.total, 61u);
  EXPECT_TRUE(geometry.sectors.usable());
}
TEST(ScanGeometry, WallThicknessFaceIsNotAGuideButRemainsAnObstacle) {
  const WallPose pose{};
  const auto scan = shape_scan(pose, {{0, -1, .06, .5, false}});
  const auto geometry = analyse_scan(scan, 0, pi / 180, .12, 3.5);
  EXPECT_GT(geometry.short_faces, 0);
  EXPECT_TRUE(geometry.walls.empty());
  EXPECT_NEAR(geometry.sectors.right.minimum_range, .5, .02);
  EXPECT_DOUBLE_EQ(geometry.sectors.right.distance, 3.5);
  LocalCostmap map;
  ASSERT_TRUE(map.update(pose, scan, 0, pi / 180, .12, 3.5));
  EXPECT_LT(map.clearance(0, -.49), .1);
  EXPECT_FALSE(map.traversable(0, -.4));
}

TEST(CostmapTracking, PlanningBufferDoesNotRejectPhysicallyClearTracking) {
  CostmapParameters p;
  p.resolution = .04;
  p.safety_margin = .04;
  LocalCostmap map(p);
  ASSERT_TRUE(map.update({}, wall_scan({}, 1.0), 0, pi / 180, .12, 3.5));
  EXPECT_FALSE(map.traversable(.80, 0));
  EXPECT_GT(map.tracking_clearance(.80, 0), map.tracking_radius());
  EXPECT_TRUE(map.tracking_line_clear({.78, -.1}, {.80, .1}));
  EXPECT_FALSE(map.tracking_line_clear({.80, 0}, {.92, 0}));
}

TEST(CostmapFollower, FollowsPlannedPath) {
  CostmapParameters mp;
  mp.resolution = .04;
  LocalCostmap map(mp);
  const WallPose pose{};
  ASSERT_TRUE(map.update(pose, wall_scan(pose, 1.6), 0, pi / 180, .12, 3.5));
  CostmapFollower follower;
  const auto command = follower.step(
      pose, map.scan_geometry().sectors, map, .05, true,
      straight(0, 0, 1.2, 0));
  EXPECT_EQ(follower.mode(), LocalMode::following);
  EXPECT_FALSE(follower.reference().empty());
  EXPECT_GT(command.linear, 0.0);
}
TEST(CostmapFollower, EmptyPlanBlocks) {
  LocalCostmap map;
  const WallPose pose{};
  ASSERT_TRUE(map.update(pose, wall_scan(pose, 1.6), 0, pi / 180, .12, 3.5));
  CostmapFollower follower;
  const auto command =
      follower.step(pose, map.scan_geometry().sectors, map, .05, true, {});
  EXPECT_EQ(follower.mode(), LocalMode::blocked);
  EXPECT_DOUBLE_EQ(command.linear, 0.0);
  EXPECT_DOUBLE_EQ(command.angular, 0.0);
}
TEST(CostmapFollower, GoalToleranceCompletes) {
  LocalCostmap map;
  const WallPose pose{6.4, 5.5, 0, 0, 0};
  ASSERT_TRUE(map.update(pose, wall_scan(pose, 1.0), 0, pi / 180, .12, 3.5));
  CostmapFollower follower;
  const auto command =
      follower.step(pose, map.scan_geometry().sectors, map, .05, true, {});
  EXPECT_EQ(follower.mode(), LocalMode::complete);
  EXPECT_DOUBLE_EQ(command.linear, 0.0);
}
TEST(CostmapController, SpeedPreviewAndAngularSlewPreventSteeringJump) {
  const std::vector<Shape> walls{{0, .56, 3, .06, false},
                                 {-1.8, -.56, 1.4, .06, false},
                                 {1.8, -.56, 1.4, .06, false}};
  CostmapParameters map_parameters;
  map_parameters.safety_margin = .04;
  map_parameters.resolution = .04;
  LocalCostmap map(map_parameters);
  LocalPlannerParameters parameters;
  parameters.max_speed = .8;
  parameters.max_angular = 1.8;
  parameters.angular_command_acceleration = 2.5;
  CostmapFollower follower(parameters);
  const auto plan = straight(-.6, .1, 1.2, .1);

  WallPose pose{-.60, .10, 0, 0, .8};
  ASSERT_TRUE(map.update(pose, shape_scan(pose, walls), 0, pi / 180, .12, 3.5));
  const auto first = follower.step(pose, map.scan_geometry().sectors, map, .05,
                                   true, plan);
  EXPECT_GT(follower.control_lookahead(), .3);
  EXPECT_LE(std::abs(first.angular), .125 + 1e-6);

  pose.y = -.10;
  ASSERT_TRUE(map.update(pose, shape_scan(pose, walls), 0, pi / 180, .12, 3.5));
  const auto second = follower.step(pose, map.scan_geometry().sectors, map,
                                    .05, true, plan);
  EXPECT_LE(std::abs(second.angular - first.angular), .125 + 1e-6);
}

TEST(GlobalPlanner, RoutesAroundKnownWall) {
  LocalCostmap map(fixed_map(8, -4, -4));
  const std::vector<Shape> walls{{1.0, 0.0, .06, .8, false}};
  const WallPose start{-1.0, 0.0, 0, 0, 0};
  ASSERT_TRUE(
      map.update(start, shape_scan(start, walls), 0, pi / 180, .12, 3.5));
  const WallPose goal_pose{3.0, 0.0, 0, 0, 0};
  ASSERT_TRUE(map.update(goal_pose, shape_scan(goal_pose, walls), 0, pi / 180,
                         .12, 3.5));
  GlobalPlanner planner;
  ASSERT_TRUE(planner.update(start, map, 1.0, {3.0, 0.0}, true));
  EXPECT_EQ(planner.target(), PlannerTarget::goal);
  ASSERT_GE(planner.path().size(), 2u);
  EXPECT_NEAR(planner.path().front().x, start.x, .06);
  EXPECT_NEAR(planner.path().back().x, 3.0, .2);
  for (std::size_t i = 1; i < planner.path().size(); ++i)
    EXPECT_TRUE(map.tracking_line_clear(planner.path()[i - 1],
                                        planner.path()[i]));
}
TEST(GlobalPlanner, SelectsFrontierTowardGoal) {
  LocalCostmap map(fixed_map(14, -7, -7));
  const WallPose start{};
  ASSERT_TRUE(map.update(start, open_scan(), 0, pi / 180, .12, 3.5));
  GlobalPlanner planner;
  ASSERT_TRUE(planner.update(start, map, 1.0, {5.0, 0.0}, true));
  EXPECT_EQ(planner.target(), PlannerTarget::frontier);
  EXPECT_GE(planner.path().size(), 2u);
  EXPECT_GT(planner.target_point().x, 1.0);
}
TEST(GlobalPlanner, ReportsExhaustedWhenNoFrontierRemains) {
  LocalCostmap map(fixed_map(4, -2, -2));
  const WallPose start{};
  const std::vector<Shape> room{{1, 0, .06, 1.2, false},
                                {-1, 0, .06, 1.2, false},
                                {0, 1, 1.2, .06, false},
                                {0, -1, 1.2, .06, false}};
  ASSERT_TRUE(
      map.update(start, shape_scan(start, room), 0, pi / 180, .12, 3.5));
  GlobalPlanner planner;
  EXPECT_FALSE(planner.update(start, map, 1.0, {10.0, 0.0}, true));
  EXPECT_TRUE(planner.exhausted());
}
TEST(GlobalPlanner, FindsFrontierInSixBySixMaze) {
  const auto walls = load_world();
  CostmapParameters mp;
  mp.resolution = .05;
  mp.size = 10;
  mp.fixed = true;
  mp.origin_x = -1.525;
  mp.origin_y = -2.0;
  mp.safety_margin = .04;
  mp.soft_radius = 0;
  LocalCostmap map(mp);
  const WallPose start{0.5, 0.5, 0, 0, 0};
  ASSERT_TRUE(
      map.update(start, shape_scan(start, walls), 0, pi / 180, .12, 3.5));
  GlobalPlanner planner;
  ASSERT_TRUE(planner.update(start, map, 1.0, {6.45, 5.5}, true));
  EXPECT_NE(planner.target(), PlannerTarget::none);
  EXPECT_GE(planner.path().size(), 2u);
}
