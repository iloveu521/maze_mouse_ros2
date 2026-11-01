#include "maze_mouse_navigation/left_wall.hpp"
#include "tinyxml2.h"
#include "gtest/gtest.h"
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <limits>
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

TEST(LeftWall, StraightTargetIsHalfAMetrePerSecondAndDataLossStops) {
  LeftWallFollower follower;
  WallCommand command;
  for (int i = 0; i < 30; ++i) {
    command = follower.step({}, readings(3.5, 0.4, 0.48), i + 1, 0.05, true);
  }
  EXPECT_NEAR(command.linear, 0.5, 1e-9);
  EXPECT_NEAR(command.angular, 0, 1e-9);
  command = follower.step({}, readings(3.5, 0.4, 0.48), 31, 0.05, false);
  EXPECT_DOUBLE_EQ(command.linear, 0);
  EXPECT_DOUBLE_EQ(command.angular, 0);
}
TEST(LeftWall, LeftOpeningHasPriorityAndRequiresDistinctScans) {
  LeftWallFollower follower;
  const auto s = readings(0.7, 1.5, 1.5);
  follower.step({}, s, 1, 0.05, true);
  for (int i = 0; i < 10; ++i) {
    follower.step({}, s, 1, 0.05, true);
  }
  EXPECT_EQ(follower.mode(), WallMode::following);
  const auto command = follower.step({}, s, 2, 0.05, true);
  EXPECT_EQ(follower.mode(), WallMode::approaching);
  EXPECT_GE(command.linear, 0);
  EXPECT_EQ(follower.left_turns(), 1);
  EXPECT_EQ(follower.right_turns(), 0);
}
TEST(LeftWall, FrontWallSelectsRightTurnOrDeadEndReversal) {
  LeftWallFollower right;
  right.step({0.5, 0.5, 0, 0}, readings(0.75, 0.4, 1.5), 1, 0.05, true);
  for (int i = 0; i < 6; ++i)
    right.step({0.5, 0.5, 0, 0}, readings(0.75, 0.4, 1.5), i + 2, 0.05, true);
  EXPECT_EQ(right.mode(), WallMode::right_turn);
  LeftWallFollower reverse;
  reverse.step({0.5, 0.5, 0, 0}, readings(0.75, 0.4, 0.48), 1, 0.05, true);
  for (int i = 0; i < 6; ++i)
    reverse.step({0.5, 0.5, 0, 0}, readings(0.75, 0.4, 0.48), i + 2, 0.05,
                 true);
  EXPECT_EQ(reverse.mode(), WallMode::u_turn);
}
TEST(LeftWall, ObstacleInvalidSectorAndGoalStop) {
  LeftWallFollower follower;
  auto s = readings(3.5, 0.4, 0.48);
  s.left.usable = false;
  EXPECT_DOUBLE_EQ(follower.step({}, s, 1, 0.05, true).linear, 0);
  follower.step({}, readings(0.1, 0.4, 0.48), 2, 0.05, true);
  EXPECT_EQ(follower.mode(), WallMode::fault);
  LeftWallFollower goal;
  EXPECT_DOUBLE_EQ(
      goal.step({6.4, 5.5, 0, 0}, readings(3.5, 3.5, 3.5), 1, 0.05, true)
          .linear,
      0);
  EXPECT_EQ(goal.mode(), WallMode::complete);
  WallParameters invalid;
  invalid.speed = -0.5;
  EXPECT_THROW(LeftWallFollower{invalid}, std::invalid_argument);
}

TEST(LeftWall, TurnRequiresSettledVelocityBeforeCruising) {
  LeftWallFollower approach;
  const auto opening = readings(3.5, 1.5, 1.5);
  approach.step({0.1, 0.5, 0, 0}, opening, 1, 0.05, true);
  approach.step({0.1, 0.5, 0, 0}, opening, 2, 0.05, true);
  for (int i = 0; i < 3; ++i)
    approach.step({0.48, 0.5, 0, 0}, opening, i + 3, 0.05, true);
  approach.step({0.4, 0.5, 0, 0}, opening, 6, 0.05, true);
  for (int i = 0; i < 2; ++i)
    approach.step({0.48, 0.5, 0, 0}, opening, i + 7, 0.05, true);
  EXPECT_EQ(approach.mode(), WallMode::approaching);
  for (int i = 0; i < 3; ++i)
    approach.step({0.48, 0.5, 0, 0}, opening, i + 9, 0.05, true);
  EXPECT_EQ(approach.mode(), WallMode::left_turn);
  LeftWallFollower follower;
  const WallPose start{0.5, 0.5, 0, 0};
  follower.step(start, readings(3.5, 1.5, 1.5), 1, 0.05, true);
  follower.step(start, readings(3.5, 1.5, 1.5), 2, 0.05, true);
  for (int i = 0; i < 5; ++i)
    follower.step(start, readings(3.5, 1.5, 1.5), i + 3, 0.05, true);
  ASSERT_EQ(follower.mode(), WallMode::left_turn);
  for (int i = 0; i < 10; ++i) {
    auto command = follower.step({0.5, 0.5, pi / 2, 0.4},
                                 readings(3.5, 0.4, 0.48), i + 8, 0.05, true);
    EXPECT_DOUBLE_EQ(command.linear, 0);
  }
  EXPECT_EQ(follower.mode(), WallMode::left_turn);
  for (int i = 0; i < 5; ++i)
    follower.step({0.5, 0.5, pi / 2, 0}, readings(3.5, 0.4, 0.48), i + 18, 0.05,
                  true);
  EXPECT_EQ(follower.mode(), WallMode::following);
}

TEST(LeftWall, AStalledJunctionTimesOutAndStops) {
  LeftWallFollower follower;
  const WallPose pose{0.1, 0.5, 0, 0};
  WallCommand command;
  for (int i = 0; i < 400; ++i) {
    command = follower.step(pose, readings(3.5, 1.5, 1.5), i + 1, 0.05, true);
  }
  EXPECT_EQ(follower.mode(), WallMode::fault);
  EXPECT_DOUBLE_EQ(command.linear, 0);
  EXPECT_DOUBLE_EQ(command.angular, 0);
}

TEST(LeftWall, RaycastWorldWithFiveHertzLaserReachesExitWithoutBodyCollision) {
  const auto shapes = load_world();
  ASSERT_FALSE(shapes.empty());
  LeftWallFollower follower;
  WallPose pose{0.5, 0.5, 0, 0};
  Sectors sectors;
  std::uint64_t scan_id = 0;
  double minimum_clearance = 100, maximum_speed = 0;
  WallMode previous_mode = WallMode::following;
  for (int step = 0; step < 16000; ++step) {
    if (step % 4 == 0) {
      std::vector<float> ranges(360, std::numeric_limits<float>::infinity());
      const double x = pose.x - 0.032 * std::cos(pose.yaw),
                   y = pose.y - 0.032 * std::sin(pose.yaw);
      for (int i = 0; i < 360; ++i) {
        const double theta = pose.yaw + i * pi / 180.0;
        double distance = 100;
        for (const auto &shape : shapes) {
          distance = std::min(
              distance, ray(shape, x, y, std::cos(theta), std::sin(theta)));
        }
        if (distance >= 0.12 && distance <= 3.5) {
          ranges[i] = distance;
        }
      }
      const auto extract = [&](double center) {
        return extract_sector(ranges, 0, pi / 180.0, 0.12, 3.5, center, pi / 6);
      };
      sectors = {extract(0), extract(pi / 2), extract(-pi / 2)};
      ++scan_id;
    }
    const auto command = follower.step(pose, sectors, scan_id, 0.05, true);
    if (follower.mode() != previous_mode) {
      std::cout << "t=" << step * 0.05 << " " << wall_mode_name(follower.mode())
                << " xy=" << pose.x << "," << pose.y << " yaw=" << pose.yaw
                << " F/L/R=" << sectors.front.distance << ","
                << sectors.left.distance << "," << sectors.right.distance
                << "\n";
      previous_mode = follower.mode();
    }
    maximum_speed = std::max(maximum_speed, command.linear);
    if (follower.mode() == WallMode::complete) {
      EXPECT_NEAR(maximum_speed, 0.5, 1e-6);
      EXPECT_GT(minimum_clearance, 0.13);
      std::cout << "Raycast complete: " << step * 0.05 << " s, min clearance "
                << minimum_clearance << " m\n";
      return;
    }
    ASSERT_NE(follower.mode(), WallMode::fault)
        << follower.reason() << " xy=" << pose.x << "," << pose.y
        << " t=" << step * 0.05;
    for (int sub = 0; sub < 5; ++sub) {
      pose.x += command.linear * std::cos(pose.yaw) * 0.01;
      pose.y += command.linear * std::sin(pose.yaw) * 0.01;
      pose.yaw = wrap_angle(pose.yaw + command.angular * 0.01);
      pose.angular_speed = command.angular;
      pose.linear_speed = command.linear;
      for (const auto &s : shapes) {
        const double distance =
            s.circle ? std::hypot(pose.x - s.x, pose.y - s.y) - s.a
                     : std::hypot(std::max(0.0, std::abs(pose.x - s.x) - s.a),
                                  std::max(0.0, std::abs(pose.y - s.y) - s.b));
        minimum_clearance = std::min(minimum_clearance, distance);
        ASSERT_GT(distance, 0.13)
            << "Body collision xy=" << pose.x << "," << pose.y
            << " t=" << step * 0.05
            << " mode=" << wall_mode_name(follower.mode());
      }
    }
  }
  FAIL() << "Did not finish. xy=" << pose.x << "," << pose.y
         << " mode=" << wall_mode_name(follower.mode());
}
