#include "maze_mouse_navigation/core.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

using namespace maze_mouse_navigation;
namespace {
const float inf = std::numeric_limits<float>::infinity();
const float invalid_range = std::numeric_limits<float>::quiet_NaN();
Sector sector(const std::vector<float> & scan, double center, double minimum = 0.0) {
  return extract_sector(scan, minimum, pi / 180.0, 0.12, 3.5, center, pi / 6.0);
}
Sector wall(double distance) {
  Sector result;
  result.distance = distance;
  result.minimum_range = distance;
  result.usable = true;
  return result;
}
}

TEST(LaserSectors, FrontWrapsAcrossZeroAndProjectsToPerpendicularDistance) {
  std::vector<float> scan(360, inf);
  for (int i = 0; i < 360; ++i) {
    const double angle = wrap_angle(i * pi / 180.0);
    if (std::abs(angle) <= pi / 12.0 + 1e-8) { scan[i] = 0.44 / std::cos(angle); }
  }
  const auto output = sector(scan, 0.0);
  EXPECT_TRUE(output.usable);
  EXPECT_EQ(output.total, 31U);
  EXPECT_NEAR(output.distance, 0.44, 1e-6);
}

TEST(LaserSectors, SupportsNegativeAngleMinAndRightHandSector) {
  std::vector<float> scan(360, inf);
  for (int i = 0; i < 360; ++i) {
    const double relative = wrap_angle(-pi + i * pi / 180.0 + pi / 2.0);
    if (std::abs(relative) < pi / 12.0 + 1e-8) { scan[i] = 0.31 / std::cos(relative); }
  }
  const auto output = sector(scan, -pi / 2.0, -pi);
  EXPECT_TRUE(output.usable);
  EXPECT_NEAR(output.distance, 0.31, 1e-6);
}

TEST(LaserSectors, InfMeansFreeSpaceButNaNAndMissingCoverageCannotDrive) {
  EXPECT_TRUE(sector(std::vector<float>(360, inf), 0.0).usable);
  EXPECT_GT(sector(std::vector<float>(360, inf), 0.0).distance, 3.0);
  EXPECT_FALSE(sector(std::vector<float>(360, invalid_range), 0.0).usable);
  EXPECT_FALSE(sector(std::vector<float>(360, -inf), 0.0).usable);
  EXPECT_FALSE(sector(std::vector<float>(360, 0.01F), 0.0).usable);
  EXPECT_FALSE(sector(std::vector<float>(10, inf), 0.0).usable);
  EXPECT_FALSE(extract_sector({}, 0.0, 0.0, 0.12, 3.5, 0.0, pi / 6.0).usable);
}

TEST(LaserSectors, RobustMedianDoesNotHideEmergencyMinimum) {
  std::vector<float> scan(360, 2.0F);
  scan[0] = 0.13F;
  const auto output = sector(scan, 0.0);
  EXPECT_TRUE(output.usable);
  EXPECT_GT(output.distance, 1.8);
  EXPECT_NEAR(output.minimum_range, 0.13, 1e-6);
}

TEST(FloodFill, UnknownGridUsesGoalFloodAndRejectsUnknownInConservativeMode) {
  GridMaze maze(6, 6);
  const auto distance = maze.flood({5, 5});
  EXPECT_EQ(distance[maze.index({0, 0})], 10);
  EXPECT_EQ(distance[maze.index({5, 5})], 0);
  EXPECT_TRUE(maze.route({0, 0}, {5, 5}, Direction::east, false).empty());
  const auto route = maze.route({0, 0}, {5, 5}, Direction::east);
  ASSERT_EQ(route.size(), 11U);
  EXPECT_EQ(route[1], (Cell{1, 0}));
}

TEST(FloodFill, ReciprocalObservationsAndNewWallTriggerRealDetour) {
  GridMaze maze(3, 2);
  EXPECT_EQ(maze.flood({2, 0})[maze.index({0, 0})], 2);
  maze.set_edge({1, 0}, Direction::east, Edge::wall);
  EXPECT_EQ(maze.edge({2, 0}, Direction::west), Edge::wall);
  EXPECT_EQ(maze.flood({2, 0})[maze.index({0, 0})], 4);
  const auto route = maze.route({0, 0}, {2, 0}, Direction::east);
  ASSERT_EQ(route.size(), 5U);
  for (std::size_t i = 1; i < route.size(); ++i) {
    EXPECT_NE(maze.edge(route[i - 1], direction_to(route[i - 1], route[i])), Edge::wall);
  }
  maze.set_edge({0, 0}, Direction::east, Edge::wall);
  maze.set_edge({0, 0}, Direction::north, Edge::wall);
  EXPECT_TRUE(maze.route({0, 0}, {2, 0}, Direction::east).empty());
  EXPECT_EQ(maze.flood({2, 0})[maze.index({0, 0})], GridMaze::unreachable);
}

TEST(FloodFill, BoundariesStayClosedAndStartAtGoalHasOneCell) {
  GridMaze maze(2, 2);
  EXPECT_THROW(maze.set_edge({0, 0}, Direction::west, Edge::open), std::invalid_argument);
  EXPECT_THROW(maze.flood({-1, 0}), std::out_of_range);
  EXPECT_EQ(maze.route({1, 1}, {1, 1}, Direction::west), (std::vector<Cell>{{1, 1}}));
  EXPECT_THROW(GridMaze(0, 6), std::invalid_argument);
}

TEST(Centering, CorrectsTowardMiddleAndHandlesSingleWallOrIntersection) {
  FollowParameters parameters;
  Sectors sectors{wall(2.0), wall(0.60), wall(0.28)};
  EXPECT_GT(follow_angular(0.0, 0.0, sectors, parameters), 0.0);
  std::swap(sectors.left, sectors.right);
  EXPECT_LT(follow_angular(0.0, 0.0, sectors, parameters), 0.0);
  sectors.left = sectors.right = wall(0.44);
  EXPECT_DOUBLE_EQ(follow_angular(0.0, 0.0, sectors, parameters), 0.0);
  sectors.left = wall(3.5);
  sectors.right = wall(0.25);
  EXPECT_GT(centering_error(sectors, parameters), 0.0);
  sectors.right = wall(3.5);
  EXPECT_DOUBLE_EQ(centering_error(sectors, parameters), 0.0);
  EXPECT_LE(std::abs(follow_angular(2.0, 1.0, sectors, parameters)), parameters.max_angular_speed);
  EXPECT_LT(follow_angular(0.0, 0.0, sectors, parameters, 0.5), 0.0);
}
