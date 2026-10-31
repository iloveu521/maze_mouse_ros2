#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace maze_mouse_navigation {

constexpr double pi = 3.14159265358979323846;
double wrap_angle(double angle);

struct Cell {
  int x{};
  int y{};
  bool operator==(const Cell &) const = default;
};

// Counterclockwise: east=0, north=1, west=2, south=3.
enum class Direction : int { east, north, west, south };
enum class Edge : int { unknown, open, wall };
Direction rotate(Direction direction, int quarter_turns);
Cell adjacent(Cell cell, Direction direction);
Direction direction_to(Cell from, Cell to);

struct Sector {
  double distance{std::numeric_limits<double>::quiet_NaN()};
  double minimum_range{std::numeric_limits<double>::infinity()};
  std::size_t total{};
  std::size_t valid{};
  bool usable{};
};

struct Sectors {
  Sector front;
  Sector left;
  Sector right;
  bool usable() const { return front.usable && left.usable && right.usable; }
};

Sector extract_sector(std::span<const float> ranges, double angle_min, double angle_increment,
                      double range_min, double range_max, double center, double width,
                      double minimum_valid_fraction = 0.6);

class GridMaze {
 public:
  static constexpr int unreachable = 1000000;
  GridMaze(int columns, int rows);
  bool contains(Cell cell) const;
  int index(Cell cell) const;
  Edge edge(Cell cell, Direction direction) const;
  void set_edge(Cell cell, Direction direction, Edge state);
  void visit(Cell cell);
  int visits(Cell cell) const;
  std::vector<int> flood(Cell goal, bool allow_unknown = true) const;
  std::vector<Cell> route(Cell start, Cell goal, Direction heading,
                          bool allow_unknown = true) const;
  int columns() const { return columns_; }
  int rows() const { return rows_; }
  int known_edges() const;

 private:
  int columns_;
  int rows_;
  std::vector<std::array<Edge, 4>> edges_;
  std::vector<int> visits_;
};

struct FollowParameters {
  double heading_gain{1.6};
  double heading_damping{0.25};
  double lateral_gain{1.8};
  double centering_gain{0.8};
  double side_wall_max{0.70};
  double desired_side_distance{0.44};
  double max_centering_correction{0.15};
  double max_angular_speed{0.6};
};

double centering_error(const Sectors & sectors, const FollowParameters & parameters);
double follow_angular(double heading_error, double lateral_error, const Sectors & sectors,
                      const FollowParameters & parameters, double measured_angular_speed = 0.0);

}  // namespace maze_mouse_navigation
