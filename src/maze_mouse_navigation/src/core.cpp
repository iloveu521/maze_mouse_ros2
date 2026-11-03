#include "maze_mouse_navigation/core.hpp"

#include <algorithm>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace maze_mouse_navigation {

double wrap_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

Direction rotate(Direction direction, int quarter_turns) {
  return static_cast<Direction>(
      (static_cast<int>(direction) + quarter_turns % 4 + 4) % 4);
}

Cell adjacent(Cell cell, Direction direction) {
  constexpr std::array<int, 4> dx{1, 0, -1, 0};
  constexpr std::array<int, 4> dy{0, 1, 0, -1};
  const auto d = static_cast<std::size_t>(direction);
  return {cell.x + dx.at(d), cell.y + dy.at(d)};
}

Direction direction_to(Cell from, Cell to) {
  for (int d = 0; d < 4; ++d) {
    if (adjacent(from, static_cast<Direction>(d)) == to) {
      return static_cast<Direction>(d);
    }
  }
  throw std::invalid_argument("Route cells must share an edge");
}

Sector extract_sector(std::span<const float> ranges, double angle_min,
                      double angle_increment, double range_min,
                      double range_max, double center, double width,
                      double minimum_valid_fraction) {
  Sector result;
  if (ranges.empty() || !std::isfinite(angle_min) ||
      !std::isfinite(angle_increment) || angle_increment == 0.0 ||
      !(range_max > range_min && range_min >= 0.0) ||
      !std::isfinite(range_max) || !std::isfinite(center) ||
      !(width > 0.0 && width <= pi) ||
      !(minimum_valid_fraction > 0.0 && minimum_valid_fraction <= 1.0)) {
    return result;
  }
  std::vector<double> distances;
  constexpr double center_half_width = 12.0 * pi / 180.0;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double delta = wrap_angle(
        angle_min + static_cast<double>(i) * angle_increment - center);
    if (std::abs(delta) > width / 2.0 + 1e-8) {
      continue;
    }
    ++result.total;
    const double raw = ranges[i];
    // +inf is a valid no-return beam, NOT a wall. NaN/-inf/out-of-range are
    // invalid.
    if (std::isinf(raw) && raw > 0) {
      distances.push_back(range_max * std::cos(delta));
      if (std::abs(delta) <= center_half_width)
        result.center_minimum_range = std::min(
            result.center_minimum_range, range_max * std::cos(delta));
      ++result.valid;
    } else if (std::isfinite(raw) && raw >= range_min && raw <= range_max) {
      result.minimum_range = std::min(result.minimum_range, raw);
      distances.push_back(raw * std::cos(delta));
      if (std::abs(delta) <= center_half_width)
        result.center_minimum_range = std::min(
            result.center_minimum_range, raw * std::cos(delta));
      ++result.valid;
    }
  }
  // Reject truncated sectors as well as sectors dominated by invalid samples.
  const double coverage =
      static_cast<double>(result.total) * std::abs(angle_increment);
  result.usable = result.total >= 3 && coverage >= width * 0.8 &&
                  static_cast<double>(result.valid) >=
                      minimum_valid_fraction * result.total;
  if (!result.usable || distances.empty()) {
    return result;
  }
  const auto middle =
      distances.begin() + static_cast<std::ptrdiff_t>(distances.size() / 2);
  std::nth_element(distances.begin(), middle, distances.end());
  result.distance = *middle;
  return result;
}

GridMaze::GridMaze(int columns, int rows) : columns_(columns), rows_(rows) {
  if (columns <= 0 || rows <= 0 || columns > 256 || rows > 256) {
    throw std::invalid_argument("Grid dimensions must be in [1, 256]");
  }
  edges_.resize(static_cast<std::size_t>(columns * rows));
  visits_.resize(edges_.size(), 0);
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < columns; ++x) {
      for (int d = 0; d < 4; ++d) {
        edges_[index({x, y})][d] =
            contains(adjacent({x, y}, static_cast<Direction>(d)))
                ? Edge::unknown
                : Edge::wall;
      }
    }
  }
}

bool GridMaze::contains(Cell cell) const {
  return cell.x >= 0 && cell.y >= 0 && cell.x < columns_ && cell.y < rows_;
}

int GridMaze::index(Cell cell) const {
  if (!contains(cell)) {
    throw std::out_of_range("Cell outside grid");
  }
  return cell.y * columns_ + cell.x;
}

Edge GridMaze::edge(Cell cell, Direction direction) const {
  return edges_.at(index(cell)).at(static_cast<std::size_t>(direction));
}

void GridMaze::set_edge(Cell cell, Direction direction, Edge state) {
  const auto next = adjacent(cell, direction);
  if (!contains(next)) {
    if (state != Edge::wall) {
      throw std::invalid_argument("Cannot open grid boundary");
    }
    return;
  }
  edges_.at(index(cell)).at(static_cast<std::size_t>(direction)) = state;
  edges_.at(index(next)).at(static_cast<std::size_t>(rotate(direction, 2))) =
      state;
}

void GridMaze::visit(Cell cell) { ++visits_.at(index(cell)); }
int GridMaze::visits(Cell cell) const { return visits_.at(index(cell)); }

std::vector<int> GridMaze::flood(Cell goal, bool allow_unknown) const {
  std::vector<int> distance(edges_.size(), unreachable);
  distance.at(index(goal)) = 0;
  std::queue<Cell> queue;
  queue.push(goal);
  while (!queue.empty()) {
    const auto cell = queue.front();
    queue.pop();
    for (int d = 0; d < 4; ++d) {
      const auto direction = static_cast<Direction>(d);
      const auto next = adjacent(cell, direction);
      const auto state = edge(cell, direction);
      if (!contains(next) || state == Edge::wall ||
          (!allow_unknown && state == Edge::unknown)) {
        continue;
      }
      if (distance[index(next)] == unreachable) {
        distance[index(next)] = distance[index(cell)] + 1;
        queue.push(next);
      }
    }
  }
  return distance;
}

std::vector<Cell> GridMaze::route(Cell start, Cell goal, Direction heading,
                                  bool allow_unknown) const {
  const auto distance = flood(goal, allow_unknown);
  if (distance.at(index(start)) == unreachable) {
    return {};
  }
  std::vector<Cell> result{start};
  while (!(result.back() == goal)) {
    const auto current = result.back();
    auto best = std::tuple{unreachable, unreachable, unreachable, unreachable};
    Direction chosen = heading;
    bool found = false;
    for (int d = 0; d < 4; ++d) {
      const auto direction = static_cast<Direction>(d);
      const auto next = adjacent(current, direction);
      const auto state = edge(current, direction);
      if (!contains(next) || state == Edge::wall ||
          (!allow_unknown && state == Edge::unknown) ||
          distance[index(next)] >= distance[index(current)]) {
        continue;
      }
      const int difference = std::abs(d - static_cast<int>(heading));
      const int turns = std::min(difference, 4 - difference);
      const auto score =
          std::tuple{distance[index(next)], visits(next), turns, d};
      if (score < best) {
        best = score;
        chosen = direction;
        found = true;
      }
    }
    if (!found) {
      return {};
    }
    result.push_back(adjacent(current, chosen));
    heading = chosen;
  }
  return result;
}

int GridMaze::known_edges() const {
  int result = 0;
  for (int y = 0; y < rows_; ++y) {
    for (int x = 0; x < columns_; ++x) {
      for (const auto d : {Direction::east, Direction::north}) {
        if (contains(adjacent({x, y}, d)) && edge({x, y}, d) != Edge::unknown) {
          ++result;
        }
      }
    }
  }
  return result;
}

double centering_error(const Sectors &sectors,
                       const FollowParameters &parameters) {
  const bool left =
      sectors.left.usable && sectors.left.distance < parameters.side_wall_max;
  const bool right =
      sectors.right.usable && sectors.right.distance < parameters.side_wall_max;
  if (left && right) {
    return (sectors.left.distance - sectors.right.distance) / 2.0;
  }
  if (left) {
    return sectors.left.distance - parameters.desired_side_distance;
  }
  if (right) {
    return parameters.desired_side_distance - sectors.right.distance;
  }
  return 0.0;
}

double follow_angular(double heading_error, double lateral_error,
                      const Sectors &sectors,
                      const FollowParameters &parameters,
                      double measured_angular_speed) {
  const double centering = std::clamp(parameters.centering_gain *
                                          centering_error(sectors, parameters),
                                      -parameters.max_centering_correction,
                                      parameters.max_centering_correction);
  return std::clamp(parameters.heading_gain * wrap_angle(heading_error) -
                        parameters.heading_damping * measured_angular_speed +
                        parameters.lateral_gain * lateral_error + centering,
                    -parameters.max_angular_speed,
                    parameters.max_angular_speed);
}

} // namespace maze_mouse_navigation
