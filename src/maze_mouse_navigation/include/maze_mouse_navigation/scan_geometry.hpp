#pragma once
#include "maze_mouse_navigation/left_wall.hpp"
#include <span>

namespace maze_mouse_navigation {
struct WallSegment {
  WallPoint a, b;
  double length() const { return std::hypot(b.x - a.x, b.y - a.y); }
};
struct ScanGeometryParameters {
  double side_degrees{60}, front_degrees{90};
  double minimum_wall_length{0.24}, fit_tolerance{0.025};
  double swept_half_width{0.18};
};
struct ScanGeometry {
  Sectors sectors;
  std::vector<WallSegment> walls;
  int short_faces{};
};
// Short faces are excluded from wall guidance, never from obstacle mapping.
ScanGeometry analyse_scan(std::span<const float> ranges, double angle_min,
                          double increment, double range_min, double range_max,
                          ScanGeometryParameters parameters = {});
} // namespace maze_mouse_navigation
