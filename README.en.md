# Maze Mouse ROS 2

中文 | [English](README.en.md)

A maze-navigation project built with **ROS 2 Humble, C++20, Gazebo Classic 11 and the
TurtleBot3 Burger**. The robot builds an occupancy grid from `/scan` and `/odom`, explores
unknown space with frontiers, plans through observed free space with clearance-aware A*, and
tracks the path with Pure Pursuit.

> This is a simulation and course-experiment project, not a complete navigation stack for
> deployment on a physical robot.

## Highlights

- Online occupancy-grid mapping without loading a maze graph or preset route.
- 8-connected A* with distance-field costs that favour corridor centres.
- Frontier exploration when the goal is unknown or not yet reachable.
- Pure Pursuit tracking with curvature, clearance and predicted-collision speed limits.
- A complete validation run on the default maze: `COMPLETE`, 103.4 s, 60.3 m, maximum measured
  speed 0.851 m/s, minimum wall clearance 0.240 m, no collisions.

## Quick start

### Requirements

- Ubuntu 22.04 (native Linux or WSL2)
- ROS 2 Humble
- Gazebo Classic 11
- CMake 3.22+ and a C++20 compiler

Install dependencies:

```bash
sudo apt update
sudo apt install gazebo ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3 ros-humble-turtlebot3-simulations
```

### Build

```bash
cd maze_mouse_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to maze_mouse_navigation
source install/setup.bash
```

Use `setup.zsh` instead of `setup.bash` in Zsh.

### Run the full simulation

```bash
ros2 launch maze_mouse_navigation navigation.launch.py
```

For servers or automated validation:

```bash
ros2 launch maze_mouse_navigation navigation.launch.py gui:=false rviz:=false
```

To launch only Gazebo and the robot:

```bash
ros2 launch maze_mouse_gazebo maze.launch.py
```

Run only one simulation instance at a time because Gazebo uses a default master port.

## Architecture

```text
Gazebo /scan + /odom
        │
        ▼
LocalCostmap ──► GlobalPlanner ──► CostmapFollower ──► /cmd_vel ──► Gazebo
  fixed grid        A* / frontier      Pure Pursuit
  ray clearing      clearance cost     collision sampler / speed governor
        │                                   │
        └──────── /maze_mouse/plan ─────────┴──► RViz2
                  /reference /trajectory /status
```

![System architecture](docs/architecture.svg)

| Component | Location | Responsibility |
| --- | --- | --- |
| `LocalCostmap` | `src/maze_mouse_navigation/src/local_costmap.cpp` | Fixed grid, ray clearing, occupancy evidence, distance field and clearance queries |
| `GlobalPlanner` | `src/maze_mouse_navigation/src/global_planner.cpp` | A*, frontier selection, corner-cut rejection and path smoothing |
| `CostmapFollower` | `src/maze_mouse_navigation/src/local_costmap.cpp` | Pure Pursuit, collision prediction, curvature/clearance limits and recovery |
| `costmap_node` | `src/maze_mouse_navigation/src/costmap_node.cpp` | Sensor subscriptions, planning, control and status publishing |
| Gazebo environment | [`src/maze_mouse_gazebo`](src/maze_mouse_gazebo/) | Maze world, robot, TF, Gazebo and RViz launch |

Planning and tracking share one fixed map. Unknown cells are not traversable by A*; when the goal
is not yet reachable, the planner selects a frontier instead. See the
[navigation package README](src/maze_mouse_navigation/README.md) for design parameters.

## Simulation and robot model

The default world is `src/maze_mouse_gazebo/worlds/maze.world`, approximately 10 m × 10 m. The
robot starts at `(-4.50, -3.44)` and the default goal is `(5.45, 3.49)`.

The project uses the **TurtleBot3 Burger**. Its model is provided by the installed ROS 2
`turtlebot3_gazebo` package rather than copied or redistributed in this repository. The
[`maze.launch.py`](src/maze_mouse_gazebo/launch/maze.launch.py) launch file sets
`TURTLEBOT3_MODEL=burger` and reads:

- `urdf/turtlebot3_burger.urdf`
- `models/turtlebot3_burger/model.sdf`

The upstream source is [ROBOTIS-GIT/turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble/turtlebot3_gazebo).
This project uses its differential drive, 360-degree lidar and wheel odometry through `/cmd_vel`,
`/scan` and `/odom`. The model, meshes, materials and Gazebo plugins remain subject to the
licenses supplied by the upstream packages.

See the [Gazebo package README](src/maze_mouse_gazebo/README.md) for world generation, geometry
checks and standalone launch options.

## Topics and parameters

| Topic | Type | Purpose |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | 360-degree lidar |
| `/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/maze_mouse/costmap` | `nav_msgs/OccupancyGrid` | Hard collision boundary |
| `/maze_mouse/plan` | `nav_msgs/Path` | Global plan |
| `/maze_mouse/reference` | `nav_msgs/Path` | Current tracking reference |
| `/maze_mouse/trajectory` | `nav_msgs/Path` | Actual trajectory |
| `/maze_mouse/status` | `std_msgs/String` | State, target, clearance and planner timing |

Defaults are in [`costmap.yaml`](src/maze_mouse_navigation/config/costmap.yaml). Common
parameters include `max_linear_speed`, `robot_radius`, `planning_margin`,
`clearance_cost_weight` and `replan_period`. The navigation package README contains the full
parameter reference.

## Repository layout

```text
maze_mouse_ros2/
├── README.md
├── README.en.md
├── docs/                         # Architecture, maze diagrams and validation notes
└── src/
    ├── maze_mouse_gazebo/        # Gazebo world, Burger, TF and RViz
    └── maze_mouse_navigation/    # C++20 A*, frontier exploration and Pure Pursuit
```

## Validation and tests

Run the C++ tests:

```bash
colcon test --packages-select maze_mouse_navigation
colcon test-result --verbose
```

Check maze topology and collision geometry:

```bash
python3 src/maze_mouse_gazebo/scripts/validate_maze.py
```

Detailed metrics, parameters and reproduction commands are in
[`docs/validation.md`](docs/validation.md).

## Limitations

- Mapping is odometry-only and accumulates drift over long runs; no SLAM is included.
- Online exploration can enter an unobserved dead end and relies on the next global replan to exit.
- Gazebo accepts higher command speeds than a physical TurtleBot3 Burger; the stock official
  teleop limit is about 0.22 m/s, so simulation speed is not real-robot performance.
- `left_wall_follower` and `maze_navigator` are retained only as historical comparisons.

## License and attribution

This repository's code and documentation are Apache-2.0 licensed; see [LICENSE](LICENSE).
TurtleBot3 models and Gazebo plugins come from the installed ROBOTIS simulation packages and
remain subject to their upstream copyrights and licenses.
