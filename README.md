# Maze Mouse ROS 2

[English](README.en.md) | 中文

基于 **ROS 2 Humble、C++20、Gazebo Classic 11 和 TurtleBot3 Burger** 的迷宫自主导航项目。
机器人仅使用激光雷达 `/scan` 和轮式里程计 `/odom` 在线建图，在未知区域使用 frontier
探索，在已知区域使用带净空代价的全局 A* 规划，并由 Pure Pursuit 控制器跟踪路径。

> 这是一个面向课程实验和算法对比的仿真项目，不是面向真实机器人部署的完整导航栈。

## 项目亮点

- 在线占据栅格建图，不读取迷宫连通图或预设路线。
- 8 邻接 A* 规划，使用距离场代价偏好走廊中心。
- 目标不可达或尚未观测时，自动选择 frontier 继续探索。
- Pure Pursuit 路径跟踪，结合曲率、净空和预测碰撞进行速度限制。
- 默认迷宫完成过完整验证：`COMPLETE`、103.4 s、60.3 m、最大实测速度 0.851 m/s、
  最小墙净空 0.240 m、无碰撞。

## 快速开始

### 环境

- Ubuntu 22.04（原生 Linux 或 WSL2）
- ROS 2 Humble
- Gazebo Classic 11
- CMake 3.22+ 和 C++20 编译器

安装依赖：

```bash
sudo apt update
sudo apt install gazebo ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3 ros-humble-turtlebot3-simulations
```

### 编译

```bash
cd maze_mouse_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to maze_mouse_navigation
source install/setup.bash
```

如果使用 Zsh，将 `setup.bash` 替换为 `setup.zsh`。

### 启动完整仿真

```bash
ros2 launch maze_mouse_navigation navigation.launch.py
```

服务器或自动验证使用无界面模式：

```bash
ros2 launch maze_mouse_navigation navigation.launch.py gui:=false rviz:=false
```

只启动 Gazebo 世界和机器人：

```bash
ros2 launch maze_mouse_gazebo maze.launch.py
```

同一时刻只运行一个使用默认 Gazebo master 端口的仿真实例。

## 系统架构

```text
Gazebo /scan + /odom
        │
        ▼
LocalCostmap ──► GlobalPlanner ──► CostmapFollower ──► /cmd_vel ──► Gazebo
  固定栅格          A* / frontier      Pure Pursuit
  射线清除          净空代价           碰撞采样 / 限速
        │                                   │
        └──────── /maze_mouse/plan ─────────┴──► RViz2
                  /reference /trajectory /status
```

![系统架构](docs/architecture.svg)

| 组件 | 位置 | 作用 |
| --- | --- | --- |
| `LocalCostmap` | `src/maze_mouse_navigation/src/local_costmap.cpp` | 固定栅格、射线清除、占据累积、距离场和净空查询 |
| `GlobalPlanner` | `src/maze_mouse_navigation/src/global_planner.cpp` | A*、frontier 选择、禁止切角和路径平滑 |
| `CostmapFollower` | `src/maze_mouse_navigation/src/local_costmap.cpp` | Pure Pursuit、碰撞预测、曲率/净空限速和脱困 |
| `costmap_node` | `src/maze_mouse_navigation/src/costmap_node.cpp` | 订阅传感器、规划、控制和发布状态 |
| Gazebo 环境 | [`src/maze_mouse_gazebo`](src/maze_mouse_gazebo/) | 迷宫世界、机器人、TF、Gazebo 和 RViz 启动 |

设计上使用一张固定地图同时服务规划和跟踪；未知格不参与 A*，目标尚不可达时改为规划到
frontier。完整设计参数见 [导航包 README](src/maze_mouse_navigation/README.md)。

## 仿真环境与机器人模型

默认世界是 `src/maze_mouse_gazebo/worlds/maze.world`，约 10 m × 10 m。机器人出生点为
`(-4.50, -3.44)`，默认目标为 `(5.45, 3.49)`。

项目使用 **TurtleBot3 Burger**，模型由 ROS 2 的 `turtlebot3_gazebo` 软件包提供，不是本仓库
复制或重新分发的资源。启动文件
[`maze.launch.py`](src/maze_mouse_gazebo/launch/maze.launch.py) 会设置
`TURTLEBOT3_MODEL=burger`，并从已安装的软件包读取：

- `urdf/turtlebot3_burger.urdf`
- `models/turtlebot3_burger/model.sdf`

模型来源为 [ROBOTIS-GIT/turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble/turtlebot3_gazebo)。
本项目使用其差速驱动、360° 激光雷达和轮式里程计，通过 `/cmd_vel`、`/scan` 和 `/odom`
与模型交互。模型、网格、材质和 Gazebo 插件遵循上游软件包附带的许可证。

更多世界生成、几何检查和单独启动方式见
[Gazebo 包 README](src/maze_mouse_gazebo/README.md)。

## 话题与参数

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | 360° 激光雷达 |
| `/odom` | `nav_msgs/Odometry` | 轮式里程计 |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令 |
| `/maze_mouse/costmap` | `nav_msgs/OccupancyGrid` | 硬碰撞边界 |
| `/maze_mouse/plan` | `nav_msgs/Path` | 全局规划路径 |
| `/maze_mouse/reference` | `nav_msgs/Path` | 当前跟踪参考 |
| `/maze_mouse/trajectory` | `nav_msgs/Path` | 实际轨迹 |
| `/maze_mouse/status` | `std_msgs/String` | 状态、目标、净空和规划耗时 |

默认参数见 [`costmap.yaml`](src/maze_mouse_navigation/config/costmap.yaml)。常用参数包括：
`max_linear_speed`、`robot_radius`、`planning_margin`、`clearance_cost_weight` 和
`replan_period`。完整参数说明见导航包 README。

## 目录结构

```text
maze_mouse_ros2/
├── README.md
├── README.en.md
├── docs/                         # 架构图、迷宫图和验证记录
└── src/
    ├── maze_mouse_gazebo/        # Gazebo 世界、Burger、TF、RViz
    └── maze_mouse_navigation/    # C++20 A*、frontier 和 Pure Pursuit
```

## 验证与测试

运行 C++ 单元测试：

```bash
colcon test --packages-select maze_mouse_navigation
colcon test-result --verbose
```

静态检查迷宫拓扑和碰撞几何：

```bash
python3 src/maze_mouse_gazebo/scripts/validate_maze.py
```

默认世界的详细指标、参数和复现命令见 [`docs/validation.md`](docs/validation.md)。

## 已知限制

- 仅使用里程计建图，长时间运行会产生累计漂移；项目没有接入 SLAM。
- 在线探索可能进入未观测死胡同，由下一次全局重规划退出。
- Gazebo 可接受较高速度指令，但真实 TurtleBot3 Burger 的官方遥控上限约为 0.22 m/s；
  仿真速度不能直接视为实车性能。
- `left_wall_follower` 和 `maze_navigator` 仅作为历史对照保留。

## 许可证与来源

本仓库代码和文档采用 Apache-2.0，详见 [LICENSE](LICENSE)。TurtleBot3 模型和 Gazebo 插件
来自 ROBOTIS 的已安装仿真包，版权和许可证归其上游项目所有。
