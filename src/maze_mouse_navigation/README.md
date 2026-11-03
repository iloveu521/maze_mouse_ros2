# 雷达全局 A* 与 Pure Pursuit 跟踪

默认控制器 `costmap_follower` 使用 **C++20、CMake 3.22**。输入是 `/scan` 和 `/odom`，
不加载 world、迷宫连通图或预设路线。控制器只做三件事：**在线建图 → 全局 A*/前沿规划 →
Pure Pursuit 跟踪与局部碰撞筛选**。

```zsh
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-up-to maze_mouse_navigation
source install/setup.zsh
ros2 launch maze_mouse_navigation navigation.launch.py
```

Bash 使用 `setup.bash`。无界面追加 `gui:=false rviz:=false`；已有仿真使用 `simulation:=false`。
同一时刻只运行一个速度控制器。

## 地图

- 固定 **14 m × 14 m、0.04 m** 的占据栅格（351² ≈ 123k 格），原点默认取起点/目标中点减去半边长；
  换 world 时可用 `costmap_origin_x/y` 覆盖，`costmap_size` 调整范围。启动时校验起点和目标都在图内。
- 每帧扫描按激光外参和里程计转换到地图坐标系：射线清除可见自由空间，命中点累积占据证据；
  未知格保持未知，不参与 A*。
- 硬膨胀仍为 **车体半径 0.14 m + 间隙 0.04 m = 0.18 m**，软膨胀关闭。
- 同一张固定图同时用于全局规划和局部跟踪净空检查，不再维护滚动窗口。

## 全局规划

1. 每次规划先构建 `free`/`clearance` 网格：`tracking_clearance > tracking_radius + planning_margin`
   （默认 0.05 m，约 0.20 m）才算可通过；对角线移动要求两个正交邻格都可通过，禁止切角。
2. 8 邻接 A*，边代价为欧氏长度乘以净空惩罚（越窄越贵，默认权重 1.0、目标净空 0.45 m），
   优先走廊中央。
3. 目标格已知可达时直接规划到目标。目标未知或不可达时，在所有可达 frontier 格
   （已知自由且 4 邻域含未知格）中按 `g + frontier_weight·distance(frontier, goal)` 选择，
   并带粘滞/切换滞回，避免在相邻前沿之间来回跳。
4. 平滑使用 free 网格上的 supercover 线段遍历 + 局部捷径；平滑后逐段用
   `tracking_line_clear` 校验，失败则退回原始网格路径。
5. 默认每 **0.5 s** 重规划一次；路径被新障碍挡住、机器人偏离路径、剩余路径小于阈值、
   目标变为可达时立即重规划。没有可达目标且没有 frontier 时进入 `NO_FRONTIER` 并停车。

## 跟踪与安全

- Pure Pursuit 沿路径弧长取自适应前瞻点，`ω = v × κ`；后续弯道只用于提前减速。
- 预测轨迹用 `tracking_clearance` 做碰撞筛选，要求始终高于跟踪半径；低于跟踪半径时
  只接受最终净空改善的弧，避免“每帧下降 2 mm”的棘轮效应。
- 窄走廊按侧向净空限速：`v ≤ sqrt(2·deceleration·(clearance - tracking_radius)) + 0.05`，
  防止高速进入 0.88 m 通道后刹不住。
- 在跟踪半径边缘且没有前进候选时，锁存一个朝较宽一侧的旋转避让方向，并允许 0.08 m/s
  低速爬出；成功后解除锁存。
- 雷达无效或数据超时立即输出零速度；持续 15 s 无平移进入 `FAULT`；到达目标
  `goal_tolerance` 后进入 `COMPLETE`。

## 参数

默认配置见 [costmap.yaml](config/costmap.yaml)，启动时可用 `params:=绝对路径` 替换。
保留 0.5 m/s 配置用于对照：

```zsh
ros2 launch maze_mouse_navigation navigation.launch.py \
  params:=$PWD/src/maze_mouse_navigation/config/costmap_conservative.yaml
```

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `max_linear_speed` / `max_angular_speed` | 1.2 m/s / 1.8 rad/s | 速度上限；窄走廊由净空限速进一步降低 |
| `linear_acceleration` / `linear_deceleration` / `angular_acceleration` | 1.0 / 1.2 / 3.0 | 预测模型；`linear_deceleration` 也用于净空停止距离限速 |
| `max_lateral_acceleration` / `angular_command_acceleration` | 0.9 m/s² / 2.5 rad/s² | 弯道横向加速度上限与角速度变化率限制 |
| `pursuit_lookahead` / `_gain` / `_max` | 0.45 m / 0.20 s / 0.70 m | 自适应 Pure Pursuit 前瞻；0.88 m 通道内更贴路径 |
| `path_preview_distance` | 1.20 m | 前方曲率预览与制动距离 |
| `path_validation_distance` / `path_deviation` | 1.50 m / 0.35 m | 跟踪参考校验距离与允许横向偏差 |
| `heading_stop_angle` | 1.2 rad | 超过该航向误差时原地对准 |
| `linear_samples` / `angular_samples` | 6 / 30 | 速度/角速度采样数量 |
| `progress_weight` / `path_weight` / `heading_weight` | 4.0 / 1.5 / 0.7 | 速度候选评分权重 |
| `clearance_weight` / `speed_weight` | 1.0 / 1.0 | 近墙惩罚与速度偏好 |
| `costmap_resolution` / `costmap_size` | 0.04 m / 14.0 m | 固定全局栅格精度与边长 |
| `costmap_origin_x` / `costmap_origin_y` | 自动居中 | 固定图左下角；省略时按 start/goal 中点计算 |
| `robot_radius` / `inflation_margin` | 0.14 m / 0.04 m | 车体包络与硬安全间隙 |
| `replan_period` | 0.5 s | 全局重规划周期 |
| `frontier_weight` | 2.5 | frontier 评分中到目标距离的权重 |
| `frontier_min_distance` | 0.40 m | 忽略机器人附近的 frontier |
| `frontier_switch_margin` / `frontier_sticky_radius` / `frontier_sticky_bonus` | 0.3 m / 0.6 m / 0.5 m | 前沿切换滞回 |
| `start_snap_radius` / `goal_snap_radius` | 0.4 m / 0.12 m | 起点/目标格量化吸附半径 |
| `planning_margin` | 0.05 m | A* 可通过格在跟踪半径之外额外保留的净空 |
| `clearance_cost_weight` / `clearance_target` | 5.0 / 0.45 m | A* 净空惩罚，强偏走廊中心 |
| `planner_path_min_remaining` / `planner_path_deviation` | 0.4 m / 0.35 m | 重规划触发阈值 |
| `planner_path_validation_distance` | 1.0 m | 前方路径被新障碍挡住的检查距离 |
| `smoothing_span` / `smoothing_clearance` | 1.5 m / 0.32 m | 捷径最大长度与必须保持的净空 |

速度、运动模型、膨胀、局部目标、采样数量、评分权重及全局规划参数均支持运行中
`ros2 param set` / `ros2 param load` 热更新。修改地图尺寸、origin、分辨率或激光外参会清空地图
并等待下一帧扫描；修改其余参数保留已建地图。

## 观察与验证

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/cmd_vel` | Twist | 实际速度指令 |
| `/maze_mouse/obstacles` | OccupancyGrid | 雷达占据地图 |
| `/maze_mouse/costmap` | OccupancyGrid | 硬碰撞边界膨胀，RViz 默认显示 |
| `/maze_mouse/plan` | Path | 全局 A* 路径（红色） |
| `/maze_mouse/reference` | Path | 裁剪后的跟踪参考（绿色） |
| `/maze_mouse/trajectory` | Path | 实际里程计轨迹（蓝色） |
| `/maze_mouse/frontier` | PointStamped | 当前 frontier 目标；无 frontier 时为 NaN |
| `/maze_mouse/status` | String | JSON：状态、目标类型、重规划次数、规划耗时、净空、路径长度等 |
| `/maze_mouse/sectors` | Vector3Stamped | 前/左/右扇区距离 |

```zsh
ros2 topic echo /maze_mouse/status std_msgs/msg/String --no-daemon --full-length
colcon test --packages-select maze_mouse_navigation
colcon test-result --verbose
```

自动检查覆盖：地图/膨胀/扇区、固定原点、跟踪净空、Pure Pursuit 预览与角速度限幅、
A* 绕墙、目标不可达、朝目标的 frontier 选择、六乘六迷宫中的 frontier 规划。
完整 Gazebo 记录与评分：

```zsh
ros2 launch maze_mouse_navigation navigation.launch.py gui:=false rviz:=false
python3 src/maze_mouse_navigation/tools/record_run.py /tmp/global_run 600
python3 src/maze_mouse_navigation/tools/score_run.py /tmp/global_run
```

实现：[global_planner.hpp](include/maze_mouse_navigation/global_planner.hpp)、
[global_planner.cpp](src/global_planner.cpp)、
[local_costmap.hpp](include/maze_mouse_navigation/local_costmap.hpp)、
[local_costmap.cpp](src/local_costmap.cpp)、[costmap_node.cpp](src/costmap_node.cpp)。

完整运行验证、轨迹图和参数记录见 [../../docs/validation.md](../../docs/validation.md)。
项目总览：[中文 README](../../README.md) · [English README](../../README.en.md)。

## 旧控制器

`left_wall_follower` 和 `maze_navigator` 保留为历史对照，默认不运行：

```zsh
ros2 launch maze_mouse_navigation navigation.launch.py controller:=left_wall_follower
ros2 launch maze_mouse_navigation navigation.launch.py controller:=maze_navigator
```

旧的墙跟随/死胡同调参记录已移除，当前默认行为由全局 A* 控制器描述。

## 说明

这是在线感知 + 局部/全局规划的方案，不保证零回头路：在线建图时仍可能进入尚未观测的死胡同，
但退出由同一次全局重规划完成，没有 dead-end/escape/u-turn 专用状态机。当前使用里程计维护地图，
未接入 SLAM；长时间累计漂移会影响固定图与出口判定。
