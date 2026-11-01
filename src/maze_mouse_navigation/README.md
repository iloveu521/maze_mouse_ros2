# 雷达局部 costmap 与连续左墙跟随

默认控制器 `costmap_follower` 使用 **C++20、CMake 3.22**。输入是 `/scan` 和 `/odom`，直行速度上限 **0.5 m/s**。转弯位置由扫描到的墙体和预测轨迹决定，不要求到达迷宫单元中心或停稳后再转弯。

```zsh
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-up-to maze_mouse_navigation
source install/setup.zsh
ros2 launch maze_mouse_navigation navigation.launch.py
```

Bash 使用 `setup.bash`。无界面追加 `gui:=false rviz:=false`；已有仿真使用 `simulation:=false`。同一时刻只运行一个速度控制器，机器人从配置起点静止启动。

## 感知与安全空间

保留前方 `[-15°,15°]`、左侧 `[75°,105°]`、右侧 `[-105°,-75°]` 三个 30° 扇区，输出投影中位距离、原始最小距离和有效性。局部建图使用完整 360° 扫描，以保留斜向墙角及转弯后的障碍信息。

扫描点按激光安装偏移和里程计转换到局部固定坐标，射线清除可见自由空间，命中点累积占据证据；地图随车辆滚动，重叠区域保留。未知区域不参与正常路径规划。

墙体膨胀包含 **车体包络半径 0.14 m + 额外间隙 0.10 m = 0.24 m**，而不是仅把点状车辆与墙隔开 0.10 m。欧氏距离场生成硬障碍区和渐变软代价；离散化与距离插值额外采用保守补偿。

## 连续控制

1. 沿扫描墙体的偏置轮廓选取左墙方向的局部目标，保持轮廓方向及目标连续性，避免在死胡同两侧来回切换。控制器不加载 world、迷宫连通图或预设解答。
2. 在局部 costmap 上使用 A* 为这个局部目标绕开障碍，再检查并简化路径。这里的 5 cm 网格只是障碍表示精度，不是 1 m 迷宫单元，也不要求车辆逐点停靠。
3. 采样线速度与角速度组合，预测加减速、执行延迟及停车过程，剔除碰撞轨迹，再按前进进度、路径偏差、航向和墙边代价选取速度。转弯中可以同时前进和转向。
4. 车辆接近膨胀区时进入 `RECOVER_CLEARANCE`，低速朝更宽敞方向恢复间隙。新扫描使当前位置浅度落入膨胀边缘时，只允许不再深入的脱离轨迹；深度重叠和未知位置停车。
5. 朝向偏差大、死胡同回转或无可行弧线时可以停车旋转，停车位置不与迷宫栅格中心绑定。雷达无效、数据过期立即输出零速度；持续 15 秒无平移进入 `FAULT`。到达配置出口后停车。

这是左墙策略加局部避障，不执行全局泛洪搜索，不保证最短路。当前版本用里程计维护局部地图，未接入 SLAM 或全局定位，长时间累计漂移仍会影响出口判定。0.5 m/s 为当前 Gazebo 实验配置。

## 参数

默认配置见 [costmap.yaml](config/costmap.yaml)，启动时可用 `params:=绝对路径` 替换。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `max_linear_speed` | 0.5 m/s | 线速度上限；转弯和避障时自动降低 |
| `max_angular_speed` | 0.9 rad/s | 角速度上限 |
| `costmap_resolution` / `costmap_size` | 0.05 m / 4.0 m | 滚动地图精度与边长 |
| `robot_radius` | 0.14 m | 车体圆形包络半径 |
| `inflation_margin` | 0.10 m | 在车体之外增加的墙边间隙 |
| `soft_cost_radius` | 0.55 m | 渐变墙边代价的范围 |
| `desired_left_distance` | 0.30 m | 用于追踪左墙方向的偏置轮廓；非强制行驶距离 |
| `prediction_horizon` / `reaction_time` | 1.5 s / 0.20 s | 轨迹预测与延迟预算 |
| `linear_acceleration` / `linear_deceleration` | 0.8 / 1.0 m/s² | 预测模型的加减速能力 |
| `angular_acceleration` | 1.5 rad/s² | 预测模型的角加速度 |
| `laser_offset_x` | -0.032 m | Burger 激光相对车体中心的纵向偏移 |
| `sector_width_deg` / `data_timeout` | 30° / 0.7 s | 扇区宽度与传感器超时 |

默认起点 `(0.5,0.5,0)`，出口目标 `(6.45,5.5)`，与首次里程计位姿建立对应。更换机器人时需要同步修改车体包络、激光安装参数和运动模型。

## 观察与验证

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/cmd_vel` | Twist | 实际速度指令 |
| `/maze_mouse/obstacles` | OccupancyGrid | 雷达占据地图 |
| `/maze_mouse/costmap` | OccupancyGrid | 膨胀区及墙边代价，RViz 默认显示 |
| `/maze_mouse/path` | Path | 当前选中的预测轨迹，黄色 |
| `/maze_mouse/trajectory` | Path | 实际里程计轨迹，蓝色 |
| `/maze_mouse/sectors` | Vector3Stamped | x=前、y=左、z=右的扇区距离 |
| `/maze_mouse/status` | String | JSON：状态、速度、膨胀半径、间隙、里程、耗时及计算时间 |

```zsh
ros2 topic echo /maze_mouse/status std_msgs/msg/String --no-daemon --full-length
colcon test --packages-select maze_mouse_gazebo maze_mouse_navigation
colcon test-result --verbose
```

测试读取实际 world 碰撞几何，用 5 Hz 射线扫描和执行器延迟检查完整跑图、移动中转弯及墙边间隙。它与 Gazebo 动力学实测分别记录在 [costmap 验证](docs/costmap_validation.md)，可查看[实际轨迹与膨胀地图](docs/costmap_run.svg)。

实现：[local_costmap.hpp](include/maze_mouse_navigation/local_costmap.hpp)、[local_costmap.cpp](src/local_costmap.cpp)、[costmap_node.cpp](src/costmap_node.cpp)。Python launch 只编排进程。

## 保留的对照控制器

```zsh
ros2 launch maze_mouse_navigation navigation.launch.py controller:=left_wall_follower
ros2 launch maze_mouse_navigation navigation.launch.py controller:=maze_navigator
```

它们分别使用 `left_wall.yaml` 和 `navigation.yaml`。[旧左墙控制器](docs/left_wall_controller.md)仍按路口中心停车转向；[旧泛洪控制器](docs/flood_fill_controller.md)保留用于对照。默认均不运行。
