# 雷达局部 costmap 与连续左墙跟随

默认控制器 `costmap_follower` 使用 **C++20、CMake 3.22**。输入是 `/scan` 和 `/odom`，速度及轨迹评分参数支持运行中修改。转弯位置由扫描到的墙体和预测轨迹决定，不要求到达迷宫单元中心或停稳后再转弯。

```zsh
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-up-to maze_mouse_navigation
source install/setup.zsh
ros2 launch maze_mouse_navigation navigation.launch.py
```

Bash 使用 `setup.bash`。无界面追加 `gui:=false rviz:=false`；已有仿真使用 `simulation:=false`。同一时刻只运行一个速度控制器，机器人从配置起点静止启动。

## 感知与安全空间

前方使用 `[-45°,45°]` 的 90° 扇区，左右分别使用 `[60°,120°]`、`[-120°,-60°]` 的 60° 扇区，输出过滤后的墙距、原始最小距离和有效性。局部建图始终使用完整 360° 扫描，以保留斜向墙角及转弯后的障碍信息。

连续扫描点先按间距聚类，再用递归直线拟合拆成墙段。长度小于 0.24 m 的墙体厚度端面不参与左墙方向和扇区墙距计算，但仍完整写入障碍 costmap。这样岔路旁的短端面不会把参考方向拉向缺口，碰撞检查也不会漏掉真实墙角。

扫描点按激光安装偏移和里程计转换到局部固定坐标，射线清除可见自由空间，命中点累积占据证据；地图随车辆滚动，重叠区域保留。未知区域不参与正常路径规划。

墙体膨胀包含 **车体包络半径 0.14 m + 额外间隙 0.04 m = 0.18 m**。软代价范围从 0.55 m 缩至 0.38 m，地图分辨率从 0.05 m 改为 0.04 m。0.88 m 通道内，扣除两侧硬膨胀后供车体中心选择的名义宽度从 0.40 m 增至 0.52 m；离散化与距离插值仍额外采用保守补偿。

## 连续控制

1. 沿拟合后的连续左墙生成偏置参考线，并保存行进方向；右侧出现岔路时继续沿左墙通过，恢复避障后也回到原来的行进方向。控制器不加载 world、迷宫连通图或预设解答。
2. 在局部 costmap 上使用 A* 为这个局部目标绕开障碍，再检查并简化路径。这里的 4 cm 网格只是障碍表示精度，不是 1 m 迷宫单元，也不要求车辆逐点停靠。
3. 前瞻距离随车速从 0.60 m 增至最多 0.90 m；若前瞻点与车辆之间的直线穿过膨胀墙面，控制器会缩短前瞻点，避免隔墙追逐弯后路径。路径曲率提供转向前馈，横向与航向 PID 负责误差修正，积分限幅并过滤角速度目标，降低连续反向纠偏。
4. 采样线速度与角速度组合，预测加减速、执行延迟及停车过程，剔除碰撞轨迹，再按前进进度、路径偏差、航向和墙边代价选取速度。前视曲率限制弯道速度，角速度变化率限制控制跳变；转弯中可以同时前进和转向。
5. 当 90° 前方扇区及拟合墙段确认前墙和两侧长墙时，控制器在约 1 m 外生成进入曲线、半圆和退出曲线，并用 costmap 检查整条曲线。无法满足掉头间隙时才回退到普通轮廓避障。
6. 车辆接近膨胀区时进入 `RECOVER_CLEARANCE`，低速朝更宽敞方向恢复间隙。当前位置进入硬膨胀边缘时，只有当后方已观测、整段间隙不减且终点间隙增加，才短暂倒退脱困；若找不到这样的后退路径则停车。雷达无效、数据过期立即输出零速度；持续 15 秒无平移进入 `FAULT`。到达配置出口后停车。

这是左墙策略加局部避障，不执行全局泛洪搜索，不保证最短路。当前版本用里程计维护局部地图，未接入 SLAM 或全局定位，长时间累计漂移仍会影响出口判定。速度上限是指令约束，实际速度和完成时间分别由 Gazebo 里程计及状态话题记录，见 [完整调参记录](docs/speed_tuning/README.md)。

## 参数

默认配置见 [costmap.yaml](config/costmap.yaml)，启动时可用 `params:=绝对路径` 替换。

保留原来的 0.5 m/s、0.10 m 额外膨胀配置用于对照：

```zsh
ros2 launch maze_mouse_navigation navigation.launch.py params:=$PWD/src/maze_mouse_navigation/config/costmap_conservative.yaml
```

运行中修改示例（单位 m/s、rad/s、m）：

```zsh
ros2 param set /maze_navigator max_linear_speed 1.0
ros2 param set /maze_navigator max_angular_speed 1.8
ros2 param set /maze_navigator inflation_margin 0.04
ros2 param load /maze_navigator src/maze_mouse_navigation/config/costmap.yaml
```

速度、运动模型、膨胀、局部目标、采样数量及评分权重均可热更新。速度和膨胀修改保留已有地图与完成状态；修改地图尺寸、分辨率或雷达外参会清空地图，等待下一帧有效扫描。起点坐标、扇区宽度和超时只能重启修改。参数先校验再应用，拒绝负余量等非法设置；YAML 使用完整节点名 `/maze_navigator`，可同时用于 launch 和 `ros2 param load`。相互约束的参数需要通过原子参数服务一起修改。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `max_linear_speed` | 0.8 m/s | Gazebo 复测配置；转弯和避障时自动降低 |
| `max_angular_speed` | 1.8 rad/s | 角速度上限 |
| `pursuit_lookahead` / `pursuit_lookahead_gain` / `pursuit_lookahead_max` | 0.60 m / 0.35 s / 0.90 m | 随速度增加的几何前瞻及上限；遇到墙会缩短 |
| `path_preview_distance` | 1.20 m | 路径曲率前馈的预览长度 |
| `heading_kp` / `heading_ki` / `heading_kd` | 0.75 / 0.02 / 0.16 | 航向误差 PID 增益，积分带限幅，微分经过滤波 |
| `lateral_gain` / `curvature_feedforward` | 0.40 / 1.0 | 横向误差修正与路径曲率前馈权重 |
| `max_lateral_acceleration` | 0.65 m/s² | 预测弯道线速度上限 |
| `angular_command_acceleration` / `angular_filter_time_constant` | 2.5 rad/s² / 0.18 s | 角速度指令变化限制和一阶滤波 |
| `costmap_resolution` / `costmap_size` | 0.04 m / 4.0 m | 滚动地图精度与边长 |
| `robot_radius` | 0.14 m | 车体圆形包络半径 |
| `inflation_margin` | 0.04 m | 在车体之外增加的墙边间隙 |
| `soft_cost_radius` | 0.38 m | 渐变墙边代价的范围 |
| `desired_left_distance` | 0.28 m | 用于追踪左墙方向的偏置轮廓；非强制行驶距离 |
| `prediction_horizon` / `reaction_time` | 0.75 s / 0.20 s | 轨迹预测与延迟预算 |
| `linear_acceleration` / `linear_deceleration` | 0.8 / 1.0 m/s² | 预测模型的加减速能力 |
| `angular_acceleration` | 3.0 rad/s² | 预测模型的角加速度 |
| `target_lookahead` / `target_tolerance` | 1.05 m / 0.20 m | 轮廓前瞻弧长及局部目标到达容差 |
| `wall_target_offset` | 0.10 m | 将局部目标向墙外偏移的距离 |
| `recovery_enter_margin` / `recovery_exit_margin` | 0.015 / 0.035 m | 在硬膨胀半径之外的恢复模式进入/退出门限 |
| `recovery_speed` | 0.30 m/s | 恢复间隙时的速度上限 |
| `heading_stop_angle` | 1.20 rad | 超过此路径航向偏差时原地转向 |
| `linear_samples` / `angular_samples` | 6 / 30 | 速度采样区间数，另包含边界及引导角速度 |
| `progress_weight` / `path_weight` / `heading_weight` | 4.0 / 1.5 / 0.7 | 前进、路径偏差与航向评分权重 |
| `clearance_weight` / `speed_weight` | 3.0 / 1.0 | 近墙惩罚与速度偏好权重 |
| `laser_offset_x` | -0.032 m | Burger 激光相对车体中心的纵向偏移 |
| `side_sector_width_deg` / `front_sector_width_deg` | 60° / 90° | 左右墙面与前方死胡同观察范围 |
| `minimum_wall_length` / `wall_fit_tolerance` | 0.24 m / 0.025 m | 可跟随长墙的最短长度与直线拟合容差 |
| `dead_end_distance` / `dead_end_front_gap` | 1.15 m / 0.45 m | 提前确认死胡同与掉头曲线的前方间隙 |
| `u_turn_speed` / `u_turn_lookahead` | 0.22 m/s / 0.12 m | 连续掉头曲线的速度与前瞻 |
| `data_timeout` | 0.7 s | 传感器超时 |

默认起点 `(0.5,0.5,0)`，出口目标 `(6.45,5.5)`，与首次里程计位姿建立对应。更换机器人时需要同步修改车体包络、激光安装参数和运动模型。

## 观察与验证

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/cmd_vel` | Twist | 实际速度指令 |
| `/maze_mouse/obstacles` | OccupancyGrid | 雷达占据地图 |
| `/maze_mouse/costmap` | OccupancyGrid | 膨胀区及墙边代价，RViz 默认显示 |
| `/maze_mouse/path` | Path | 当前选中的预测轨迹，黄色 |
| `/maze_mouse/reference` | Path | 拟合左墙参考线或完整掉头曲线，绿色 |
| `/maze_mouse/trajectory` | Path | 实际里程计轨迹，蓝色 |
| `/maze_mouse/sectors` | Vector3Stamped | x=前、y=左、z=右的扇区距离 |
| `/maze_mouse/status` | String | JSON：状态、速度、膨胀半径、间隙、跟踪误差、实际前瞻、目标角速度、里程、耗时及计算时间 |

```zsh
ros2 topic echo /maze_mouse/status std_msgs/msg/String --no-daemon --full-length
colcon test --packages-select maze_mouse_gazebo maze_mouse_navigation
colcon test-result --verbose
```

自动检查包括地图、旧控制器、局部 costmap 整图回归、60°/90° 扇区、短端面过滤、岔路保持左墙方向，以及正入和偏航进入死胡同的提前识别。最新 Gazebo 结果见[路径跟踪调参记录](docs/path_tracking_tuning.md)与[墙段和掉头验证](docs/scan_geometry_validation.md)；[原提速记录](docs/speed_tuning/README.md)和[原 0.5 m/s 验证](docs/costmap_validation.md)保留作历史对照。

实现：[local_costmap.hpp](include/maze_mouse_navigation/local_costmap.hpp)、[local_costmap.cpp](src/local_costmap.cpp)、[costmap_node.cpp](src/costmap_node.cpp)。Python launch 只编排进程。

## 保留的对照控制器

```zsh
ros2 launch maze_mouse_navigation navigation.launch.py controller:=left_wall_follower
ros2 launch maze_mouse_navigation navigation.launch.py controller:=maze_navigator
```

它们分别使用 `left_wall.yaml` 和 `navigation.yaml`。[旧左墙控制器](docs/left_wall_controller.md)仍按路口中心停车转向；[旧泛洪控制器](docs/flood_fill_controller.md)保留用于对照。默认均不运行。
