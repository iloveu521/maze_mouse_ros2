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

硬膨胀设为 **车体包络半径 0.14 m + 额外间隙 0.04 m = 0.18 m**，软膨胀已关闭。0.88 m 通道内，扣除两侧硬膨胀后供车体中心选择的名义宽度为 0.52 m。局部滚动地图扩至 8×8 m，覆盖 3.5 m 雷达扫描的完整圆形范围，分辨率保持 0.04 m。

## 连续控制

1. 沿拟合后的连续左墙生成偏置参考曲线，并保存行进方向；右侧出现岔路时继续沿左墙通过，恢复避障后也回到原来的行进方向。若扫描边界短暂跳到反向轮廓，会拒绝该候选并继续跟随有效缓存路径；只有确认死胡同才执行掉头。控制器不加载 world、迷宫连通图或预设解答。
2. 参考曲线放入滚动缓存。每次新扫描检查当前前方路径，仍安全时保留原曲线；接近曲线尾端时，才将新扫描生成的曲线拼到旧曲线尾端。新障碍挡住当前曲线时立即重置受影响段。
3. Pure Pursuit 沿曲线弧长取自适应前瞻点，曲线曲率提供转向前馈，横向与航向 PID 修正里程计跟踪误差；角速度经滤波与变化率限制。普通通道前瞻上限为 0.90 m，弯道按预测曲率限速。
4. 绿色参考曲线是跟踪目标。控制器围绕 Pure Pursuit 转向指令筛选通过碰撞检查的速度和角速度，并预测反应延迟及制动过程；短时指令轨迹只作安全校验，不作为新的路径目标，也不在 RViz 默认显示。
5. 前墙在 2.35 m 内可见时，控制器结合拟合墙段和两侧扇区检查是否为封闭走廊。若一侧有出口，保留墙体轮廓寻迹来通过岔路；只有两侧受限且前方封闭时，才原地旋转 180°，随后沿已观测通道退出。
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
ros2 param set /maze_navigator soft_cost_radius 0.0
ros2 param load /maze_navigator src/maze_mouse_navigation/config/costmap.yaml
```

速度、运动模型、膨胀、局部目标、采样数量及评分权重均可热更新。速度和膨胀修改保留已有地图与完成状态；修改地图尺寸、分辨率或雷达外参会清空地图，等待下一帧有效扫描。起点坐标、扇区宽度和超时只能重启修改。参数先校验再应用，拒绝负余量等非法设置；YAML 使用完整节点名 `/maze_navigator`，可同时用于 launch 和 `ros2 param load`。相互约束的参数需要通过原子参数服务一起修改。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `max_linear_speed` | 0.5 m/s | Gazebo 复测配置；转弯和避障时自动降低 |
| `max_angular_speed` | 1.8 rad/s | 角速度上限 |
| `pursuit_lookahead` / `pursuit_lookahead_gain` / `pursuit_lookahead_max` | 0.60 m / 0.35 s / 0.90 m | 随速度增加的几何前瞻及上限，沿曲线弧长取点 |
| `path_preview_distance` | 1.20 m | 路径曲率前馈的预览长度 |
| `path_refresh_distance` / `path_validation_distance` | 0.90 m / 1.50 m | 路径尾部接续距离与新扫描障碍检查距离 |
| `heading_kp` / `heading_ki` / `heading_kd` | 0.75 / 0.02 / 0.16 | 航向误差 PID，积分限幅、微分滤波 |
| `lateral_gain` / `curvature_feedforward` | 0.40 / 1.0 | 横向误差修正与路径曲率前馈 |
| `max_lateral_acceleration` | 0.65 m/s² | 预测弯道线速度上限 |
| `angular_command_acceleration` / `angular_filter_time_constant` | 2.5 rad/s² / 0.18 s | 角速度指令变化限制和一阶滤波 |

| `costmap_resolution` / `costmap_size` | 0.04 m / 8.0 m | 滚动地图精度与边长；覆盖 3.5 m 雷达量程 |
| `robot_radius` | 0.14 m | 车体圆形包络半径 |
| `inflation_margin` | 0.04 m | 在车体之外增加的硬安全间隙 |
| `soft_cost_radius` | 0.0 m | 软膨胀关闭；仅使用硬碰撞边界 |
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
| `dead_end_distance` / `dead_end_front_gap` / `dead_end_side_opening` | 3.0 m / 0.20 m / 0.75 m | 在雷达有效范围内提前识别封闭走廊；有侧向开口时沿墙体轮廓选择支路 |
| `u_turn_speed` / `u_turn_lookahead` | 0.22 m/s / 0.20 m | 原地旋转后的退出速度上限与路径前瞻 |
| `data_timeout` | 0.7 s | 传感器超时 |

控制器围绕绿色滚动参考曲线的 Pure Pursuit 转向指令做短时安全筛选，RViz 默认不显示被选中的短时指令轨迹，只显示绿色参考路径和蓝色实际轨迹。缓存路径在前方仍有效时保持不变，接近尾部时与新扫描生成的局部路径拼接，发现前方路径被新障碍挡住则重置该段。

滚动路径与死胡同掉头的 Gazebo 结果见[验证记录](docs/rolling_path_tuning.md)。

默认 Gazebo 世界为 `maze_mouse_gazebo/worlds/maze.world`；起点为 `(-4.50,-3.44,0)`，出口目标为 `(5.45,3.49)`。起点与首次里程计位姿建立对应。更换机器人时需要同步修改车体包络、激光安装参数和运动模型。

## 观察与验证

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/cmd_vel` | Twist | 实际速度指令 |
| `/maze_mouse/obstacles` | OccupancyGrid | 雷达占据地图 |
| `/maze_mouse/costmap` | OccupancyGrid | 仅显示硬碰撞边界膨胀，RViz 默认显示 |
| `/maze_mouse/path` | Path | 当前 Pure Pursuit 滚动参考路径（兼容话题） |
| `/maze_mouse/reference` | Path | 当前 Pure Pursuit 滚动参考路径，RViz 绿色 |
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
