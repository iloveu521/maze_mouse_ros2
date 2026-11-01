# C++20 左墙跟随：直行目标 0.5 m/s

默认启动使用左墙跟随控制器 `left_wall_follower`，直行目标速度为 **0.5 m/s**。感知、决策、路口定位与速度控制均为 C++20，使用 CMake 3.22 构建。Python launch 仅编排进程。

控制器订阅 `/scan` 与 `/odom`，不读取 world 墙体、预设参考路线或泛洪距离场。起点、出口坐标与单元间距由参数提供，决策优先级是 **左转 → 直行 → 右转 → 死胡同掉头**。

## 编译启动

工作空间根目录，Zsh：

```zsh
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-up-to maze_mouse_navigation
source install/setup.zsh
ros2 launch maze_mouse_navigation navigation.launch.py
```

Bash 对应使用 `setup.bash`。无界面运行追加 `gui:=false rviz:=false`。已有仿真时使用 `simulation:=false`，机器人应停在配置起点且朝向正确，同一时刻由一个程序发布 `/cmd_vel`。

旧泛洪控制器保留用于对照，默认不运行：

```zsh
ros2 launch maze_mouse_navigation navigation.launch.py controller:=maze_navigator
```

启动文件分别选择 `config/left_wall.yaml` 和旧版 `config/navigation.yaml`，也可用 `params:=绝对路径` 指定配置。旧版说明见 [泛洪控制器](docs/flood_fill_controller.md)。

## 雷达与控制

- 前方扇区 `[-15°,15°]`、左侧 `[75°,105°]`、右侧 `[-105°,-75°]`。根据扫描消息角度选点，使用投影中位距离；原始最小测距单独用于停车。
- 左侧距离误差提供贴墙修正，里程计航向和角速度用于保持方向及抑制摆动。左侧开口处减速，并用不同扫描帧确认，避免把同一帧重复计数。
- 确认左侧开口后优先左转；前方受阻而右侧开放时右转；三侧受阻时掉头。接近墙角但侧向开口尚未显现时先低速接近，避免过早把路口判成死胡同。
- 需要转弯时减速到当前路口中心，确认车体停稳后，利用里程计航向及角速度反馈原地转向。连续确认转角、线速度和角速度稳定后，再等待新雷达帧并前进。使用里程计格号对路口去重，只利用 1 m 单元间距，不存储迷宫连通图或搜索路线。
- 不再每走一格就停车观测。直道目标 0.5 m/s；开口确认、路口接近阶段速度上限为 0.20 m/s，随后逐步减速。转弯、死胡同掉头和最终到达出口会停车。
- 到达出口区域后使用里程计接近配置目标并停车；雷达无效、数据超时立即输出零速度，过近障碍、路口定位偏差或动作超时进入 `FAULT`。

0.5 m/s 是本次 **Gazebo 仿真速度实验** 的控制配置，超出 Burger 实车的标称线速度范围；本次结果不作为实车速度能力的结论。

## 参数与话题

默认参数见 [left_wall.yaml](config/left_wall.yaml)：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `max_linear_speed` | 0.5 m/s | 直行目标上限 |
| `approach_speed` | 0.20 m/s | 路口接近与开口确认速度上限 |
| `desired_left_distance` | 0.40 m | 雷达到左墙的期望距离 |
| `cell_size` | 1.0 m | 路口定位与去重的单元间距 |
| `opening_distance` | 0.78 m | 侧向开口判定距离 |
| `opening_scans` | 2 | 开口确认所需不同扫描帧数 |
| `front_turn_distance` | 0.82 m | 开始考虑前方转弯的距离 |
| `front_stop_distance` | 0.20 m | 前方紧急停车距离 |
| `max_angular_speed` | 0.6 rad/s | 角速度指令上限 |
| `data_timeout` | 0.7 s | 雷达/里程计接收超时 |

参数在启动时读取，起点对应首次静止里程计位姿。默认起点 `(0.5,0.5,0)`，出口外目标 `(6.45,5.5,0)`。

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| `/scan`、`/odom` | LaserScan、Odometry | 实际传感器输入 |
| `/cmd_vel` | geometry_msgs/Twist | 速度指令 |
| `/maze_mouse/sectors` | geometry_msgs/Vector3Stamped | x=前、y=左、z=右的投影中位距离 |
| `/maze_mouse/status` | std_msgs/String | JSON：模式、位置、测距、转弯次数、速度与耗时 |
| `/maze_mouse/path` | nav_msgs/Path | 当前路口接近目标，RViz 黄色 |
| `/maze_mouse/trajectory` | nav_msgs/Path | 实际里程计轨迹，RViz 蓝色 |

`max_command_speed` 是运行期间的最大线速度指令，`max_measured_speed` 是里程计报告的最大前向速度绝对值，两者分别记录。状态及路径均为控制器实际运行输出。

```zsh
ros2 topic echo /maze_mouse/status --full-length
colcon test --packages-select maze_mouse_gazebo maze_mouse_navigation
colcon test-result --verbose
```

## 验证与范围

测试包含左优先、开口多帧确认、直行 0.5 m/s、无效数据停车、右转/掉头和出口停车，并用真实 world 的碰撞几何、5 Hz 射线扫描进行完整运动学检查。运动学检查与 Gazebo 动力学实测分开记录，结果见 [左墙跟随验证](docs/left_wall_validation.md)。旧版 `docs/validation.md` 只记录泛洪实验，不用于描述本版本效果。

实现入口：`include/maze_mouse_navigation/left_wall.hpp`、`src/left_wall.cpp` 和 `src/left_wall_node.cpp`。该版本面向当前轴对齐格网迷宫，不包含 SLAM；左墙策略不保证最短路线，在存在独立墙岛的迷宫中也不保证到达任意目标。

本功能为 AI 辅助实现；课程材料请按任务书记录实际使用范围、人工修改与实测过程。
