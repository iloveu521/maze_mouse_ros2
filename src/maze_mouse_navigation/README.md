# Maze Mouse Navigation

默认控制器是独立的 `left_wall_follower`，使用 C++20 和 CMake 3.22，面向 ROS 2 Humble、TurtleBot3 Burger 与 Gazebo Classic。控制器不加载迷宫地图或预设路线。

```zsh
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-up-to maze_mouse_navigation
source install/setup.zsh
ros2 launch maze_mouse_navigation navigation.launch.py
```

无界面运行：

```zsh
ros2 launch maze_mouse_navigation navigation.launch.py gui:=false rviz:=false
```

## 左墙跟随与路径跟踪

控制器将完整雷达扫描融合到滚动局部 costmap，并用拟合墙段和左右 60°、前方 90° 扇区识别开口。墙段长度过滤只影响方向引导，所有雷达命中仍参与障碍代价。

遇到左侧开口时，控制器生成并锁定一条经 costmap 检查的 90° 圆弧；直行段沿左墙偏置，默认目标为约 0.44 m 的墙距，接近通道中线。若开口已被雷达识别但圆弧尚未被地图验证，则沿前方以短直线逐步探测，再重新生成路径。圆弧使用切向方向、曲率前馈和横向误差反馈跟踪；直行上限 0.5 m/s，弯道默认上限 0.18 m/s，按路径曲率和剩余距离继续限速。速度与转弯半径均在 YAML 中配置。

前方路径在规划时整体检查；跟踪期间重新检查车体当前位置和前方约 0.24 m 的路径段。路径完成后再识别下一处路口，不会每个控制周期重新预测一个短期局部目标。无法生成圆弧时才尝试其他转向；掉头优先使用通过代价地图验证的圆弧，空间不足时仅在原地净空足够的情况下旋转，否则停车。

当前默认配置使用 0.12 m 车体圆形半径和 0 m 额外膨胀余量。若更换机器人或调整雷达外参，需要同步检查 `robot_radius`、`laser_offset_*` 与 `laser_yaw`。

## 参数

配置文件：[left_wall.yaml](config/left_wall.yaml)。启动时可用绝对路径替换：

```zsh
ros2 launch maze_mouse_navigation navigation.launch.py params:=/absolute/path/left_wall.yaml
```

主要参数：

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `max_linear_speed` | 0.50 m/s | 直行速度上限 |
| `turn_speed` | 0.18 m/s | 圆弧跟踪速度上限 |
| `desired_left_distance` | 0.44 m | 连续左墙参考偏置，通道中心约为此值 |
| `min_turn_radius` / `max_turn_radius` | 0.14 / 0.42 m | 90° 与掉头圆弧候选范围 |
| `pursuit_lookahead` / `pursuit_lookahead_max` | 0.55 / 0.90 m | 沿路径选取切向和曲率参考 |
| `max_lateral_acceleration` | 0.55 m/s² | 根据曲率限制弯道速度 |
| `heading_gain` / `lateral_gain` | 1.2 / 2.0 | 航向和横向路径误差反馈 |
| `angular_command_acceleration` | 2.5 rad/s² | 限制角速度指令变化 |
| `robot_radius` / `inflation_margin` | 0.12 / 0.00 m | 车体碰撞包络及附加墙距 |
| `costmap_resolution` / `costmap_size` | 0.04 / 4.0 m | 局部地图精度与边长 |
| `side_sector_width_deg` / `front_sector_width_deg` | 60° / 90° | 路口观察扇区 |

## 观察话题

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度命令 |
| `/maze_mouse/path` | `nav_msgs/Path` | 当前锁定的局部路径 |
| `/maze_mouse/trajectory` | `nav_msgs/Path` | 里程计轨迹 |
| `/maze_mouse/sectors` | `geometry_msgs/Vector3Stamped` | 前、左、右扇区距离 |
| `/maze_mouse/status` | `std_msgs/String` | JSON 状态、误差、速度、转向计数与耗时 |

其它控制器可显式选择：`controller:=costmap_follower` 或 `controller:=maze_navigator`。旧控制器和对应配置保留作比较。

## 当前验证状态

`colcon build --packages-select maze_mouse_navigation --symlink-install` 已通过。Gazebo headless 运行确认机器人可沿中心附近以 0.5 m/s 直行，并生成、执行连续左转弧线；在后续紧凑弯道仍观察到 costmap 阻塞，整张迷宫的完成运行尚未验证。相关状态通过 `/maze_mouse/status` 输出，不能将当前版本视为完整跑图已通过。
