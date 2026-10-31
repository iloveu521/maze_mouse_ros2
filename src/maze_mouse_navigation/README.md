# C++20 雷达居中导航与泛洪搜索

所有感知、搜索、决策与控制算法均使用 C++20。Python launch 仅编排进程。

节点订阅 `/scan` 与 `/odom`，在线识别单元间的墙和通路，不读取 Gazebo world、地图生成器布局或既有中心参考轨迹。启动只需已知格网尺寸、起点、目标格及出口方向。

## 编译启动

使用 CMake 3.22（已验证 3.22.1）与支持 C++20 的编译器，本机为 GCC 11.4。

工作空间根目录，Zsh：

```zsh
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-up-to maze_mouse_navigation
source install/setup.zsh
ros2 launch maze_mouse_navigation navigation.launch.py
```

Bash 对应使用 `setup.bash`。无界面运行追加 `gui:=false rviz:=false`。已有仿真时可追加 `simulation:=false`，但机器人必须停在配置的起点，朝向与 `start_heading` 一致，且不能有其他程序同时发布 `/cmd_vel`。

## 算法

1. 雷达前方、左侧、右侧扇区默认分别为 `[-15°,15°]`、`[75°,105°]`、`[-105°,-75°]`。按照消息的 `angle_min` 和 `angle_increment` 选点，兼容跨 0° 和负角度起始扫描。
2. 对扇区内有效距离作余弦投影，再取中位数，以减轻角度和离群点的影响。前方原始最小有效距离另作紧急停车判断。`+Inf` 视为量程内未命中；NaN、`-Inf`、超出量程和覆盖不足不用于驾驶。
3. 单元中心停车并收集三帧扫描，多数投票识别前、左、右边界；已驶过的通路保持开放，对称更新相邻单元。其余内部边界初始为未知。
4. 从目标单元反向 BFS 泛洪，目标距离为 0，距离逐层加 1。探索时暂将未知边当作可通行；发现墙体后重新泛洪。优先选择距离下降的相邻格，等距离时比较访问次数与转向次数。如果起步时选中了未扫描的后方边界，先转向扫描确认，再执行移动。这是在线搜索，并非读取预设路线。
5. 路径执行采用单元中心航点：里程计闭环转向，使用实测角速度提供阻尼，连续确认角度与速度稳定后才向前行驶。两侧都有墙时根据左右距离差居中；只有一侧墙时保持期望侧向距离；交叉口两侧开阔时主要依靠航向和横向偏差。进入下一格后重新观察与规划。
6. 到达目标格后按配置的出口方向驶出并停车。数据过期、扇区无效时停车等待；不可达、过大跟踪偏差或动作超时进入 `FAULT` 并持续发送零速度。

该版本以可靠的在线探索为基线，单元中心会停车观测，拐弯采用原地转向，不是此前仅用于检查地图的连续圆弧跟踪。后续可在已探明地图上对比更平滑、更快的路径执行器。

两侧都有墙时，居中误差为 `e_center = (d_left - d_right) / 2`。角速度由航向误差、到通道中心线的横向误差、居中误差和实测角速度阻尼叠加，再做限幅。前方最小测距停车判断优先于行驶控制。

实现位置：`include/maze_mouse_navigation/core.hpp` 与 `src/core.cpp` 为雷达分区、迷宫边界、泛洪和居中计算；`src/navigator.cpp` 为 ROS 通信、坐标对齐和观察/转向/行驶状态机。所有这些文件按 `-std=c++20` 编译。

## 配置与话题

`config/navigation.yaml` 提供格网、目标、30° 扇区、墙体判定阈值、速度和控制增益。默认线速度 0.20 m/s，上限按 Burger 的 0.22 m/s 校验。参数启动时读取；改变起点或重新开始实验，应重启节点并把机器人复位到起点。

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| `/scan` | sensor_msgs/LaserScan | 输入雷达 |
| `/odom` | nav_msgs/Odometry | 输入里程计 |
| `/cmd_vel` | geometry_msgs/Twist | 输出速度 |
| `/maze_mouse/sectors` | geometry_msgs/Vector3Stamped | x=前方、y=左侧、z=右侧的投影中位距离 |
| `/maze_mouse/status` | std_msgs/String | JSON 状态、位置、距离、已知边数、移动次数、泛洪值等 |
| `/maze_mouse/path` | nav_msgs/Path | 基于当前观测、允许未知边的候选路线，RViz 中黄色 |
| `/maze_mouse/trajectory` | nav_msgs/Path | 实际里程计轨迹，RViz 中蓝色 |

泛洪值为按 `y * columns + x` 排列的数组；`-1` 表示当前不可达。候选路线可能经过尚未观察的单元，会随探索更新。路径和轨迹发布在输入里程计坐标系，节点根据首次静止位姿将它与格网起点对齐。

```zsh
ros2 topic echo /maze_mouse/status --full-length
ros2 topic echo /maze_mouse/sectors
colcon test --packages-select maze_mouse_navigation
colcon test-result --verbose
```

此实现面向轴对齐、已知单元尺寸的矩形墙迷宫，不包含 SLAM 或任意非结构化环境建图。三扇区足以完成当前静态迷宫的局部识别，不代表全向动态避障。

## 来源与使用范围

代码为本项目 AI 辅助实现；课程报告需按任务书记录 AI 使用范围与后续人工调试。实际验证结果记录在 `docs/validation.md`。该包是新增导航基线，地图包原有的参考路径仿真数据不作为本算法的运行结果。
