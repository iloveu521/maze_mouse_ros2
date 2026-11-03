# 迷宫鼠 Gazebo 环境

面向 **ROS 2 Humble / TurtleBot3 Burger / Gazebo Classic 11** 实验，提供一键启动文件。本包负责环境与机器人加载，供后续路径搜索、沿通道中心导航、轨迹平滑及速度规划实验使用。

`maze.launch.py` 和 `world.launch.py` 默认加载 [worlds/maze.world](worlds/maze.world)。此图约 10×10 m，坐标中心在世界原点；西侧入口位于 y≈-3.44 m，起点为 (-4.50, -3.44)，东侧出口位于 y≈3.49 m，导航目标为 (5.45, 3.49)。机器人出生位置和默认 costmap 导航目标已与此 world 对齐。`worlds/maze_6x6.world` 是另一张生成测试地图，仍保留供脚本和历史验证使用。

默认的 C++20 costmap 路径跟踪导航见相邻的 [maze_mouse_navigation](../maze_mouse_navigation/README.md)；本包的 `maze.launch.py` 负责环境、机器人、TF 和可视化启动。`maze_6x6.world` 的设计图可以由 `scripts/generate_maze.py` 重新生成到 `docs/`（脚本会自动创建目录）。

## `maze_6x6.world` 生成地图参数

| 项目 | 数值 / 说明 |
| --- | --- |
| 单元数 | 6 列 × 6 行，36 个单元全部连通 |
| 单元间距 | 1.00 m |
| 墙中心线包围尺寸 | 6.00 × 6.00 m |
| 含墙外沿尺寸 | 6.12 × 6.12 m |
| 墙厚 / 墙高 | 0.12 m / 1.20 m |
| 直走廊净宽 | 0.88 m |
| 地板 | 8.00 × 8.00 m，表面 z=0，含入口与出口外侧缓冲区 |
| 坐标 | 西南墙角为 (0, 0)，东为 +x，北为 +y，单位 m |
| 入口 | 西墙 x=0、y∈(0, 1) |
| 出生位置 | (0.50, 0.50, 0.01)，yaw=0，朝 +x；位于入口内第一格 |
| 出口 | 东墙 x=6、y∈(5, 6) |
| 出口外参考目标 | (6.45, 5.50) |
| 支路 | 5 条死胡同，适合路径搜索、岔路选择与掉头练习 |
| 中心参考轨迹 | 圆弧半径 0.50 m，采样间距 ≤0.05 m，全长约 16.37 m |

布局包括直线、直角 L 弯、U 形回折及 Z/S 形连续反向转弯。**S 形为矩形墙组成的折线 S 弯**，没有复用老师 `s_shape_maze.world` 的大半径圆弧几何；它不是连续曲率的圆弧 S 弯。

保留静态 box 墙、cylinder 墙端头、实体地板和深蓝灰配色；为 6 米尺度重新设计布局与墙厚，墙、端头均有与外观一致的 collision。入口和出口色块只有 visual，不会挡车或影响雷达。世界不依赖在线模型下载；以 scene ambient 设置环境光，使用合法的非空 world 名称。

## 居中行驶与速度优化

矩形墙角不要求小车到航点停车、原地转向。`config/centerline_reference.json` 同时提供原始通道中心航点 `cell_center_waypoints` 和用相切圆弧连接的 `smoothed_points`，后者允许连续通过 U 弯、反向弯。半径 0.50 m 的参考轨迹中心距墙至少 0.44 m；用半径 0.17 m 的机器人圆形包络计算，静态余量至少 0.27 m。

这张图适合比较折线跟踪与平滑轨迹跟踪、跟踪误差、过弯减速策略和完成时间。连续转弯仍有曲率约束，关系为 `ω = v / R`；例如以 0.20 m/s 跟踪半径 0.50 m 的圆弧，理想角速度约 0.40 rad/s。当前安装版本的官方 Burger 遥控脚本线速度上限为 0.22 m/s；仿真插件接收命令不等于已验证实车可用速度，不应仅靠提高命令值定义优化效果。

当前布局为树状迷宫，入口至出口只有一条拓扑通路，可用于比较搜索耗时、展开节点数以及导航性能；比较多条路线之间的选择时，可在后续版本打通少数内墙构成环路。参考轨迹是设计数据，算法实验中是否允许读取，应按实验设置区分已知地图规划和未知环境探索。

## 安装与编译

Ubuntu 22.04、ROS 2 Humble、CMake 3.22（本机为 3.22.1）：

```bash
sudo apt update
sudo apt install gazebo ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3 ros-humble-turtlebot3-simulations

cd ~/workspace/maze_mouse_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select maze_mouse_gazebo
source install/setup.bash
```

以上为 Bash 示例。如果终端使用 Zsh，将两个 `setup.bash` 分别换成 `setup.zsh`，其余命令相同。`build/`、`install/`、`log/` 是构建产物，提交代码时保留 `src/` 即可。

## 启动

一次加载迷宫、Burger、robot_state_publisher、Gazebo 和 RViz2：

```bash
ros2 launch maze_mouse_gazebo maze.launch.py
```

无界面启动（用于服务器或自动检查）：

```bash
ros2 launch maze_mouse_gazebo maze.launch.py gui:=false rviz:=false
```

只查看世界：

```bash
ros2 launch maze_mouse_gazebo world.launch.py
```

可选参数：`gui`、`rviz`、`paused`、`x`、`y`、`z`、`yaw`。例如 `rviz:=false` 只关闭 RViz。`world.launch.py` 另支持 `world` 和 `verbose`。不要同时运行多个使用默认 Gazebo master 端口的仿真实例。

机器人由已安装的官方 TurtleBot3 模型加载，不依赖终端预先设置型号；另开终端使用官方遥控时：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

RViz 固定坐标为 `odom`，显示机器人、TF 和 `/scan`。Gazebo world 的墙体不会自动显示在 RViz；当前实验没有启动 SLAM、地图服务或全局导航。世界坐标用于本 README 中的出生点/目标点；控制节点若使用 `/odom` 做到达判断，应先确认当前版本插件的里程计原点与启动位姿的关系。

## 检查

仿真运行时，另开已 source 环境的终端：

```bash
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic echo /odom --once
ros2 topic list
```

无需启动仿真即可校验配置拓扑、实际 SDF 墙体碰撞几何及平滑中心线余量：

```bash
python3 src/maze_mouse_gazebo/scripts/validate_maze.py
colcon test --packages-select maze_mouse_gazebo
colcon test-result --verbose
```

几何检查逐一核对全部相邻单元、全部外墙与两处开口，沿连线每 0.02 m 采样；以半径 0.17 m 的圆形包络验证通路余量，并检查导出的平滑参考轨迹。静态检查不能代替真实控制器的转弯和速度调试；默认世界的实际运行验证见根目录 [docs/validation.md](../../docs/validation.md)。

## 修改地图

修改 `config/maze_layout.json` 的 `main_path` 和 `branches`，每个路径列表中的相邻坐标必须共边，所有单元应连通。默认布局为树，也允许后续添加通路形成环路。随后在工作空间根目录执行：

```bash
python3 src/maze_mouse_gazebo/scripts/generate_maze.py
python3 src/maze_mouse_gazebo/scripts/validate_maze.py
colcon build --symlink-install --packages-select maze_mouse_gazebo
```

生成器会同时刷新 `worlds/maze_6x6.world`、两张 SVG 和 `config/centerline_reference.json`，避免参考数据和 world 不一致。若通过 `ros2 run` 调用已安装的生成器/检查器，需显式传 `--package-dir "$PWD/src/maze_mouse_gazebo"`，指向源码包。

```text
src/maze_mouse_gazebo/
├── CMakeLists.txt / package.xml
├── config/maze_layout.json       # 参数、通路、出生点和目标点
├── config/centerline_reference.json # 中心航点与圆弧连接的参考轨迹
├── worlds/maze.world             # 默认 10 m × 10 m 世界（导航使用）
├── worlds/maze_6x6.world         # 生成器输出的 6 × 6 测试世界
├── launch/world.launch.py       # 仅世界
├── launch/maze.launch.py        # 世界 + Burger + TF + RViz2
├── rviz/maze.rviz
├── scripts/generate_maze.py     # Python 标准库，无额外 pip 依赖
└── scripts/validate_maze.py     # 拓扑与碰撞校验
```

## 来源与使用记录

- 课程要求：工作空间根目录《任务书1：机器人感知与控制综合实践.docx》。
- 教师提供的原始 world 已从仓库移除；当前默认世界为 `worlds/maze.world`。
- 机器人及插件：安装的 [ROBOTIS TurtleBot3 simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble/turtlebot3_gazebo)，由其自身许可证约束；本包没有复制其模型源码。
- AI 辅助范围：本包迷宫布局、world 生成器、启动配置、设计图、几何检查和说明文档。用于课程材料时，请按任务书记录后续人工修改、调试和实测过程；组员分工应由实际参与者填写。
- 默认世界的实际运行验证见 [docs/validation.md](../../docs/validation.md)。
