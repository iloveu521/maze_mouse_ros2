# maze_mouse_ws

《机器人感知与控制综合实践》的 ROS 2 Humble 工作空间。

- `maps/`：老师提供的原始世界文件，保持原样。
- `src/maze_mouse_gazebo/`：约 6×6 米的 6×6 单元迷宫、Gazebo/TurtleBot3/RViz2 启动文件及地图生成和校验工具。
- `src/maze_mouse_navigation/`：C++20 前/左/右 30° 雷达分区、居中控制、在线泛洪搜索与路径跟随。

[完整使用说明](src/maze_mouse_gazebo/README.md) · [地图俯视设计图](src/maze_mouse_gazebo/docs/maze_6x6.svg) · [参考通路](src/maze_mouse_gazebo/docs/maze_6x6_solution.svg)

地图不绑定特定导航算法。已安装 Gazebo 和 TurtleBot3，并实际检查 `/scan`、`/odom` 及 Burger 沿平滑参考轨迹完成跑图；结果见 [验证记录](src/maze_mouse_gazebo/docs/validation.md)。默认启动只加载环境、机器人和可视化。

当前终端为 Zsh，编译并启动：

```zsh
cd ~/workspace/maze_mouse_ws
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-select maze_mouse_gazebo
source install/setup.zsh
ros2 launch maze_mouse_gazebo maze.launch.py
```

Bash 终端将 `setup.zsh` 改为 `setup.bash`。无界面运行追加 `gui:=false rviz:=false`。

使用新增的 C++20 在线导航（自动启动仿真，需先关闭上面的仅地图实例）：

```zsh
colcon build --symlink-install --packages-up-to maze_mouse_navigation
source install/setup.zsh
ros2 launch maze_mouse_navigation navigation.launch.py
```

算法说明、参数和状态话题见 [导航功能包说明](src/maze_mouse_navigation/README.md)。导航使用实际雷达和里程计在线搜索，不加载地图包的预设中心参考路径。
