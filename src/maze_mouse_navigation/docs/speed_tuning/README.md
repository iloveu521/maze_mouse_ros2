# Gazebo 整图调参记录

2026-11-01，ROS 2 Humble / Gazebo Classic 11.10.2 / TurtleBot3 Burger / CMake 3.22.1 / C++20。

默认采用 [costmap.yaml](../../config/costmap.yaml)：线速度指令上限 **1.2 m/s**，角速度上限 **2.0 rad/s**，额外膨胀 **0.04 m**，总硬膨胀半径 **0.18 m**。相同参数三次完成分别为 **111.9、109.5、119.6 s**，均值 **113.7 s**；相对历史 140.5 s 基线减少约 **19.1%**。样本量较小，均值仅用于本次调参比较。

1.5 m/s 的单次最快为 107.4 s，但相同参数复跑为 126.8 s，因此保留为对照配置，未作为默认。速度上限不等于实际速度：默认组里程计峰值范围为 0.871–0.933 m/s。没有修改 Burger 模型来强制实现更高速度。

## 实验条件与计时

所有下表参数组均直接启动新的 Gazebo `gzserver`，运行实际激光、里程计与差速驱动物理仿真；无 GUI / RViz。不是用简化运动模型替代 Gazebo 调参。world、起点、出口和 Burger SDF 保持一致，5 Hz / 360 束激光，雷达偏移 -0.032 m，轮加速度参数 1.0、轮扭矩参数 20。导航定时器为 50 ms。

- `elapsed_sim_s`：从首次传感器就绪到进入 `COMPLETE` / `FAULT` 的仿真时间；不计启动 Gazebo 的墙钟等待。
- `stopped_sim_s`：运行期间里程计 `|v| < 0.03 m/s` 的累计时间，包含原地转向；不是独立的规划计算耗时。
- `max_measured_speed`：里程计实际速度的峰值，与最大指令分开记录。
- 完成条件：首次进入出口 `(6.45, 5.5)` 的 0.14 m 容差，锁定完成并持续发布零线速度、零角速度；终态位置可能因制动惯性有后续位移。

## 全部参数组

表内 v 单位 m/s，w 单位 rad/s，余量单位 m，时间单位 s。每行配置链接包含完整参数；`linear_samples` 未写明的早期组按当时默认值 6 执行。

| 组 / 配置 | v / w 上限 | 额外余量 | 线速度采样区间 | 结果 | 完成或失败时间 | 实测速度峰值 | 低速累计 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [1](trial1.yaml) | 0.8 / 1.6 | 0.04 | 6 | COMPLETE | 140.9 | 0.794 | 14.38 |
| [2](trial2.yaml) | 1.2 / 2.0 | 0.04 | 6 | COMPLETE | 111.9 | 0.874 | 8.63 |
| [3](trial3.yaml) | 1.5 / 2.4 | 0.03 | 6 | FAULT | 66.2 | 1.064 | 20.56 |
| [4](trial4.yaml) | 1.5 / 2.2 | 0.03 | 6 | FAULT | 54.2 | 0.927 | 20.26 |
| [5](trial5.yaml) | 1.2 / 2.2 | 0.04 | 12 | COMPLETE | 116.5 | 0.869 | 14.30 |
| [6](trial6.yaml) | 1.0 / 1.8 | 0.04 | 12 | COMPLETE | 119.2 | 0.848 | 10.13 |
| [7](trial7.yaml) | 1.2 / 2.0 | 0.04 | 6 | COMPLETE | 109.5 | 0.933 | 11.48 |
| [8](trial8.yaml) | 1.5 / 2.0 | 0.04 | 6 | COMPLETE | 107.4 | 0.921 | 11.91 |
| [9](trial9.yaml) | 1.5 / 2.0 | 0.04 | 6 | COMPLETE | 126.8 | 0.887 | 16.21 |
| [10](trial10.yaml) | 1.2 / 2.0 | 0.04 | 6 | COMPLETE | 119.6 | 0.871 | 8.91 |

原 0.5 m/s、额外 0.10 m 余量的 140.5 s 基线及原始证据保存在 [历史验证](../costmap_validation.md)；启动对照使用 [costmap_conservative.yaml](../../config/costmap_conservative.yaml)。

第 3 组暴露了“目标仍是自由格，但已被新扫描的墙体隔断”的持续停滞，已修复为当周期重选可达目标，并增加对应回归测试。第 4 组包含该修复，但 0.03 m 余量与更激进的预测仍未完成；两组均按失败记录，没有参与完成时间比较。第 5、6 组增加采样与近墙惩罚也没有更快。第 7 组为第 2 组参数在修复后的复跑；第 10 组为最终默认配置直接启动验收。第 8、9 组只在第 7 组基础上提高线速度上限到 1.5 m/s。

## 参数调整及空间变化

车体半径保持 0.14 m，额外余量从 0.10 m 缩至 0.04 m；软代价范围 0.55 → 0.38 m；地图分辨率 0.05 → 0.04 m。0.88 m 直道扣除两侧硬膨胀后的中心可选宽度从 0.40 → 0.52 m，另有保守的离散化与插值补偿。

缩短预测时域 1.50 → 0.75 s，仍保留 0.20 s 反应时间和完整制动轨迹检查；角速度上限 0.9 → 2.0 rad/s，角加速度预测 1.5 → 3.0 rad/s²。局部轮廓前瞻 0.85 → 1.05 m，调整恢复间隙门限与速度偏好，让车辆更早形成转弯弧线。保持左墙策略，不加载解答，不要求走通道中心，也不要求到迷宫格中心停车。

[轨迹、costmap 与速度曲线](run.svg)来自最终第 10 组的话题数据。橙色为实际里程计轨迹，灰色为历史基线；局部 costmap 截图与蓝色轨迹为同次仿真时刻；速度曲线分别显示指令和实际里程计速度。

默认三次运行的最小有效激光测距分别为 0.201、0.201、0.199 m。雷达测距不是车体间隙。根据记录的里程计位置与 world 碰撞几何离线计算，最小中心墙距分别约 0.231、0.244、0.240 m，扣除 0.14 m 包络后约 0.091、0.104、0.100 m。计算结果见 [间距估计](clearance_estimates.json)；这是里程计推算，不是独立接触检测。world 仅用于离线核对和绘图，不提供给控制器。

## 运行与热更新

在工作空间根目录执行：

```zsh
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-up-to maze_mouse_navigation
source install/setup.zsh
ALSOFT_DRIVERS=null ros2 launch maze_mouse_navigation navigation.launch.py gui:=false rviz:=false
```

`ALSOFT_DRIVERS=null` 仅跳过无声卡环境的音频初始化。需要界面时使用 `gui:=true rviz:=true`。运行中另开已 source 的终端：

```zsh
ros2 param set /maze_navigator max_linear_speed 1.0
ros2 param set /maze_navigator max_angular_speed 1.8
ros2 param set /maze_navigator inflation_margin 0.04
ros2 param load /maze_navigator src/maze_mouse_navigation/config/costmap.yaml
ros2 topic echo /maze_mouse/status std_msgs/msg/String --no-daemon --full-length
```

热更新保留已有墙体证据；不重置已完成状态。地图分辨率、尺寸或雷达外参变化需等待重新建图。状态新增配置速度、更新次数、低速累计时间。非法负膨胀会被拒绝；整份默认 YAML 可加载，证据见 [最终参数服务检查](hot_parameters_final.log)、[热更新后完成状态](completion_after_hot_update.yaml)及[完成后零速度指令](completion_command.yaml)；早期检查另存于 [参数检查](hot_parameters.log)。

## 验证与数据

构建成功；`colcon test --packages-select maze_mouse_gazebo maze_mouse_navigation`、`colcon test-result --verbose`：**29 项，0 错误，0 失败，0 跳过**，见 [检查日志](tests.log)。包括热更新保留地图及完成状态、旧目标被障碍隔断后的重选，以及保留的整图简化模型回归。导航、建图、路径和控制算法全部是 C++20；离线数据记录与绘图不参与导航。

各组的 `trialN_run.log` 和 `trialN_result.json`（第 1 组为 YAML）保存状态变化与最终指标。第 2、7、8、9、10 组保留稀疏轨迹 CSV；第 8、9、10 组另保留全频里程计，第 10 组保留速度指令、costmap 快照。对应文件：[最终状态](trial10_result.json)、[运行日志](trial10_run.log)、[里程计](trial10_odom.csv)、[轨迹](trial10_trajectory.csv)、[速度指令](trial10_commands.csv)、[costmap](costmap_snapshot.yaml)。

本次 world SHA-256：`f8bc340c84e573c53cd40d0e3ac128934353d0f71c076f8bc9fc80b441af1a8e`；已安装 Burger SDF SHA-256：`179e9595d4e229cf207c3fc0515aa66f4bc4434a37b398fc3c39753dc2feffb1`。结果限定于当前迷宫和仿真模型；左墙策略仍不保证最短路，尚未接入 SLAM。
