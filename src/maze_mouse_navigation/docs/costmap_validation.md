# 激光 costmap 连续左墙控制验证

2026-11-01，ROS 2 Humble、Gazebo Classic 11、TurtleBot3 Burger、CMake 3.22.1、C++20。本页保留提速前 0.5 m/s、额外膨胀 0.10 m 的历史基线，配置现保存在 `config/costmap_conservative.yaml`。最新默认配置与 Gazebo 复测见 [调参记录](speed_tuning/README.md)。

## 自动检查

`colcon test --packages-select maze_mouse_gazebo maze_mouse_navigation` 与 `colcon test-result --verbose`：26 项测试通过，无失败、无跳过。包含原有地图及两种历史控制器的检查，新版 costmap 有 7 项检查：

- 车体半径与额外 0.10 m 膨胀分别生效。
- 无效扫描不生成自由区域，未知位置不允许行驶。
- 滚动地图保留墙体位置，补偿 Burger 激光的 -0.032 m 安装偏移。
- 无有效输入、目标完成时输出零速度。
- 完整 world 的干净扫描、偏离中心起步并带 0.01 m 高斯测距噪声两种回归。

整图回归只在测试端读取 world 碰撞形状并计算 360 束、5 Hz 激光；控制器接收扫描、扇区、里程计，不读取 world 或参考解答。执行器含线/角速度滞后与加速度限制，20 Hz 控制，100 Hz 检查碰撞距离。断言同时前进和转向的样本超过 100 个、车辆不从入口绕到迷宫外侧、最终到达出口，并始终检查中心与真实墙面的距离大于 0.23 m。

| 回归 | 模型完成时间 | 最大指令速度 | 中心到真实墙面最小距离 |
| --- | --- | --- | --- |
| 干净扫描，标准起点 | 127.2 s | 0.500 m/s | 0.3457 m |
| 0.01 m 噪声，起点 `(0.56,0.47,0.07)` | 125.6 s | 0.4999 m/s | 0.3445 m |

上述为简化运动模型，不等同于 Gazebo 动力学结果。完整输出见 [回归日志](costmap_geometry.log)。

## Gazebo 实测

使用实际 `/scan`、`/odom`，默认参数与未修改的 Burger 模型，无界面启动命令：

```zsh
ALSOFT_DRIVERS=null ros2 launch maze_mouse_navigation navigation.launch.py gui:=false rviz:=false params:=$PWD/src/maze_mouse_navigation/config/costmap_conservative.yaml
```

`ALSOFT_DRIVERS=null` 仅避免无声卡环境的音频初始化等待，不修改控制器、雷达或物理参数。

| 项目 | 最终代码实测结果 |
| --- | --- |
| 最终状态 | `COMPLETE`，无故障 |
| 出口位置 | 约 `(6.320,5.487)` |
| 完成时间 | **140.5 s** 仿真时间，含转弯、掉头及数据等待 |
| 里程计累计路程 | **28.20 m** |
| 最大线速度指令 | 0.500 m/s |
| 里程计最大前向速度绝对值 | 约 0.5102 m/s |
| 最小有效雷达测距 | 约 0.2977 m |
| 本次最长轨迹选择计算时间 | 约 2.74 ms，不含建图回调 |
| 完成后指令 | 线速度和角速度均为零 |

证据：[终态](costmap_result.yaml)、[运行日志](costmap_run.log)、[里程计轨迹 CSV](costmap_trajectory.csv)、[完成后速度](costmap_completion_command.yaml)、[运行中的 costmap 快照](costmap_snapshot.yaml)。[实际轨迹图](costmap_run.svg)由本次话题数据离线绘制，墙体几何仅用于绘图。

雷达最小距离是测距值，不是车体中心到墙面的距离，也不是扣除车体半径后的余量。里程计速度存在瞬态超调，因此与指令上限分别记录。

首轮版本实测 143.2 s；补全候选速度的制动预测后，最终版本复测如上。该结果与历史路口停车版本的 212.8 s 对比约减少 34% 的仿真时间，但单次运行含传感器噪声和调度差异，不能当作统计性能结论。

## 适用范围

当前结果验证该迷宫、Burger 仿真和 0.5 m/s 上限下的完整运行。左墙策略不保证全局最短路径，局部地图使用里程计，尚未接入 SLAM。车辆可以走通道内可行位置、提前形成弧线，但死胡同和大角度改向时仍允许停车旋转。旧路口停车控制器的数据保留在 [历史验证](left_wall_validation.md)，两者不混作同一控制器的结果。
