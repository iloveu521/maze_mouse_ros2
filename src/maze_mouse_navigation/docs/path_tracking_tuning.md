# 路径跟踪与控制器调参

ROS 2 Humble、Gazebo Classic、TurtleBot3 Burger。控制算法使用 C++20，CMake 3.22。默认最大线速度为 0.8 m/s，额外墙体膨胀为 0.04 m。

## 改动

- 前瞻距离按 `0.60 + 0.35 × 当前速度` 增长，最多 0.90 m。若车与前瞻点之间的射线穿过硬膨胀墙面，就逐步缩短前瞻，避免隔墙追逐弯后路径。
- 以纯跟踪曲率和路径切线预览曲率混合成转向前馈；航向和横向误差由带积分限幅、微分滤波的 PID 修正。
- 预测路径曲率限制弯道速度；角速度目标经过 0.18 s 一阶滤波，并受 2.5 rad/s² 指令变化率限制。
- 若当前位置进入硬膨胀边缘，只在后方已观测、倒退全段间隙不减且终点间隙增加时，短暂以最多 0.12 m/s 倒退。更深重叠或未知空间仍停车。
- 状态 JSON 记录 `tracking_error`、`control_lookahead` 和 `target_angular`，便于在 RViz 轨迹之外诊断控制行为。

## Gazebo 结果

同一张 6×6 地图、同一 0.8 m/s 上限下，两个完整且独立启动的仿真都到达出口：

| 指标 | 旧控制器基线 | 新控制器试跑 A | 新控制器复跑 B |
| --- | ---: | ---: | ---: |
| 状态 | COMPLETE | COMPLETE | COMPLETE |
| 仿真用时 | 123.5 s | 104.4 s | 106.1 s |
| 低于 0.03 m/s 累计时间 | 3.96 s | 9.24 s | 11.01 s |
| 里程计路程 | 30.10 m | 26.87 m | 28.13 m |
| 真实墙体最小中心距离 | 未保存 | 0.249 m | 0.227 m |
| 恢复 / 阻塞状态进入次数 | 未保存 | 8 / 1 | 9 / 1 |
| 指令角速度反向次数 | 35 | 34 | 36 |
| 指令角速度总变差 / 秒 | 1.622 | 1.164 | 1.176 |

两次新试跑平均 105.25 s，比基线快 14.8%；角速度指令总变差约低 28%，最小实测墙距均高于 0.22 m。里程计角速度的反向次数仍约 28 次，与旧基线相同，说明物理转弯的过冲还有继续优化空间。恢复次数也没有消失。这里的实测足以确认完成时间和指令平滑度改善，但不代表每个随机扰动或地图都同样稳定。

轨迹叠图：[Gazebo 实际轨迹](path_tracking_runs.svg)。终态和完整话题记录通过 `tools/record_run.py` 保存；可用 `tools/score_run.py` 统计角速度换向、恢复状态、实际墙距，用 `tools/plot_runs.py` 生成轨迹图。

## 参数

默认值见 [`config/costmap.yaml`](../config/costmap.yaml)。主要控制参数如下：

| 参数 | 值 | 作用 |
| --- | ---: | --- |
| `pursuit_lookahead` / `pursuit_lookahead_gain` / `pursuit_lookahead_max` | 0.60 m / 0.35 s / 0.90 m | 随车速增加前瞻 |
| `path_preview_distance` | 1.20 m | 路径曲率预览长度 |
| `heading_kp` / `heading_ki` / `heading_kd` | 0.75 / 0.02 / 0.16 | 航向 PID 增益 |
| `lateral_gain` / `curvature_feedforward` | 0.40 / 1.0 | 横向误差和路径曲率权重 |
| `max_lateral_acceleration` | 0.65 m/s² | 弯道预测速度上限 |
| `angular_command_acceleration` / `angular_filter_time_constant` | 2.5 rad/s² / 0.18 s | 转向变化率和滤波 |

可通过 ROS 参数在线调整，并同步记录 `/maze_mouse/status`、`/cmd_vel` 与 `/odom`，按完成状态、耗时、低速累计、角速度总变差和最小墙距一起比较。
