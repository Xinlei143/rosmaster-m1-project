# 工作日志：Ogre1 双 180° GPU LiDAR 诊断与修复

**日期：2026-09-04**  
**项目：rosmaster-m1-project**

## 目标

处理 Fortress / OGRE2 单个 360° GPU LiDAR 在完整 M1 仿真负载下会进入整帧
`-Inf` 锁死的问题，同时保持 Nav2 下游继续只消费统一的 `/scan` 接口。

## 1. 问题发现

历史完整仿真中，单个 360° GPU LiDAR 运行一段时间后会从正常数据突然转为
667 个 beam 均为 `-Inf` 的 scan。该状态持续存在且不自行恢复，会使 AMCL、
local costmap、Collision Monitor 与 MPPI 失去可靠激光输入。

这不是一次性异常值或 RViz 显示问题：故障定义为预期 beam 数的整帧均为
`-Inf`，并统计首次发生时间、正常到异常的转移、连续坏帧长度和恢复次数。

## 2. 检测与定位方法

新增/扩展了原始数据采集与分类工具，沿真实数据路径同时记录：

```text
Gazebo Transport 原始 GPU LiDAR
            ↓
ros_gz_bridge 后的 ROS raw LaserScan
            ↓
统一 /scan 与 Nav2 消费链路
```

每帧记录 beam 数、有限值与 `+Inf`/`-Inf`/NaN 比例、仿真时间戳及连续坏帧
状态。Gazebo raw 与 ROS raw 按共同时间戳进行分类匹配，避免将 bridge 或 RViz
误判为故障源。

同时进行了 raw-only 场景隔离、静态/动态模型和回移实验；这些实验用于缩小
故障边界，不直接替代完整高风险回归条件。

## 3. 关键诊断结论

故障在 Gazebo 原始 GPU LiDAR 输出层已经出现，ROS bridge 没有引入该异常：
Gazebo raw 与 ROS raw 在共同时间戳上的分类一致。问题边界因此落在完整 M1
负载下 Fortress OGRE2 单 360° GPU LiDAR 的渲染/读回路径，而不是 Nav2 参数、
RViz 或 bridge。

历史 OGRE2 单 360°路径被保留为 regression/debug 模式，而没有被删除或用
软件雷达掩盖。

## 4. 实施方案

采用只替换 LiDAR backend、保持 ROS 下游接口不变的方案：

```text
历史路径：OGRE2 + 单 360° GPU LiDAR  → /scan

生产路径：Ogre1 + front 180° GPU LiDAR  ┐
                    rear 180° GPU LiDAR   ├→ dual_laser_merger → 唯一 /scan
                                            ┘
```

- front 使用 `laser_scan_link`；rear 使用同位置、yaw = π 的
  `laser_scan_rear_link`。
- 两路均为 334 beams、±90°、12 Hz、0.05--12 m，分别发布 `/scan_front` 与
  `/scan_rear`。
- merger 输出 667 bins、约 12 Hz、frame 为 `laser_scan_link` 的唯一 `/scan`。
- bridge 与 merger 根据单/双雷达模式互斥启动，避免 `/scan` 出现双 publisher。
- 默认配置固定为 `render_engine:=ogre dual_gpu_lidar:=true`；历史回归使用
  `render_engine:=ogre2 dual_gpu_lidar:=false`。
- AMCL、Nav2 costmap、Collision Monitor 与 RViz 的 `/scan` 配置不作改动。

运行依赖为 `ros-humble-dual-laser-merger`；需在实际机器上永久安装：

```bash
sudo apt install ros-humble-dual-laser-merger
```

## 5. 验证过程

先完成静态与单元契约验证：world render engine 一致性、Xacro 单/双模式展开、
beam/topic/frame/FOV、bridge/merger 互斥、唯一 `/scan` publisher、包依赖与
Nav2 `/scan` 配置未改变。

随后完成短时 smoke，再按完整重启与严格交错顺序运行：

```text
A1 → B1 → A2 → B2 → A3 → B3
```

- A：历史 OGRE2 单 360°高风险条件。
- B：只替换为 Ogre1 双 180°与 merger，其余条件保持相同。
- 每个 run 为 300 s，保存 Gazebo raw、ROS raw、merged scan、进程日志、命令、
  环境、逐 run summary 与 campaign summary。

测试结果：support package 98 项测试通过，bringup package 19 项测试通过。

## 6. A/B 结果

### A：OGRE2 单 360°历史路径

三次均复现 whole-frame `-Inf` latch：

| 重复 | 首次锁死时间 | 锁死占比 | 连续锁死时长 | 恢复 |
|---|---:|---:|---:|---:|
| A1 | 52.622 s | 82.443% | 247.008 s | 0 |
| A2 | 52.788 s | 82.373% | 246.593 s | 0 |
| A3 | 52.954 s | 82.312% | 246.344 s | 0 |

### B：Ogre1 双 180° + merger

三次、每次 300 s 均通过：

- 前后两路共六条 Gazebo raw trace：whole-frame `-Inf` 为 0、NaN 为 0、连续
  all-negative streak 为 0。
- 每条 raw half-scan 固定为 334 beams，p50/p95 帧间隔均为 0.083 s，约 12.048 Hz。
- Gazebo raw 与 ROS raw 共同时间戳分类 mismatch 为 0。
- merged `/scan` 固定为 667 bins、`laser_scan_link`、约 12.048 Hz，并且只有一个
  publisher。

完整产物：

- `results/gpu_lidar_dual_ab_20260904/campaign_summary.json`
- `results/gpu_lidar_dual_navigation_20260904/campaign_summary.json`

## 7. 结论与边界

**LiDAR backend：通过。** 在相同高风险回归条件下，OGRE2 单 360°路径 3/3
可复现锁死；Ogre1 双 180° + merger 3×300 s 未复现该锁死，同时保持统一 `/scan`
接口。因此不再继续修改 Gazebo 源码或 OGRE2 单 360°生产路径。

**完整导航系统：部分通过。** 三次 300 s 导航中，双路 raw LiDAR 持续稳定，
机器人完成实际移动，Nav2/MPPI 命令链持续工作；但动态障碍的 costmap 检测率为
0.611、0.882、0.906，未达到 0.95 的验收阈值，最长漏检也超过 0.5 s。前向近障碍
已记录 Collision Monitor stop polygon 触发；后向明确倒车命令到
`/m1/cmd_vel_raw` 归零的正式闭环证据尚未完成。

后续只处理下列两项，不再调试 LiDAR backend：

1. 后向 Collision Monitor 的确定性 stop 闭环。
2. 动态障碍在 `raw /scan → local costmap obstacle cells → Collision Monitor / MPPI`
   三层中的持续可观测性。

## 8. 代码提交

- `8e33ea2 test(sim): extend GPU LiDAR failure diagnostics`
- `aa53cb7 fix(sim): use dual Ogre GPU LiDAR scans`

