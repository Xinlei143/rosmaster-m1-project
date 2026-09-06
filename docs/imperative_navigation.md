# Imperative Navigation 并行方法

本分支恢复了历史提交 `b8ca8ca` 中的 Imperative 局部避障算法，并将 ROS 2 接口适配到当前仓库。
当前 `main` 将 Nav2 MPPI、默认 Imperative 和 localized Imperative 作为并列仿真入口；Imperative
仍必须通过独立启动文件运行，不能与 Nav2 Gazebo 控制器同时启动。完整系统接口说明见
[system_architecture.md](system_architecture.md)。

## 算法链路

```text
LaserScan + /odom
      │
      ▼
有效回波 → 相邻激光点聚类 → 点簇质心/半径
      │
      ▼
常速度 Kalman 跟踪：[x, y, vx, vy]
      │
      ├─ 当前激光点和位姿辅助点地图
      └─ 障碍物未来位置：p(t) = p + v t
      │
      ▼
20 步滚动时域候选动作 → 目标/控制量/碰撞代价 → 下一步速度
```

点簇使用连续激光束且相邻点距离不超过 `0.30 m` 的规则连接；少于 3 个点的簇和跨度超过
`1.20 m` 的大型簇不进入动态跟踪，但有效原始激光点仍保留在碰撞检测中。Kalman 跟踪使用
`0.80 m` 关联门限，最多保留 5 个丢失扫描周期。默认控制周期为 `0.1 s`，预测时域为 20 步，
即 2 秒。

## 当前仓库接口

| 场景 | 输入 | 输出 | 复用基础设施 |
| --- | --- | --- | --- |
| Gazebo | `/scan` 或 `/sim_scan`、`/odom`、`/m1/dynamic_obstacles` | `/cmd_vel` | `m1_nav2_support` 的 `m1.sdf`、动态障碍物、软件雷达 |
| 实机 | `/scan`、`/odom`、`odom → laser` TF | `/imperative/cmd_vel_raw` | `m1_nav2_support/m1_cmd_watchdog` |

Gazebo 软件雷达模式下，Imperative 控制器直接订阅 `/sim_scan`；为保持 RViz 和仓库接口一致，
启动文件只在该模式启动当前 `m1_nav2_bringup/scan_relay`，将扫描复制到 `/scan`。GPU 雷达模式
直接使用 `/scan`。GPU 雷达产生饱和最小量帧时，控制器会安全停机并可使用当前世界静态障碍物
fallback，不会把饱和帧当作正常点云。

## 启动

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  yahboomcar_description m1_nav2_support m1_nav2_bringup imperative_navigation \
  --symlink-install
source install/setup.bash
```

Gazebo：

```bash
ros2 launch imperative_navigation imperative_m1_gazebo.launch.py \
  gui:=true rviz:=true software_lidar:=false dynamic_obstacles:=true
```

默认 Imperative 的目标只由 `goal_x/goal_y` 启动参数给定；它没有 RViz `2D Goal Pose` 订阅或导航 Action。

localized Gazebo（AMCL、map 目标和 watchdog；默认干运行）：

```bash
ros2 launch imperative_navigation imperative_m1_localized_gazebo.launch.py \
  gui:=true enabled:=false
```

实机干运行：

```bash
ros2 launch imperative_navigation imperative_cmd_watchdog.launch.py
ros2 launch imperative_navigation imperative_m1_real.launch.py \
  goal_x:=0.5 goal_y:=0.0 enabled:=false
```

实机控制器默认只感知、跟踪、规划和发布零原始指令。缺少或过期的 `/scan`、`/odom`、TF，或者
激光近距离急停条件触发时，也会保持零指令。物理运动不属于本分支的自动验收步骤。

## 可视化与安全边界

- `/imperative/planned_path`：当前滚动时域计划；
- `/imperative/tracks`：已确认障碍物轨迹；
- Gazebo 直接使用 `/cmd_vel`，实机必须由独立 watchdog 将 `/imperative/cmd_vel_raw` 转发到 `/cmd_vel`；
- Imperative 和 Nav2 的启动入口必须二选一；
- 默认不修改当前 Nav2 的参数、生命周期顺序或速度控制链。
