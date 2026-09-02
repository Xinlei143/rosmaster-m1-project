# Rosmaster M1 Nav2 MPPI 与 Imperative 避障

当前默认入口仍是 ROS 2 Humble 的 Nav2 MPPI 避障仿真与实机 bringup；`imperative` 分支另外恢复了
旧版 Imperative 局部规划器，作为并行、可独立启动的实验方法。

- 全局规划：`nav2_navfn_planner`；
- 局部避障控制：`nav2_mppi_controller::MPPIController`；
- 底盘：M1 麦克纳姆轮，MPPI 使用 `Omni` 并保留 `linear.x`、`linear.y` 与 `angular.z`；
- 障碍输入：`/scan` 进入 Nav2 的全局/局部 Costmap；
- 安全链：MPPI → Velocity Smoother → Collision Monitor → watchdog → `/cmd_vel`。

Imperative 方法不会被 Nav2 默认启动文件自动启动。它使用独立的 `imperative_navigation` 包和启动
文件，同一时刻不要与 Nav2 Gazebo 控制器一起运行。

## 构建

```bash
cd /home/lin24311/car_ws2/rosmaster-m1-project
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-mppi-controller ros-humble-ros-gz-sim

colcon build --packages-select \
  yahboomcar_description m1_nav2_support m1_nav2_bringup imperative_navigation \
  --symlink-install
source install/setup.bash
```

## Gazebo 与 RViz MPPI 仿真

```bash
ros2 launch m1_nav2_bringup nav2_m1_gazebo.launch.py \
  gui:=true rviz:=true dynamic_obstacles:=true software_lidar:=false
```

Gazebo 启动后，在 RViz 选择 **2D Goal Pose** 设置目标点。MPPI 会依据实时 `/scan` 更新的 Costmap 进行静态和反应式动态避障；若无安全可行轨迹，安全行为是减速或停车。

## 实机 MPPI 启动

底盘驱动必须提供 `/scan`、`/odom` 与 `odom → base_footprint` TF：

```bash
ros2 launch m1_nav2_bringup nav2_m1_real.launch.py rviz:=true
```

## Imperative 并行方法

Imperative 使用 LaserScan 相邻回波聚类、`[x, y, vx, vy]` 常速度 Kalman 跟踪和 20 步滚动时域候选
轨迹。动态障碍物未来位置采用常速度外推，不是学习模型。完整算法说明见
[`docs/imperative_navigation.md`](docs/imperative_navigation.md)。

Gazebo 仿真只启动当前仓库的 Gazebo、软件雷达和动态障碍物适配器，再启动 Imperative 控制器：

```bash
ros2 launch imperative_navigation imperative_m1_gazebo.launch.py \
  gui:=true rviz:=true dynamic_obstacles:=true software_lidar:=false
```

实机入口默认是干运行，只有显式设置 `enabled:=true` 才会输出物理原始指令；watchdog 应在独立终端
运行：

```bash
ros2 launch imperative_navigation imperative_cmd_watchdog.launch.py
ros2 launch imperative_navigation imperative_m1_real.launch.py enabled:=false
```
