# Rosmaster M1 Nav2 MPPI 避障

本仓库仅保留 ROS 2 Humble 的 Nav2 MPPI 避障仿真与实机 bringup。

- 全局规划：`nav2_navfn_planner`；
- 局部避障控制：`nav2_mppi_controller::MPPIController`；
- 底盘：M1 麦克纳姆轮，MPPI 使用 `Omni` 并保留 `linear.x`、`linear.y` 与 `angular.z`；
- 障碍输入：`/scan` 进入 Nav2 的全局/局部 Costmap；
- 安全链：MPPI → Velocity Smoother → Collision Monitor → watchdog → `/cmd_vel`。

已移除旧的 MPC、滚动时域自定义规划、Torch 算法、点云聚类、卡尔曼跟踪、动态轨迹预测与相关测试/文档。

## 构建

```bash
cd /home/lin24311/car_ws2/rosmaster-m1-project
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-mppi-controller ros-humble-ros-gz-sim

colcon build --packages-select \
  yahboomcar_description m1_nav2_support m1_nav2_bringup \
  --symlink-install
source install/setup.bash
```

## Gazebo 与 RViz MPPI 仿真

```bash
ros2 launch m1_nav2_bringup nav2_m1_gazebo.launch.py \
  gui:=true rviz:=true dynamic_obstacles:=true
```

Gazebo 启动后，在 RViz 选择 **2D Goal Pose** 设置目标点。MPPI 会依据实时 `/scan` 更新的 Costmap 进行静态和反应式动态避障；若无安全可行轨迹，安全行为是减速或停车。

## 实机 MPPI 启动

底盘驱动必须提供 `/scan`、`/odom` 与 `odom → base_footprint` TF：

```bash
ros2 launch m1_nav2_bringup nav2_m1_real.launch.py rviz:=true
```
