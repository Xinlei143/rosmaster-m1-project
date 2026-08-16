# house / cafe 场景与多形状动态障碍物

## 场景

仿真现在支持三个场景 profile：

- `imperative_m1`：原有小房间，保持原有静态圆柱和连续运动模式。
- `house`：住宅网格场景。
- `cafe`：咖啡馆网格场景。

`house` 和 `cafe` 的模型资源已经复制到当前 ROS 包，不依赖原始工作空间。正式场景测试使用 Gazebo GPU LiDAR 扫描家具、墙壁和动态障碍物。

## 动态障碍物

`house` 和 `cafe` 各包含三个由 `dynamic_obstacle_mover` 控制的模型：

1. 橙色圆柱；
2. 绿色方形长方体；
3. 蓝色细长长方体。

它们按照场景 profile 中的固定闭合路线运动，便于重复实验。`dynamic_seed` 保留用于实验标记和兼容旧模式；`route` 模式本身不会生成不可复现的随机路线。

静态测试传入 `dynamic_obstacles:=false` 后，节点会通过 Gazebo `set_pose` 服务把三个模型停到场景外，并发布空的动态障碍物真值消息。

## Linux GPU LiDAR

默认使用 GPU LiDAR 和 `ogre`：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch imperative_navigation imperative_m1_gazebo.launch.py \
  scene:=house \
  software_lidar:=false \
  render_engine:=ogre \
  dynamic_obstacles:=true
```

咖啡馆场景只需改为 `scene:=cafe`。

`render_engine` 默认值是 `ogre`。GPU LiDAR 仍使用机器人模型原有的 `gpu_lidar` 传感器，规划器从 `/scan` 读取真实 Gazebo 场景扫描结果。

## WSL fallback

如果当前环境无法使用 GPU LiDAR，可以运行：

```bash
ros2 launch imperative_navigation imperative_m1_gazebo.launch.py \
  scene:=house \
  software_lidar:=true \
  dynamic_obstacles:=true
```

此时 software LiDAR 发布 `/sim_scan`，支持场景边界、配置中的静态 primitive 以及三种动态障碍物形状。它不会完整重建 house 网格家具，因此只用于开发和接口调试，不用于正式避障结论。

## 静态 / 动态测试

记录一次测试：

```bash
ros2 launch imperative_navigation imperative_m1_gazebo.launch.py \
  scene:=cafe \
  software_lidar:=false \
  dynamic_obstacles:=true \
  record_performance:=true \
  performance_output_dir:=/tmp/imperative_cafe_dynamic_20260814
```

静态基线将 `dynamic_obstacles` 改为 `false`，每个场景建议使用至少三个固定 seed 重复运行。结果目录包含命令、里程计、动态障碍物轨迹和 `summary.json`；其中 `minimum_lidar_clearance_m` 反映实际 LiDAR 的最近返回距离。
