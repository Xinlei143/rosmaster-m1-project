# Rosmaster M1 Imperative Navigation

这是一个基于 ROS 2 Humble 的 Rosmaster M1 二维导航项目。核心算法使用激光雷达和里程计进行障碍物检测、动态目标跟踪、位姿辅助点地图更新，并通过短时滚动时域选择速度指令。

项目支持两种运行方式：

- Gazebo Sim 仿真：启动虚拟 M1、静态/动态障碍物、ROS bridge、规划器和 RViz；
- Rosmaster M1 实机：接入真实 `/scan`、`/odom` 和 TF，并通过独立 watchdog 安全输出 `/cmd_vel`。

详细算法说明和常用命令也可以查看：

- [`docs/算法框架说明.md`](docs/算法框架说明.md)
- [`docs/常见运行命令.md`](docs/常见运行命令.md)

## 1. 项目结构

```text
src/imperative_navigation/
├── algorithm/
│   └── Imperative_learning_2D_moving.py   # 核心检测、跟踪和规划算法
├── imperative_navigation/
│   ├── controller_node.py                 # Gazebo/通用仿真适配器
│   ├── m1_controller_node.py              # Rosmaster M1 实机适配器
│   ├── cmd_vel_watchdog_node.py           # 实机速度安全看门狗
│   ├── software_lidar.py                  # Gazebo 软件 LiDAR
│   ├── dynamic_obstacle_mover.py          # Gazebo 动态障碍物
│   └── algorithm_loader.py                 # 加载核心算法
├── launch/
│   ├── imperative_m1_gazebo.launch.py     # Gazebo + bridge + planner + RViz
│   ├── imperative_m1_real.launch.py       # 实机规划器
│   └── imperative_cmd_watchdog.launch.py  # 实机 watchdog
├── worlds/imperative_m1.sdf               # Gazebo 世界和 M1 模型
└── rviz/imperative.rviz                   # RViz 默认配置
```

## 2. 算法概览

每个控制周期大致执行以下流程：

```text
LaserScan + Odometry/TF
        ↓
坐标转换和输入时效检查
        ↓
激光点聚类与圆形障碍物拟合
        ↓
障碍物关联、速度估计和动态轨迹确认
        ↓
位姿辅助点地图更新
        ↓
滚动时域候选轨迹评价
        ↓
选择加速度 → 更新速度 → 转换到机体坐标系
        ↓
/cmd_vel
```

核心规划器不是 Nav2 全局规划器，而是基于当前局部观测的滚动时域规划器。它使用当前点云、历史点地图和动态障碍物的匀速预测，在多个候选加速度计划中选择总代价最低的计划。

项目使用 PyTorch 做张量和向量计算，但没有加载神经网络模型，也没有训练流程。`torch` 在这里主要用于位置、速度、距离、轨迹预测和代价计算。

## 3. Python 和 ROS 2 环境

### 3.1 推荐环境

当前推荐使用：

```text
ROS 2 Humble
系统 Python 3.10：运行 rclpy 和构建 ROS 包
pendulum-rl Python 3.10 环境：提供 torch 和 numpy
```

`lerobot` 环境使用 Python 3.12，不能直接加载 ROS 2 Humble 的 Python 3.10 版本 `rclpy`，不建议用于本项目 ROS 节点。

### 3.2 构建项目

必须从项目根目录构建。注意：应直接使用系统 `colcon`，不要使用 `python /usr/bin/colcon`，否则容易把 Conda 的 setuptools 和 ROS 的 setuptools 混用。

```bash
cd /home/xinlei/Data/ROS/rosmaster-m1-project

source /opt/ros/humble/setup.bash
unset PYTHONPATH

/usr/bin/colcon build \
  --packages-select imperative_navigation \
  --symlink-install
```

构建完成后加载工作空间：

```bash
source /home/xinlei/Data/ROS/rosmaster-m1-project/install/setup.bash
```

检查包是否安装成功：

```bash
ros2 pkg prefix imperative_navigation
ros2 pkg executables imperative_navigation
```

### 3.3 运行时加载 Torch

如果系统 Python 中已经可以导入 Torch，可以跳过这一步。否则使用 `pendulum-rl` 中的 Torch：

```bash
conda activate pendulum-rl
source /opt/ros/humble/setup.bash
source /home/xinlei/Data/ROS/rosmaster-m1-project/install/setup.bash

export PYTHONPATH=/home/xinlei/Data/robotics_ws/miniconda3/envs/pendulum-rl/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}
```

验证运行时依赖：

```bash
/usr/bin/python3 -c "import torch, rclpy; print('torch:', torch.__version__); print('rclpy: ok')"
```

这里节点脚本显示 `#!/usr/bin/python3` 是正常的：ROS 2 使用系统 Python 加载 `rclpy`，再通过 `PYTHONPATH` 加载 `pendulum-rl` 中的 Torch。

## 4. Gazebo 仿真

### 4.1 启动完整仿真

推荐在终端中执行：

```bash
conda activate pendulum-rl
source /opt/ros/humble/setup.bash
source /home/xinlei/Data/ROS/rosmaster-m1-project/install/setup.bash
export PYTHONPATH=/home/xinlei/Data/robotics_ws/miniconda3/envs/pendulum-rl/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}

ros2 launch imperative_navigation imperative_m1_gazebo.launch.py
```

该 launch 会启动：

- Gazebo Sim 6；
- `ros_gz_bridge`；
- `dynamic_obstacle_mover`；
- `software_lidar`；
- `imperative_controller`；
- RViz。

默认目标点为 `(2.5, 1.5)`。Gazebo 世界中的 M1 初始位置约为 `(-2.5, -1.5)`，场景中还包含墙体、静态圆柱和动态圆柱。

### 4.2 常用仿真参数

```bash
# 不启动 RViz，只测试 Gazebo 和导航节点
ros2 launch imperative_navigation imperative_m1_gazebo.launch.py rviz:=false

# 不显示 Gazebo GUI，适合远程或无显示环境
ros2 launch imperative_navigation imperative_m1_gazebo.launch.py gui:=false

# 使用更保守的速度和机器人安全距离
ros2 launch imperative_navigation imperative_m1_gazebo.launch.py \
  max_speed:=0.20 \
  max_acceleration:=0.50 \
  robot_radius:=0.18 \
  safety_margin:=0.10

# 仅当 GPU LiDAR 已确认正常时关闭软件 LiDAR
ros2 launch imperative_navigation imperative_m1_gazebo.launch.py \
  software_lidar:=false
```

当前 WSLg + Gazebo Sim 6 环境中，GPU LiDAR 可能把所有光束都返回为最小量程 `0.08 m`。因此默认开启 `software_lidar`，它根据 Gazebo 中的墙体和圆柱进行二维射线检测，并发布 `/sim_scan`。

软件 LiDAR 是仿真兼容层，不代表实机使用仿真障碍物真值。实机模式不会启动或使用它。

### 4.3 Gazebo 暂停检查

控制器使用仿真时间。如果 Gazebo 暂停，`/clock` 不推进，ROS 2 定时器也不会正常执行，小车会表现为完全不动。

```bash
ros2 topic echo /clock --once
ros2 topic hz /clock
```

如果没有消息，请点击 Gazebo 窗口的播放按钮，确保仿真没有暂停。

## 5. ROS 话题和节点

### 5.1 Gazebo 话题链路

```text
Gazebo MecanumDrive
        ↑
ros_gz_bridge ← /cmd_vel ← imperative_controller

Gazebo OdometryPublisher → /odom → imperative_controller/software_lidar
Gazebo LiDAR               → /scan
software_lidar             → /sim_scan → imperative_controller
```

### 5.2 主要话题

| 话题 | 类型 | 作用 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Gazebo 或底盘最终速度输入 |
| `/imperative/cmd_vel_raw` | `geometry_msgs/msg/Twist` | 实机规划器的原始速度输出 |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo bridge 或实机 LiDAR 扫描 |
| `/sim_scan` | `sensor_msgs/msg/LaserScan` | Gazebo 软件 LiDAR 扫描 |
| `/odom` | `nav_msgs/msg/Odometry` | 机器人位置、姿态和速度 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo 仿真时间 |
| `/imperative/planned_path` | `nav_msgs/msg/Path` | 当前滚动计划路径 |
| `/imperative/tracks` | `visualization_msgs/msg/MarkerArray` | 已确认障碍物轨迹 |
| `/imperative/obstacle_centers` | `geometry_msgs/msg/PoseArray` | 实机适配器发布的障碍物中心 |

### 5.3 常用检查命令

```bash
ros2 node list
ros2 topic list

ros2 topic info /cmd_vel --verbose
ros2 topic info /odom --verbose
ros2 topic info /sim_scan --verbose

ros2 topic hz /clock
ros2 topic hz /odom
ros2 topic hz /sim_scan
ros2 topic hz /cmd_vel

ros2 topic echo /odom --once
ros2 topic echo /sim_scan --once
ros2 topic echo /cmd_vel
```

正常仿真中应能看到 `/imperative_controller`、`/parameter_bridge`、`/software_lidar` 和 `/dynamic_obstacle_mover` 等节点。

## 6. 小车不动时的排查顺序

### 6.1 `/cmd_vel` 没有发布者

```bash
ros2 topic info /cmd_vel --verbose
```

如果显示：

```text
Publisher count: 0
Subscription count: 1
```

说明 bridge 在等待速度，但 `imperative_controller` 没有启动成功。检查：

```bash
ros2 node list
ros2 node info /imperative_controller
```

同时查看启动 Gazebo 的终端中是否有：

```text
process has died
ModuleNotFoundError
Traceback
```

### 6.2 有节点但没有消息

先检查仿真时间：

```bash
ros2 topic hz /clock
```

再检查输入：

```bash
ros2 topic hz /odom
ros2 topic hz /sim_scan
```

节点和话题存在，不代表消息正在流动；`/clock`、`/odom` 和 `/sim_scan` 都应有持续消息。

### 6.3 直接测试 Gazebo 底盘插件

这个测试只适用于仿真，用来绕过规划器：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

如果直接发送速度后小车移动，说明 Gazebo 底盘和 bridge 正常，问题在规划器输出；如果仍然不动，应检查 Gazebo 是否暂停以及 MecanumDrive 插件是否收到 Gazebo Transport 消息。

停止测试并发送零速度：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 6.4 RViz 丢弃激光消息

类似下面的日志：

```text
Message Filter dropping message ... queue is full
```

通常表示激光帧 `m1/base_link/lidar` 到 RViz 固定坐标系 `odom` 的 TF 不完整，主要影响激光显示，不一定会阻止 Gazebo 运动。应先检查：

```bash
ros2 topic echo /tf --once
ros2 run tf2_ros tf2_echo odom base_link
```

## 7. Rosmaster M1 实机

本项目不直接操作串口、电机或底盘 MCU。实机通信链路为：

```text
imperative_m1_controller
        ↓
/imperative/cmd_vel_raw
        ↓
imperative_cmd_watchdog
        ↓
/cmd_vel
        ↓
厂家底盘驱动
        ↓
M1 电机底盘
```

厂家底盘、EKF、YDLidar 和 TF launch 不包含在本仓库中，需要先启动本机的厂家工作空间。

### 7.1 启动厂家底盘

具体 launch 文件名取决于本机厂家工作空间。启动后应确认存在：

```text
/scan                  sensor_msgs/msg/LaserScan
/odom                  nav_msgs/msg/Odometry
odom -> laser 的 TF
底盘对 /cmd_vel 的订阅
```

### 7.2 启动 watchdog

在独立终端中运行，并保持该终端一直开启：

```bash
conda activate pendulum-rl
source /opt/ros/humble/setup.bash
source /path/to/yahboomcar_ros2_ws/install/setup.bash
source /home/xinlei/Data/ROS/rosmaster-m1-project/install/setup.bash
export PYTHONPATH=/home/xinlei/Data/robotics_ws/miniconda3/envs/pendulum-rl/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}

ros2 launch imperative_navigation imperative_cmd_watchdog.launch.py
```

watchdog 默认以 `20 Hz` 发布 `/cmd_vel`。如果超过 `0.60 s` 没有收到 `/imperative/cmd_vel_raw`，就会强制发布零速度。

确认它是 `/cmd_vel` 的唯一发布者：

```bash
ros2 topic info /cmd_vel --verbose
```

如果还有手柄、键盘遥控等节点发布 `/cmd_vel`，必须先停止它们。

### 7.3 实机 dry-run

实机规划器默认 `enabled=false`。此模式会运行感知、跟踪、规划和可视化，但不会让小车运动：

```bash
conda activate pendulum-rl
source /opt/ros/humble/setup.bash
source /path/to/yahboomcar_ros2_ws/install/setup.bash
source /home/xinlei/Data/ROS/rosmaster-m1-project/install/setup.bash
export PYTHONPATH=/home/xinlei/Data/robotics_ws/miniconda3/envs/pendulum-rl/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}

ros2 launch imperative_navigation imperative_m1_real.launch.py \
  goal_x:=1.0 goal_y:=0.0 enabled:=false
```

### 7.4 启用实机运动

确认传感器、TF、watchdog 和 `/cmd_vel` 发布者都正常后，才在空旷区域显式启用：

```bash
ros2 launch imperative_navigation imperative_m1_real.launch.py \
  goal_x:=1.0 goal_y:=0.0 enabled:=true
```

`goal_x` 和 `goal_y` 是 `/odom` 坐标系中的绝对坐标，不是相对当前点的位移。运行前查看当前位姿：

```bash
ros2 topic echo /odom --once
```

实机默认安全参数包括：最大速度 `0.18 m/s`、最大加速度 `0.25 m/s²`、机器人半径 `0.18 m`、规划安全裕量 `0.18 m`、TF 最大年龄 `0.30 s`、激光急停距离 `0.45 m`。

## 8. 离线运行核心算法

该命令不启动 ROS、Gazebo 或实机，只运行算法文件内部的仿真障碍物和 LiDAR：

```bash
conda activate pendulum-rl
python src/imperative_navigation/algorithm/Imperative_learning_2D_moving.py
```

## 9. 其他说明

- Gazebo 仿真中，规划器直接发布 `/cmd_vel`；
- 实机中，规划器只发布 `/imperative/cmd_vel_raw`，watchdog 才发布 `/cmd_vel`；
- 修改 Python 源码后，使用 `--symlink-install` 构建可以减少复制，但构建时仍应使用系统 `/usr/bin/colcon`；
- `robot_radius` 和 `safety_margin` 决定规划碰撞距离，`emergency_stop_distance` 是独立的最终激光急停阈值；
- Gazebo 使用仿真时间，实机使用真实 ROS 时间；
- 运行实机前必须先做 dry-run 和低速、空旷区域测试。
