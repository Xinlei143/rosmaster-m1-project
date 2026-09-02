# Rosmaster M1 Imperative 避障

`imperative` 分支以 Imperative 局部避障算法为主。默认仿真入口为
`imperative_navigation` 包的独立启动文件；算法使用 LaserScan 相邻回波聚类、`[x, y, vx, vy]`
常速度 Kalman 跟踪与 20 步滚动时域候选轨迹，动态障碍物未来位置采用常速度外推，而非学习模型。

- 底盘：M1 麦克纳姆轮，控制输出包含 `linear.x`、`linear.y` 与 `angular.z`；
- 障碍输入：`/scan`；
- 仿真控制输出：Gazebo 直接使用 `/cmd_vel`；
- 可视化与算法细节：[`docs/imperative_navigation.md`](docs/imperative_navigation.md)。

仓库保留 Nav2 MPPI 启动文件作为对照基线，但它不是本分支的默认方法。同一时刻只能启动
Imperative 或 Nav2 Gazebo 控制器之一，不能同时运行。

## 构建

```bash
cd /home/xinlei/Data/ROS/rosmaster-m1-project
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-numpy python3-matplotlib python3-pytest \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-mppi-controller \
  ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces \
  ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-xacro

unset PYTHONPATH

/usr/bin/colcon build --packages-select \
  yahboomcar_description m1_nav2_support m1_nav2_bringup imperative_navigation \
  --symlink-install
source /home/xinlei/Data/ROS/rosmaster-m1-project/install/setup.bash
```

## Python 与 PyTorch 运行环境

ROS 2 Humble 的 `rclpy` 使用系统 Python 3.10；Imperative 的张量计算使用 `pendulum-rl` 环境中的
PyTorch 和 NumPy。不要直接用 Python 3.12 的 Conda 环境加载 ROS 2 节点。

```bash
conda activate pendulum-rl
source /opt/ros/humble/setup.bash
source /home/xinlei/Data/ROS/rosmaster-m1-project/install/setup.bash
export PYTHONPATH=/home/xinlei/Data/robotics_ws/miniconda3/envs/pendulum-rl/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}

/usr/bin/python3 -c "import torch, numpy, rclpy; print('torch:', torch.__version__); print('numpy:', numpy.__version__); print('rclpy: ok')"
```

构建使用 `/usr/bin/colcon`，运行 ROS 节点使用 `/usr/bin/python3`；仅通过 `PYTHONPATH` 引入
`pendulum-rl` 的 `torch`，这样不会把系统 `rclpy` 与 Conda 的 Python 版本混用。

## Imperative Gazebo 与 RViz 仿真

```bash
ros2 launch imperative_navigation imperative_m1_gazebo.launch.py \
  gui:=true rviz:=true dynamic_obstacles:=true software_lidar:=false
```

Gazebo 启动后，在 RViz 选择 **2D Goal Pose** 设置目标点。Imperative 控制器依据实时 `/scan`
跟踪动态障碍物并生成局部候选轨迹；缺少有效传感器、里程计或 TF 数据时会保持安全零指令。

## Nav2 MPPI 对照基线

如需运行保留的 Nav2 MPPI 基线，可使用以下 Gazebo 入口：

```bash
ros2 launch m1_nav2_bringup nav2_m1_gazebo.launch.py \
  gui:=true rviz:=true dynamic_obstacles:=true software_lidar:=false
```

实机 Nav2 基线要求底盘驱动提供 `/scan`、`/odom` 与 `odom → base_footprint` TF：

```bash
ros2 launch m1_nav2_bringup nav2_m1_real.launch.py rviz:=true
```

## Imperative 实机入口

实机入口默认是干运行，只有显式设置 `enabled:=true` 才会输出物理原始指令；watchdog 应在独立终端
运行：

```bash
ros2 launch imperative_navigation imperative_cmd_watchdog.launch.py
ros2 launch imperative_navigation imperative_m1_real.launch.py enabled:=false
```
