  cd ~/car_ws2/rosmaster-m1-project
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  PYTHONPATH="/home/lin24311/car_ws/.venv-ros2-ml/lib/python3.10/site-packages:$PYTHONPATH" \
  ros2 launch imperative_navigation imperative_m1_gazebo.launch.py


  • 实机 2 m 前向规划已实现并构建完成。

  新增了专用启动文件 src/imperative_navigation/launch/imperative_m1_straight_2m.launch.py，控制器会在第一帧有效 /odom 时
  锁定“初始车头前方 2 m”目标；复用完整仿真规划器，速度限制为 0.12 m/s、加速度 0.20 m/s²。到点后持续发布零速度，watchdog
  仍独立运行。

  核心改动在 src/imperative_navigation/imperative_navigation/m1_controller_node.py，并新增相对目标单元测试。相关测试：10
  passed。

  实机运行顺序：

  # 终端 1：先启动厂家底盘、雷达、EKF/odom 与 TF

  # 终端 2：watchdog 必须持续运行
  cd ~/car_ws2/rosmaster-m1-project
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch imperative_navigation imperative_cmd_watchdog.launch.py

  # 终端 3：先 dry-run，不会动
  cd ~/car_ws2/rosmaster-m1-project
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  PYTHONPATH="/home/lin24311/car_ws/.venv-ros2-ml/lib/python3.10/site-packages:$PYTHONPATH" \
  ros2 launch imperative_navigation imperative_m1_straight_2m.launch.py

  确认 /scan、/odom、TF、规划轨迹正常且场地安全后：

  PYTHONPATH="/home/lin24311/car_ws/.venv-ros2-ml/lib/python3.10/site-packages:$PYTHONPATH" \
  ros2 launch imperative_navigation imperative_m1_straight_2m.launch.py enabled:=tru