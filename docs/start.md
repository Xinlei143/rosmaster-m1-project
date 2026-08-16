  cd ~/car_ws2/rosmaster-m1-project
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  PYTHONPATH="/home/lin24311/car_ws/.venv-ros2-ml/lib/python3.10/site-packages:$PYTHONPATH" \
  ros2 launch imperative_navigation imperative_m1_gazebo.launch.py