# Imperative M1 Gazebo Sim test

This package runs the original `Imperative_learning_2D_moving.py` planner against
Gazebo Sim sensor messages. It does not use the planner's internal obstacle or
LiDAR truth simulation.

## Build and run

From the `Rosmaster` directory:

```bash
source /opt/ros/humble/setup.bash
source proj_ws/install/setup.bash
ros2 launch imperative_navigation imperative_m1_gazebo.launch.py
```

If the workspace has not yet been built, build it once first:

```bash
cd /home/lin24311/car_ws/Rosmaster
source /opt/ros/humble/setup.bash
cd proj_ws
PYTHONPATH=/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH} \
  /home/lin24311/car_ws/.venv-ros2-ml/bin/python /usr/bin/colcon \
  build --packages-select imperative_navigation --symlink-install
```

This virtual environment has no `bin/activate` script, so the build command
explicitly uses its Python interpreter. The installed ROS 2 node will then use
the same interpreter and can access both `rclpy` and `torch`.

Gazebo displays the M1, room, and static / moving obstacles. A yellow static
cylinder at `(-2.05, -1.18)` forces an immediate start maneuver; the red
cylinder is parked at `(-3.0, 2.2)` and does not obstruct the normal route.
The three remaining cylinders begin around the goal, select randomized first
targets in the robot's route, and then patrol independently at 0.34, 0.42, and
0.50 m/s. Their target points and straight-line segments maintain clearance
from both static cylinders. RViz displays the bridged laser scan, tracked
obstacles, and the planner's selected local path.

On this WSLg + Gazebo Sim 6 installation, the OGRE GPU LiDAR produces a
saturated frame (all rays are its 0.08 m near clipping distance). The default
launch therefore enables `software_lidar`, which raycasts the Gazebo walls and
circular models and publishes a normal `/sim_scan` `LaserScan`. The controller
then follows its ordinary scan clustering, map update, tracking, and predicted
motion-planning path. This software raycaster is a WSLg simulation compatibility
layer, not a replacement for physical hardware perception. Disable it with
`software_lidar:=false` only on a host where the GPU lidar produces valid scans;
a saturated physical scan still commands a stop.

The controller publishes to `/cmd_vel` only in this simulation launch. For the
physical M1, the real controller instead writes `/imperative/cmd_vel_raw`; the
separately launched watchdog is the sole publisher connected to `/cmd_vel`.

## Rosmaster M1 physical robot

`imperative_m1_real.launch.py` is a separate hardware node.  It uses the
topics verified in `../../docs/小车nodes.doc`: `/scan` (`LaserScan` from YDLidar), `/odom`
(`Odometry` from the EKF), the `odom`/`laser` TF tree, and writes `Twist` to
`/imperative/cmd_vel_raw`. A separate watchdog is the only project publisher
of the M1 driver's `/cmd_vel`. It does **not** start Gazebo, use the Gazebo
software lidar, or accept simulator obstacle positions.

Start the normal M1 base, EKF, laser and TF launch first. Then build and source
the vendor workspace followed by this workspace. Start the watchdog in a
dedicated terminal **before** starting the planner; leave that terminal running
when stopping or restarting the planner. Goals are
coordinates in the current `/odom` frame (normally the start point is 0, 0):

```bash
cd /home/lin24311/car_ws/Rosmaster
source /opt/ros/humble/setup.bash
source yahboomcar_ros2_ws/install/setup.bash
source proj_ws/install/setup.bash
ros2 launch imperative_navigation imperative_cmd_watchdog.launch.py
```

In a second terminal, source the same workspaces and run the planner:

```bash
cd /home/lin24311/car_ws/Rosmaster
source /opt/ros/humble/setup.bash
source yahboomcar_ros2_ws/install/setup.bash
source proj_ws/install/setup.bash
ros2 launch imperative_navigation imperative_m1_real.launch.py \
  goal_x:=1.0 goal_y:=0.0 enabled:=true
```

The watchdog forwards fresh `/imperative/cmd_vel_raw` commands unchanged, but
after 0.60 s without a raw command it publishes a zero `Twist` to `/cmd_vel` at
20 Hz continuously. It also publishes zero commands before the controller has
started. Therefore Ctrl-C in the planner terminal cannot leave the last driver
command latched: the watchdog terminal remains running and transitions to zero
after the timeout. Confirm that the watchdog is the only `/cmd_vel` publisher:

```bash
ros2 topic info /cmd_vel --verbose
```

The output must list `/imperative_cmd_watchdog` as the only publisher. If any
other publisher is listed (for example teleoperation), stop it before testing.

### Setting a physical goal

`goal_x` and `goal_y` are absolute coordinates in the ROS `/odom` frame, in
metres. They are not a distance relative to where the robot happens to be when
the command is run. Check the current pose before a test:

```bash
ros2 topic echo /odom --once
```

Read `pose.pose.position.x` and `pose.pose.position.y`. The `/odom` origin is
normally set when the M1 EKF starts or is reset. Mark a known origin on the
floor, measure the target point from it in metres, and use those measured
coordinates. For example, a marked target at x = 1.20 m and y = -0.50 m is
started with:

```bash
ros2 launch imperative_navigation imperative_m1_real.launch.py \
  goal_x:=1.20 goal_y:=-0.50 enabled:=true
```

Verify the positive x/y directions first with a very short, clear-area manual
movement while watching `/odom`; do not assume the floor layout matches the
robot's initial heading. To send a different goal, stop the current navigation
node with Ctrl-C and launch it again with the new `goal_x` and `goal_y` values.

The physical launch is disabled by default. In this dry-run mode it continues
to process sensors, TF, obstacle tracks, planning, and visualization, but it
forces every `/cmd_vel` command to zero. It stops on a missing/stale scan
or odometry message, unavailable laser TF, a laser return at or below 0.45 m,
goal arrival, or Ctrl-C; `enabled=false` also forces a zero command after every
otherwise valid planning cycle. Its conservative indoor
first-test defaults are 0.18 m/s maximum speed, 0.25 m/s² maximum acceleration,
0.18 m robot radius, 0.18 m normal planning safety margin, 0.30 s maximum TF
age, and a 0.45 m emergency-stop distance. The controller
uses the latest available `odom` <- `laser` transform, but stops if it is older
than `tf_max_age`. Before enabling it, stop `joy_ctrl` and every other
publisher of `/cmd_vel`; the watchdog must remain its only publisher, so two
publishers would race each other. Tune `max_speed`, `max_acceleration`, and
`emergency_stop_distance` only after low-speed validation in an unobstructed
test area.

`robot_radius` is the physical footprint used by planning. `safety_margin` is
the additional normal planning clearance; together they form the planner's
collision radius. `emergency_stop_distance` remains a separate final laser
stop threshold and is not derived from either value. For a carefully measured,
low-speed narrow-space test, for example:

```bash
ros2 launch imperative_navigation imperative_m1_real.launch.py \
  goal_x:=0.50 goal_y:=0.00 enabled:=true \
  robot_radius:=0.18 safety_margin:=0.10 \
  emergency_stop_distance:=0.25 \
  max_speed:=0.20 max_acceleration:=0.50
```

The controller emits a low-frequency performance profile with its full planner
time and the LaserScan conversion, detection, tracking, map update, planning,
motion-step, and visualization stage times. `torch_num_threads:=0` keeps the
PyTorch default; it can be changed only for on-robot performance benchmarking
without changing the planner's mathematical result.

Both adapters set the planner's `DT` from the actual elapsed time between
planner callbacks. This keeps track estimation and rollout timing aligned when
the RDK X5 cannot finish a callback at the configured 10 Hz. To compare Gazebo
with the current narrow-space physical settings, launch it with the same
geometry and motion parameters:

```bash
ros2 launch imperative_navigation imperative_m1_gazebo.launch.py \
  max_speed:=0.20 max_acceleration:=0.50 \
  robot_radius:=0.18 safety_margin:=0.05
```

For RViz inspection, the node publishes `/imperative/planned_path`,
`/imperative/tracks`, and `/imperative/obstacle_centers` in the `odom` frame.
