# M1 motion diagnostics

These tests must run with exactly one Gazebo instance. Before starting an
isolated test, stop the interactive Nav2 launch with `Ctrl-C`; use a separate
`ROS_DOMAIN_ID` and `GZ_PARTITION` only to avoid accidental topic mixing, not to
run two simulations at once.

Build the workspace first:

```bash
colcon build --packages-select yahboomcar_description m1_nav2_support m1_nav2_bringup --symlink-install
source install/setup.bash
```

## 1. Physics-only straight line

Start Gazebo without Nav2, using native GPU LiDAR and no moving obstacles:

```bash
export ROS_DOMAIN_ID=83
export GZ_PARTITION=m1_physics_diagnostic
ros2 launch m1_nav2_support m1_gazebo.launch.py gui:=false software_lidar:=false dynamic_obstacles:=false
```

In another terminal with the same environment, run:

```bash
ros2 run m1_nav2_support m1_motion_diagnostic --ros-args \
  -p mode:=physics_straight \
  -p duration_seconds:=10.0 \
  -p command_x:=0.20 \
  -p output:=/tmp/m1_physics_straight.json
```

The important fields are `forward_distance_m`, `lateral_max_abs_m`,
`yaw_max_abs_rad`, and `command_topics`. A valid run should move roughly 2 m,
keep lateral drift below 0.05 m, and keep yaw drift below 0.035 rad. If the
forward distance is nearly zero, diagnose the `/cmd_vel` bridge before judging
the contact model.

## 2. Nav2 straight-line baseline

Restart with the complete Nav2 launch, then run a target below the central
obstacle so the global path is straight:

```bash
ros2 launch m1_nav2_bringup nav2_m1_gazebo.launch.py \
  gui:=false rviz:=false dynamic_obstacles:=false software_lidar:=false
```

After lifecycle activation:

```bash
ros2 run m1_nav2_support m1_motion_diagnostic --ros-args \
  -p mode:=nav_goal \
  -p goal_x:=-0.80 -p goal_y:=-1.50 -p goal_yaw:=0.0 \
  -p goal_timeout_seconds:=45.0 \
  -p output:=/tmp/m1_nav_straight.json
```

Compare `command_topics` with the odometry trajectory. Nonzero
`/cmd_vel_smoothed` followed by zero `/m1/cmd_vel_raw` identifies a downstream
Collision Monitor veto. Nonzero `vy/wz` already in `/cmd_vel_nav` identifies
MPPI or localization behavior.

## 3. Near-obstacle goal matrix

Use the same `nav_goal` command with the following cases, changing only the
goal parameters. `approach_yaw` is the expected arrival direction used by the
preflight footprint check.

```text
goal distance from (0,0): 0.50, 0.60, 0.75 m
goal yaw: 0.0 and 1.5708 rad
goal x: -distance
goal y: 0.0
approach_yaw: 0.0
```

Each JSON result reports `goal_preflight.final_footprint_free` and
`goal_preflight.rotation_sweep_free`, plus the actual action result. A
footprint-feasible case must finish; an infeasible terminal rotation must be
reported as such instead of being interpreted as a costmap display failure.

