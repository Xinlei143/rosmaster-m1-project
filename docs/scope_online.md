# M1 SCOPE online observer

This package runs the frozen TempleRAIL SCOPE model beside Nav2. It reads
`/scan` and `/odom`; it does not publish velocity commands, modify a costmap,
or participate in MPPI planning.

## Runtime environment

The strict `scope-repro` environment remains the reference baseline. Build the
ROS 2 runtime in a separate Python 3.10 virtual environment:

```bash
python3 -m venv --system-site-packages .venv-scope-runtime
.venv-scope-runtime/bin/python -m pip install \
  -r src/m1_scope_predictor/requirements-cu124.txt
source /opt/ros/humble/setup.bash
.venv-scope-runtime/bin/python -m colcon build --packages-select \
  m1_scope_bridge m1_scope_predictor m1_nav2_bringup --symlink-install
source install/setup.bash
```

The package contains byte-for-byte copies of the two MIT-licensed model source
files from commit `211b199ebfcbe54eeac99202bbe62aca593f7bed`. The checkpoint
is external, hash-checked at startup, and is not committed to this repository.

## Standalone use

Start the existing Gazebo/Nav2 launch first, without its RViz instance:

```bash
ros2 launch m1_nav2_bringup nav2_m1_gazebo.launch.py gui:=true rviz:=false
```

Then start the observer and its dedicated RViz configuration:

```bash
ros2 launch m1_scope_predictor scope_online.launch.py \
  use_sim_time:=true rviz:=true \
  model_path:=/home/xinlei/Data/SCOPE-repro/reference/scope/model/scope_model.pth
```

The first prediction appears after the ten-frame history warms up. The default
uses a 0.5 s horizon and four Monte Carlo samples. Full SCOPE prioritizes a
fresh latest result over processing every nominal 10 Hz job.

To enable the Gazebo-only delayed evaluator, add
`evaluator_enabled:=true`. It uses `/ground_truth/odom` only to align the future
LaserScan with the prediction frame and reports whole endpoint-grid IoU, F1,
and MAE through `/scope/diagnostics`.

## Topics and frames

- `/scope/current_ogm`: current scan endpoints. `100` is occupied and `-1`
  means not marked by a scan endpoint; it must not be interpreted as free.
- `/scope/prediction`: dense future occupancy probability mapped to `0..100`.
- `/scope/uncertainty`: pixel standard deviation divided by `0.5`, clipped,
  and mapped to `0..100` for visualization only.
- `/scope/diagnostics`: history state, inference latency, result age, output
  rate, dropped jobs, allocated CUDA memory, and optional evaluator metrics.

All grids use the `odom` frame. The current grid origin is the current laser
pose composed with `(0,-3.2)` m; prediction and uncertainty use the predicted
future laser pose. Internally, arrays remain float tensors and are not
round-tripped through `OccupancyGrid`.

## Optional combined launch

After standalone validation, the same observer can be included without
changing the default Nav2 path:

```bash
ros2 launch m1_nav2_bringup nav2_m1_gazebo.launch.py \
  scope_enabled:=true scope_num_samples:=4
```

`scope_enabled` defaults to `false`. These outputs are visualization and
measurement artifacts only; they are not evidence of closed-loop navigation
benefit, real-time guarantees, or real-robot safety.
