# Native GPU LiDAR `-Inf` diagnosis

This procedure locates invalid ranges before changing Nav2. It keeps the
native Gazebo sensor enabled (`software_lidar:=false`) and captures the raw
Gazebo Transport `/scan` independently from the ROS topic produced by
`ros_gz_bridge`.

Build and source the support package first:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select m1_nav2_support --symlink-install
source install/setup.bash
```

For the raw-only scene-isolation matrix (Test A and R0--R5), run the
server-only collector.  It captures the native Gazebo Transport topic directly
and deliberately does not start `m1_gazebo.launch.py`, Nav2,
`ros_gz_bridge`, RViz, or the obstacle mover:

```bash
ros2 run m1_nav2_support gpu_lidar_scene_matrix \
  --output-root results/gpu_lidar_scene_matrix_20260903 \
  --duration 180 --repeats 3 --bad-streak 60 --max-wall-seconds 420
```

The runner stores each repeat's raw JSON, Gazebo logs, expanded/converted
model input, metadata, and `status.json`.  Its top-level summary is rebuilt
from all durable per-repeat status files, so separately launched case groups
remain one auditable matrix.  A bad-streak early stop is only 60 consecutive
667-beam frames that are all `-Inf`; partial `-Inf` is measured separately.

The older GUI/bridge matrix below is retained as historical evidence and is
not a substitute for the raw-only scene-isolation matrix:

Run a short pilot while the desktop display is available:

```bash
python3 scripts/run_gpu_lidar_matrix.py \
  --output-root /tmp/m1-gpu-lidar-pilot \
  --duration 15 --repeats 1
```

The complete matrix uses three 60-second repeats per condition:

```bash
python3 scripts/run_gpu_lidar_matrix.py \
  --output-root results/gpu_lidar_matrix \
  --duration 60 --repeats 3
```

Aggregate an existing run without restarting Gazebo:

```bash
python3 scripts/summarize_gpu_lidar_matrix.py \
  --root results/gpu_lidar_matrix \
  --json-output results/gpu_lidar_matrix/matrix_summary.json \
  --markdown-output results/gpu_lidar_matrix/matrix_summary.md
```

Each repeat starts only `m1_gazebo.launch.py`, with a stationary robot,
`dynamic_obstacles:=false`, `slip_enabled:=false`, and the native `ogre2`
engine. It stores `gazebo_scan.jsonlog` and
`gazebo_scan_summary.json` for the raw message, and
`ros_scan_summary.json` for the bridge-side `sensor_msgs/LaserScan`. Both
summaries classify finite, `+Inf`, `-Inf`, and NaN separately. Gazebo launch,
RViz, bridge/diagnostic logs, `ogre2.log`, and renderer/GPU metadata stay in
the same repeat directory.

Interpretation:

* If raw Gazebo and ROS summaries show the same `-Inf` ratio, investigate
  OGRE2/OpenGL/device selection; the bridge is not the source.
* If raw Gazebo is finite but ROS `/scan` has `-Inf`, investigate the bridge
  conversion/version and compare the saved raw and ROS frames.
* Do not clamp, restamp, or replace invalid ranges in the diagnostic path.

The support launch accepts `render_engine:=...` for a controlled A/B test and
defaults to `ogre2`. An engine is considered a valid candidate only if it
supports this GPU sensor and preserves the expected 667 beams at about 12 Hz.

## 2026-09-03 evidence

The completed run is in `results/gpu_lidar_matrix`. All 18 repeats finished
successfully. Across all six GUI/RViz combinations, both the raw Gazebo side
and the bridge-side ROS side reported zero `-Inf`, zero `+Inf`, zero NaN, 667
beams, and approximately 12 Hz. The result is therefore **no reproduction in
that matrix**, not proof that the earlier full-navigation failure has
disappeared.

The host for this evidence is native Linux with an NVIDIA RTX 4060 (not WSLg).
The static, ROS-free 667-beam 360°/180° worlds documented in
[`gpu_lidar_readback_ab.md`](gpu_lidar_readback_ab.md) were clean for
`3 x 300 s` each.  They are therefore a negative control, not the full
failure reproducer.

The full stationary M1 workload *does* reproduce: system Fortress apt runs
had 68.204%, 68.174%, and 68.250% whole-frame `-Inf`; raw Gazebo and ROS
`/scan` matched on every common timestamp.  A local exact-tag Fortress build
reproduced in legacy mode, while its persistent `AsyncTextureTicket` final
readback mode remained at 67.640%, 68.257%, and 68.257%.  Thus neither Nav2,
the bridge, nor PR #1303-style final readback allocation churn is established
as the source or a fix.  The current evidence boundary is the Fortress OGRE2
GPU-LiDAR render/readback chain under the complete M1 workload.

For the same full stationary condition, an M1 180-degree GPU LiDAR A/B still
produced 68.293%, 68.275%, and 66.012% whole-frame `-Inf`.  Do not propose a
single 180-degree sensor as a workaround.  An opt-in cubemap first-pass
readback probe was also not accepted as localization evidence: even a
deferred `AsyncTextureTicket` download perturbed the failure transition.

The non-invasive discriminator is now complete.  In a self-contained local
Fortress prefix, only the second-pass GLSL shader was changed to map a
negative sampled `range` to finite `0.125 m`; no extra GPU-to-CPU transfer was
added.  The exact 300-second stationary M1 run loaded both core and OGRE2
plugin from that prefix and produced 3565 raw/3567 ROS frames at about 12.05
Hz with no non-finite values.  Frames 1–1131 retained normal finite ranges;
frames 1132–3565 were all 667-beam `0.125 m` sentinel frames.  The unchanged
path turns to whole-frame `-Inf` at the same boundary.  Thus the value entering
the second-pass shader is already negative, while the second-pass output
texture and final readback remain functional.  The fault is now bounded to
the first-pass cubemap textures, `cubeUVTex` lookup, or OGRE2 scheduling /
resource state before the second pass—not Nav2, `ros_gz_bridge`, or the final
GPU-to-CPU readback.  This diagnostic sentinel is not a production clamp or
workaround.  A Harmonic/gz-rendering8 run of the same stationary M1 condition
is the next environmental control.

The captured OGRE2 logs consistently selected NVIDIA GeForce RTX 4060 Laptop
GPU with NVIDIA 580.173.02 / OpenGL 4.5. They also consistently contained two
EGL device-initialization failures and mesh tangent/material warnings, but
those warnings did not correlate with invalid scan values. The `ogre` pilot
produced finite data but logged a 180-degree GPU-rays FOV cap, so it is not a
valid replacement for this 360-degree sensor. `LIBGL_ALWAYS_SOFTWARE=1` was
ignored because the EGL path explicitly selected a hardware device; it did
not constitute a Mesa backend test.

## 2026-09-04 raw-only scene-isolation result

The first-round matrix is stored in
[`results/gpu_lidar_scene_matrix_20260903`](../results/gpu_lidar_scene_matrix_20260903).
It contains Test A plus R0--R5, with `3 x 180 s` repetitions per condition.
All 21 repetitions were valid: each had exactly one native scan publisher,
667 beams in every frame, no malformed frame, a successful Gazebo server and
raw-capture exit, and a simulation duration between 180.004 and 180.079 s
(2,213--2,222 frames per repetition).  The simulation-stamp gap p50, p95,
and maximum were all 0.083 s; the acceptance check intentionally uses these
steady-cadence values rather than the startup-sensitive endpoint rate.

| condition | whole-frame `-Inf` | partial `-Inf` result |
|---|---:|---|
| Test A: static M1, raw-only | 0 | none |
| R0: minimal world, world-loaded primitive | 0 | none |
| R1: minimal world, dynamic primitive | 0 | none |
| R2: M1 world, dynamic primitive | 0 | none |
| R3: minimal world, dynamic full M1 | 0 | reproducible: repeat mean 0.566--0.568%; one frame at most 9.895% |
| R4: M1 world, dynamic full M1 | 0 | none |
| R5: M1 world, world-loaded full M1 | 0 | none |

Thus this matrix **did not reproduce the historical whole-frame latch-up**:
there were no good-to-bad whole-frame transitions, recoveries, or all-negative
frames in any repeat.  It does show a smaller, reproducible partial-range
phenomenon in R3 that is absent from R4, even though both use a dynamically
created full M1.  That is a scene/model interaction lead for a future OGRE2
render-path comparison, not evidence that it is the historical latch-up's
root cause and not authorization to change Nav2, the bridge, shaders, or the
production sensor path.
