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
