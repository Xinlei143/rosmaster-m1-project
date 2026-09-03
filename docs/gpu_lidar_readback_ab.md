# Fortress OGRE2 GPU-LiDAR readback A/B

This harness has two deliberately separate roles: a ROS-free **negative
control** for the GPU sensor, and a controlled local-library loader for a
legacy-versus-persistent GPU-to-CPU readback A/B.  It must not be described as
a standalone reproducer: the minimal world did not reproduce the whole-frame
`-Inf` failure.  The known reproducer is the stationary, full M1
Gazebo/Nav2/RViz workload documented in
[`gpu_lidar_diagnostics.md`](gpu_lidar_diagnostics.md).

## Minimal world: negative control

The two SDF files are identical except for horizontal FOV.  Both keep 667
beams and a 12 Hz update rate:

```text
gpu_lidar_minimal_360.sdf: [-pi, pi]
gpu_lidar_minimal_180.sdf: [-pi/2, pi/2]
```

Run a 10-second pilot against the system Fortress rendering libraries:

```bash
python3 src/m1_nav2_support/m1_nav2_support/gpu_lidar_readback_ab.py \
  --output-root /tmp/gpu-lidar-readback-system-pilot \
  --duration 10 --repeats 1 --case 360deg --mode persistent \
  --startup-grace 2
```

The runner needs permission to start Gazebo on a desktop Linux host.  A
restricted shell may fail before rendering with a `getifaddrs` or Gazebo log
directory error; that is an execution-environment failure, not a LiDAR
measurement.

The completed `3 x 300 s` runs for both 360 degrees and 180 degrees had zero
bad frames.  They show that neither long duration alone nor the nominal beam
count/FOV alone triggers the fault.  Keep them as a regression control, but
do not use a clean minimal-world result to claim that a rendering change fixed
the M1 failure.

## Local Fortress backport

The local source tree used for the A/B is:

```text
/home/xinlei/Data/ROS/gz-rendering6-pr1303/src/gz-rendering
```

It is based on the exact Fortress-era `ignition-rendering6` tag
`9ee7e969f885dc75267404a01f1911bc511c94f1`.  The only functional change is a
private `Ogre2GpuReadbackTicket` used by `Ogre2GpuRays::PostRender()`; it keeps
one `Ogre::AsyncTextureTicket` and reuses its staging allocation.  Setting
`GZ_RENDERING_OGRE2_LEGACY_READBACK=1` selects the original
`Ogre::Image2::convertFromTexture()` path; `0` selects the persistent path.

Build/install into a separate prefix (never `/usr`):

```bash
cmake -S /home/xinlei/Data/ROS/gz-rendering6-pr1303/src/gz-rendering \
  -B /home/xinlei/Data/ROS/gz-rendering6-pr1303/build-stageprobe -GNinja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_INSTALL_PREFIX=/home/xinlei/Data/ROS/gz-rendering6-pr1303/install-stageprobe \
  -DBUILD_TESTING=OFF
ninja -C /home/xinlei/Data/ROS/gz-rendering6-pr1303/build-stageprobe \
  ignition-rendering6-ogre2
cmake -DCMAKE_INSTALL_COMPONENT=Unspecified -P \
  /home/xinlei/Data/ROS/gz-rendering6-pr1303/build-stageprobe/ogre2/src/cmake_install.cmake
```

Use the component install above rather than a full aggregate install: the
Fortress-era tree can otherwise stop on an unrelated Ogre-1 compatibility
alias.  The local CMake adjustments create the Ogre and Ogre-2 compatibility
aliases in the build/install tree, not in the ROS workspace.  Do not repair
this by creating untracked aliases in the project root.

The runner sets `LD_LIBRARY_PATH`, `IGN_RENDERING_PLUGIN_PATH`, and the
Fortress resource root (`share/ignition/ignition-rendering6`) for this prefix.
Before a result is accepted, `status.json` must report
`library_validation.ok=true`; the validator reads `/proc/<Gazebo-child>/maps`
and requires both `libignition-rendering6.so` and the OGRE2 plugin to come from
the requested prefix.

## A/B commands and acceptance

First run one repeat of each mode to catch loader/resource errors:

```bash
PREFIX=/home/xinlei/Data/ROS/gz-rendering6-pr1303/install-stageprobe
python3 src/m1_nav2_support/m1_nav2_support/gpu_lidar_readback_ab.py \
  --output-root /tmp/gpu-lidar-readback-local-pilot \
  --duration 10 --repeats 1 --case 360deg --mode legacy --prefix "$PREFIX"
python3 src/m1_nav2_support/m1_nav2_support/gpu_lidar_readback_ab.py \
  --output-root /tmp/gpu-lidar-readback-local-pilot \
  --duration 10 --repeats 1 --case 360deg --mode persistent --prefix "$PREFIX"
```

The long minimal-world campaign is three 300-second repeats for each FOV and
readback mode.  Keep each mode in a separate output root (or run sequentially
with the same root) so failed artifacts remain attributable.  These are
negative-control runs, not the primary acceptance test for the M1 fault:

```bash
PREFIX=/home/xinlei/Data/ROS/gz-rendering6-pr1303/install-stageprobe
for MODE in legacy persistent; do
  python3 src/m1_nav2_support/m1_nav2_support/gpu_lidar_readback_ab.py \
    --output-root results/gpu_lidar_readback_${MODE} \
    --duration 300 --repeats 3 --case 360deg --mode "$MODE" \
    --prefix "$PREFIX"
  python3 src/m1_nav2_support/m1_nav2_support/gpu_lidar_readback_ab.py \
    --output-root results/gpu_lidar_readback_${MODE} \
    --duration 300 --repeats 3 --case 180deg --mode "$MODE" \
    --prefix "$PREFIX"
done
```

Each repeat is accepted only when it has nonzero frames, exactly 667 beams,
about 12 Hz, zero malformed frames, and explicit finite/`+Inf`/`-Inf`/NaN
counts.  A zero-frame capture is a failure even if the JSON analyzer itself
returns successfully.  Compare `gazebo_scan_summary.json`, `ogre2.log`, and
`resources.jsonl`; do not clamp or rewrite ranges.

## Measured M1 result

The primary experiment held the complete stationary M1 condition fixed:
native GPU LiDAR, M1 world, GUI on, RViz on, dynamic obstacles on, Nav2
running, and no navigation goals.  It recorded both Gazebo Transport `/scan`
and ROS `/scan` for 300 seconds.

| Library/readback condition | 300-second results | Interpretation |
| --- | --- | --- |
| System Fortress apt | 68.204%, 68.174%, 68.250% whole-frame `-Inf` | Baseline fault reproduces. |
| Local exact-tag legacy | 43.238%, 68.303%, 68.239% | Rebuilding locally is not a cure; one run has trigger-time variance. |
| Local persistent ticket | 67.640%, 68.257%, 68.257% | Persistent final readback does not fix the fault. |
| Local persistent, extra diagnostic run | 68.276% | The local prefix was verified from `/proc/<pid>/maps`; raw and ROS bad-frame states matched. |

The upstream change that motivated this backport is
[gz-rendering PR #1303](https://github.com/gazebosim/gz-rendering/pull/1303),
merged after Fortress.  In this environment its persistent-ticket idea is a
valid negative result, **not** a fix for the observed M1 `-Inf` failure.

An M1 180-degree A/B also remained bad (68.293%, 68.275%, and 66.012%), so a
single 180-degree GPU LiDAR is not a supported workaround.  The static
minimal 180-degree world remains useful only as a clean control.

### First-pass diagnostic boundary

The final texture is already all `-Inf` inside `Ogre2GpuRays::PostRender()`:
the observed transition was callback 1131 at simulation time 93.791 s and
continued through the capture.  An attempted first-pass
`AsyncTextureTicket` probe was intentionally deferred for two later render
callbacks, but it still did not return from the download path after the
transition: no `download_started` message or subsequent PostRender heartbeat
was emitted.  Because the probe changes the failing render path, it cannot be
used to assign a value to cubemap face 1.  Do not treat the earlier immediate
or deferred first-pass readbacks as proof that any particular face is corrupt.

### GPU-only second-pass sentinel A/B

To avoid another intrusive GPU-to-CPU readback, a self-contained temporary
Fortress prefix changed only the GLSL second-pass output after it sampled its
input:

```glsl
if (range < 0.0)
  range = 0.125;
```

All finite inputs remain untouched.  The 300-second stationary M1 run loaded
both `libignition-rendering6.so` and the OGRE2 plugin from that self-contained
prefix, so it was not a resource-overlay or mixed-library result.  It
produced 3565 raw frames and 3567 ROS frames at about 12.05 Hz, all with 667
beams and no `-Inf`, `+Inf`, or NaN.  The per-frame boundary was decisive:

| Frames | Raw output |
| --- | --- |
| 1–1131 | Normal finite ranges |
| 1132–3565 | 2434 complete frames, each 667 beams at `0.125 m` |

The unmodified persistent path changes at the same boundary to whole-frame
`-Inf`.  Therefore the value sampled by the second-pass shader (`d.x`) is
already negative for every final beam, and the shader can still write and the
final texture can still be read back.  This excludes the final
`secondPassTexture` GPU-to-CPU readback as the direct source of the all
`-Inf` frames.

It does **not** prove that one particular cubemap face is corrupt.  The
remaining upstream boundary is the first-pass cubemap render textures, the
`cubeUVTex` lookup texture, or OGRE2 compositor/resource scheduling before
the second-pass shader samples them.  The sentinel is a temporary diagnostic,
not a production range clamp or workaround.  The next useful control is a
Harmonic/gz-rendering8 test of the same M1 stationary condition, or a
GPU-only first-pass/cube-lookup sentinel that does not add a CPU readback.
