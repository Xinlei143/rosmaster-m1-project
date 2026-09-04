# Fortress Ogre2GpuRays single-camera backport

基线为 `ignition-rendering6_6.6.4`（`9ee7e969`）。仓库中的 patch 只修改
`ogre2/src/Ogre2GpuRays.cc`，不包含 shader、readback 或传感器参数改动。

```bash
mkdir -p /tmp/gz-rendering6-singlecam-20260904/src
git -C /home/xinlei/Data/ROS/gz-rendering6-pr1303/src/gz-rendering \
  archive ignition-rendering6_6.6.4 \
  | tar -x -C /tmp/gz-rendering6-singlecam-20260904/src
git -C /tmp/gz-rendering6-singlecam-20260904/src apply \
  /home/xinlei/Data/ROS/rosmaster-m1-project/patches/ignition-rendering6-6.6.4-single-cubemap-camera.patch
cmake -S /tmp/gz-rendering6-singlecam-20260904/src \
  -B /tmp/gz-rendering6-singlecam-20260904/build -GNinja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
  -DCMAKE_INSTALL_PREFIX=/tmp/gz-rendering6-singlecam-20260904/prefix
ninja -C /tmp/gz-rendering6-singlecam-20260904/build \
  ignition-rendering6 ignition-rendering6-ogre2 INTEGRATION_gpu_rays
```

本次实际构建安装于 `/tmp/gz-rendering6-singlecam-20260904/prefix`。由于 pilot
已复现持续整帧 `-Inf`，按失败门槛没有执行 `3 x 600 s`。
