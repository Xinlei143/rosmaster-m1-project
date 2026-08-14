#!/usr/bin/env bash
set -euo pipefail

mode="${1:-static}"
duration="${2:-60}"
output_dir="${3:-/tmp/imperative-m1-${mode}-$(date +%Y%m%d-%H%M%S)}"

case "${mode}" in
  static)
    move_obstacles=false
    random_seed=1234
    run_name="static-obstacles"
    ;;
  dynamic)
    move_obstacles=true
    random_seed=20260814
    run_name="dynamic-obstacles-seed-${random_seed}"
    ;;
  *)
    echo "Usage: $0 {static|dynamic} [duration_seconds] [output_dir]" >&2
    exit 2
    ;;
esac

mkdir -p "${output_dir}"
if [[ -n "$(find "${output_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "Output directory is not empty: ${output_dir}" >&2
  exit 1
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/rosmaster-m1-experiment-roslog}"
mkdir -p "${ROS_LOG_DIR}"

set +e
timeout --foreground --signal=INT "${duration}s" \
  ros2 launch imperative_navigation imperative_m1_gazebo.launch.py \
  gui:=false rviz:=false software_lidar:=true planner:=true record:=true \
  move_obstacles:="${move_obstacles}" random_seed:="${random_seed}" \
  run_name:="${run_name}" log_dir:="${output_dir}"
launch_status=$?
set -e
if [[ "${launch_status}" -ne 0 && "${launch_status}" -ne 124 && "${launch_status}" -ne 130 ]]; then
  echo "Gazebo launch failed with exit code ${launch_status}" >&2
  exit "${launch_status}"
fi

python3 "$(dirname "$0")/plot_experiment.py" "${output_dir}"
echo "Experiment complete: ${output_dir}"
