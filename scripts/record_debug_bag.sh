#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output_dir="${BAG_OUTPUT_DIR:-${DRONE_GAZEBO_LOG_DIR:-${repo_root}/log}/rosbag_city_debug}"

mkdir -p "$(dirname "${output_dir}")"

ros2 bag record \
  --output "${output_dir}" \
  /drone_city_nav/remembered_lidar_points \
  /drone_city_nav/raw_memory_obstacle_points \
  /drone_city_nav/raw_memory_obstacle_points_3d \
  /drone_city_nav/raw_memory_darkness_points_3d \
  /drone_city_nav/static_building_markers \
  /drone_city_nav/obstacle_memory_grid \
  /drone_city_nav/obstacle_memory_provenance \
  /drone_city_nav/obstacle_memory_snapshot \
  /drone_city_nav/raw_obstacle_snapshot \
  /drone_city_nav/raw_obstacle_snapshot_3d \
  /drone_city_nav/raw_obstacle_delta_3d \
  /drone_city_nav/latest_sensor_obstacle_scan \
  /lidar_3d/points \
  /drone_city_nav/mppi/execution_horizon \
  /drone_city_nav/mppi/status \
  /drone_city_nav/mppi/path \
  /drone_city_nav/mppi/markers \
  /drone_city_nav/mppi/applied_control \
  /clock \
  /tf \
  /tf_static \
  /fmu/out/vehicle_local_position_v1 \
  /fmu/out/vehicle_attitude \
  /fmu/out/sensor_combined \
  /fmu/out/timesync_status \
  /fmu/out/vehicle_status_v1
