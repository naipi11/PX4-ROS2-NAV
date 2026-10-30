#!/usr/bin/env bash
set -euo pipefail
# build_replay.sh OUT: build inside the container; OUT is relative to log/tools/lio.
mkdir -p /workspace/log/tools/lio && cd /workspace/log/tools/lio
g++ -O2 -std=c++20 -I/workspace/drone_city_nav/src -I/workspace/drone_city_nav/include -I/usr/include/eigen3 \
  /workspace/tools/lio/replay_lio.cpp /workspace/drone_city_nav/src/lidar_inertial_odometry.cpp \
  /workspace/drone_city_nav/src/point_plane_map_3d.cpp -o "$1"
