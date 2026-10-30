#!/usr/bin/env bash
set -euo pipefail
# build_replay_src.sh TRACKER_CPP OUT: inside the container, builds replay_vio against the repository's filter and the
# given tracker source (the repository's or a variant); the binary OUT lands in log/tools/vio.
mkdir -p /workspace/log/tools/vio && cd /workspace/log/tools/vio
g++ -O2 -std=c++20 -I/workspace/drone_city_nav/src -I/workspace/drone_city_nav/include -I/usr/include/eigen3 $(pkg-config --cflags opencv4) /workspace/tools/vio/replay_vio.cpp /workspace/drone_city_nav/src/visual_inertial_odometry.cpp "$1" -o "$2" \
  -lopencv_core -lopencv_imgproc -lopencv_video -lopencv_calib3d
