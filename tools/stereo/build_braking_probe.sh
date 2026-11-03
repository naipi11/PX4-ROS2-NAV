#!/usr/bin/env bash
set -euo pipefail
# Build inside the project container; output remains under retained tool data.
mkdir -p /workspace/log/tools/stereo && cd /workspace/log/tools/stereo
g++ -O2 -std=c++20 -I/workspace/drone_city_nav/include \
  /workspace/tools/stereo/braking_probe.cpp \
  /workspace/drone_city_nav/src/sensor_braking_contract_3d.cpp \
  /workspace/drone_city_nav/src/stopping_distance.cpp -o braking_probe
