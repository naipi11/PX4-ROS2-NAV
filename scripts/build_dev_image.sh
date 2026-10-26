#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# A deliberate development build does not replace docker/dev-image.lock.
# Opt in with DRONE_GAZEBO_DEV_IMAGE=drone-gazebo-dev:latest to use its result.

docker build \
  --file "${repo_root}/docker/Dockerfile" \
  --tag drone-gazebo-dev:latest \
  "${repo_root}"
