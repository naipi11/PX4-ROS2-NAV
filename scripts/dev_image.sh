#!/usr/bin/env bash

# Print the local immutable image ID. Optional remaining arguments prefix Docker
# (cleanup uses timeout); this helper never pulls, builds or updates the lock.
resolve_dev_image() {
  local repository="$1"
  shift
  local reference="${DRONE_GAZEBO_DEV_IMAGE:-}"
  local image_id
  if [[ -z "${reference}" ]]; then
    if [[ ! -r "${repository}/docker/dev-image.lock" ]]; then
      echo "Missing docker/dev-image.lock; restore the baseline source or set DRONE_GAZEBO_DEV_IMAGE explicitly." >&2
      return 2
    fi
    reference="$(<"${repository}/docker/dev-image.lock")"
    if [[ ! "${reference}" =~ ^sha256:[0-9a-f]{64}$ ]]; then
      echo "docker/dev-image.lock must contain one immutable sha256 image ID." >&2
      return 2
    fi
  fi
  if ! image_id="$("$@" docker image inspect --format '{{.Id}}' "${reference}" 2>/dev/null)"; then
    echo "Dev image ${reference} is not available locally. Restore the archived image with docker load, or set DRONE_GAZEBO_DEV_IMAGE to an explicit local non-baseline image. No image will be rebuilt automatically." >&2
    return 2
  fi
  if [[ ! "${image_id}" =~ ^sha256:[0-9a-f]{64}$ ]]; then
    echo "Docker did not return an immutable image ID for ${reference}." >&2
    return 2
  fi
  printf '%s\n' "${image_id}"
}
