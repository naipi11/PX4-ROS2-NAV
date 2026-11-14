#!/usr/bin/env bash
# One flight per checkout at a time.
#
# The MAVLink guard covers PX4's ports, but the model staging directory, the
# Gazebo world and the flight's own PX4 are per checkout: two flights from one
# checkout share them, and one of two concurrent flights on 2026-11-14 failed with
# `cp: cannot create directory .../models/lidar_3d_v1: No such file or directory`
# while the other was flying. Both also run stop_sim.sh, which stops this
# repository's simulation containers, so two sessions on one checkout stop each
# other's flight - and a flight stopped that way is an autopilot that leaves with
# no message.
#
# The lock is a directory holding its owner, and a lock whose owner is gone is
# taken over: a flight killed with SIGKILL cannot remove its own.
#
# Sourced by scripts/run_drone_nav_sim.sh.

acquire_flight_lock() {
  flight_lock_dir="${repo_root}/log/.flight-lock"
  flight_lock_owner_is_alive() {
    local owner_pid
    owner_pid="$(sed -n 's/^pid=\([0-9][0-9]*\).*/\1/p' \
      "${flight_lock_dir}/owner" 2>/dev/null | head -1)"
    [[ -n "${owner_pid}" ]] || return 1
    local state
    state="$(awk '{print $3}' "/proc/${owner_pid}/stat" 2>/dev/null)" || return 1
    [[ -n "${state}" && "${state}" != "Z" ]]
  }
  if ! mkdir "${flight_lock_dir}" 2>/dev/null; then
    if flight_lock_owner_is_alive; then
      echo "Refusing to fly: another flight is running from this checkout." >&2
      sed 's/^/  /' "${flight_lock_dir}/owner" >&2 2>/dev/null || true
      echo "Wait for it to finish, or remove ${flight_lock_dir} if its owner is gone." >&2
      exit 3
    fi
    echo "Taking over a flight lock whose owner is gone:" >&2
    sed 's/^/  /' "${flight_lock_dir}/owner" >&2 2>/dev/null || true
    rm -rf "${flight_lock_dir}"
    if ! mkdir "${flight_lock_dir}" 2>/dev/null; then
      echo "Refusing to fly: ${flight_lock_dir} could not be taken." >&2
      exit 3
    fi
  fi
  printf 'pid=%s host=%s started=%s run_id=%s\n' "$$" "$(hostname)" \
    "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "${run_id}" > "${flight_lock_dir}/owner"
}

release_flight_lock() {
  [[ -n "${flight_lock_dir:-}" ]] || return 0
  rm -rf "${flight_lock_dir}"
}
