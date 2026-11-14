#!/usr/bin/env bash
# Keep foreign MAVLink clients off the flight's autopilot.
#
# PX4 SITL sends its GCS stream to 127.0.0.1:14550 and its offboard stream to
# 14540, so a process on the host that binds either one receives that stream and
# can command the vehicle over MAVLink. On 2026-11-12 a pymavlink client did
# exactly that and destroyed four flights with a NAV_TAKEOFF
# (log/runs/nav_state17_root_cause_20261112.txt, four crashes, three successes
# without it). The navigation stack drives PX4 over uXRCE/DDS, so holding these
# ports costs the flight nothing.
#
# The guard runs here, where the flight starts, and not in the sim_*.sh wrapper:
# tools/series2.sh and a plain `make sim-...` run the flight script directly, and
# those are the acceptance flights (2026-11-14). The container shares the host's
# network namespace (container_run.sh --network host), so the guard sees and holds
# the same ports the wrapper's copy did.
#
# Sourced by scripts/run_drone_nav_sim.sh; reads and sets its caller's variables.

# Probes the ports and then holds them for the flight. Refuses, with the flight's
# exit code 3, when a port is already held - which is another process able to
# command the vehicle in flight.
start_mavlink_guard() {
  mavlink_guard_log="${repo_root}/log/mavlink_guard.log"
  mavlink_guard_ports_args=()
  if [[ -n "${SITL_MAVLINK_GUARD_PORTS:-}" ]]; then
    mavlink_guard_ports_args=(--ports "${SITL_MAVLINK_GUARD_PORTS}")
  fi
  mavlink_guard_pid=""
  case "${SITL_MAVLINK_GUARD:-true}" in
    0 | false | no | off)
      echo "MAVLink guard: disabled by SITL_MAVLINK_GUARD"
      return 0
      ;;
  esac
  if ! python3 "${repo_root}/scripts/sitl_mavlink_guard.py" --probe \
    --log "${mavlink_guard_log}" "${mavlink_guard_ports_args[@]}"; then
    cat >&2 <<'REFUSAL'
Refusing to fly: a process on this host already holds a PX4 SITL MAVLink port,
so it can command the vehicle in flight. Name it with
`ss -lunp | grep -E ':14550|:14540'` and stop it, or set
SITL_MAVLINK_GUARD=false to fly without this gate.
REFUSAL
    exit 3
  fi
  mkdir -p "$(dirname "${mavlink_guard_log}")"
  # The guard frees the ports by itself when it exits, however it exits, so a
  # flight that dies early leaves nothing holding them; the flight stops it
  # explicitly as well.
  python3 "${repo_root}/scripts/sitl_mavlink_guard.py" \
    --log "${mavlink_guard_log}" "${mavlink_guard_ports_args[@]}" >/dev/null 2>&1 &
  mavlink_guard_pid=$!
  sleep 0.3
  if ! kill -0 "${mavlink_guard_pid}" 2>/dev/null; then
    echo "Refusing to fly: the MAVLink guard could not hold its ports" >&2
    exit 3
  fi
}

stop_mavlink_guard() {
  [[ -n "${mavlink_guard_pid:-}" ]] || return 0
  kill "${mavlink_guard_pid}" 2>/dev/null || true
  wait "${mavlink_guard_pid}" 2>/dev/null || true
  mavlink_guard_pid=""
}
