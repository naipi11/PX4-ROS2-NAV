#!/usr/bin/env bash
# Runs one simulation make target in the dev container between two cleanups:
# whatever a previous run left behind is stopped first, and whatever this run
# leaves behind (an interrupted launch, a container that outlived its
# simulator) is stopped once it ends, however it ends. It never waits for the
# host: the user's run starts at once.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$#" -eq 0 ]]; then
  echo "Usage: $0 make <sim-target> [...]" >&2
  exit 2
fi

"${repo_root}/scripts/prune_sim_logs.sh"
"${repo_root}/scripts/cleanup_sim_processes.sh"
# No wait for a quiet host here (the owner, 2026-10-30): a run the user starts
# starts at once, whatever the host is doing. The quiet-host gate of the
# acceptance series (A4) lives in tools/series2.sh, which the series tools call.

# PX4 SITL sends its GCS stream to 127.0.0.1:14550 and its offboard stream to
# 14540, so a process on this host that binds either one can command the vehicle
# over MAVLink. On 2026-11-12 a foreign pymavlink client did exactly that and
# destroyed four flights with a NAV_TAKEOFF before it was found
# (log/runs/nav_state17_root_cause_20261112.txt). A flight therefore refuses to
# start while anything holds those ports, and holds them itself for its
# duration; the navigation stack drives PX4 over uXRCE/DDS, so nothing of the
# stack wants them. SITL_MAVLINK_GUARD=false flies without this gate.
mavlink_guard_log="${repo_root}/log/mavlink_guard.log"
case "${SITL_MAVLINK_GUARD:-true}" in
  0 | false | no | off)
    echo "MAVLink guard: disabled by SITL_MAVLINK_GUARD"
    ;;
  *)
    if ! python3 "${repo_root}/scripts/sitl_mavlink_guard.py" --probe; then
      cat >&2 <<'REFUSAL'
Refusing to fly: a process on this host already holds a PX4 SITL MAVLink port,
so it can command the vehicle in flight. Name it with
`ss -lunp | grep -E ':14550|:14540'` and stop it, or set
SITL_MAVLINK_GUARD=false to fly without this gate.
REFUSAL
      exit 3
    fi
    mkdir -p "$(dirname "${mavlink_guard_log}")"
    # The guard frees the ports by itself when this script exits, however it
    # exits, so a flight that dies early leaves nothing holding them.
    python3 "${repo_root}/scripts/sitl_mavlink_guard.py" \
      --log "${mavlink_guard_log}" >/dev/null 2>&1 &
    mavlink_guard_pid=$!
    sleep 0.3
    if ! kill -0 "${mavlink_guard_pid}" 2>/dev/null; then
      echo "Refusing to fly: the MAVLink guard could not hold its ports" >&2
      exit 3
    fi
    ;;
esac

trap '"${repo_root}/scripts/cleanup_sim_processes.sh" || true' EXIT
"${repo_root}/scripts/container_run.sh" "$@"
