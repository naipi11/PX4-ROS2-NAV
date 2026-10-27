#!/usr/bin/env bash
# series2.sh [--lidar] RUN [RUN...]: inspect each flight before starting the next.
# Exit 0: all passed; 1: valid flight failed; 2: host void; 3: setup/evidence error.
# INT/TERM preserve partial evidence and stop only this repository's simulation.
set -euo pipefail
# Unhandled setup failures are not a measured flight failure.
trap 'exit 3' ERR
cd "$(dirname "${BASH_SOURCE[0]}")/.."

profile=cameras
envs=()
if [[ "${1:-}" == "--lidar" ]]; then
  profile=lidar
  envs=(CAMERA_PROFILE=none NAVIGATION_SENSOR_PROFILE=lidar)
  shift
fi
if [[ "${1:-}" == "--help" ]]; then
  echo "Usage: $0 [--lidar] RUN [RUN...]"
  exit 0
fi
if [[ "$#" -eq 0 ]]; then
  echo "At least one unused run ID is required." >&2
  exit 3
fi
declare -A seen=()
for id in "$@"; do
  if [[ ! "$id" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] ||
    [[ -n "${seen[$id]+present}" ]] ||
    [[ -e "log/runs/$id" || -e "log/tools/run_$id.log" ]]; then
    echo "Refusing invalid, duplicate or existing run ID: $id" >&2
    exit 3
  fi
  seen["$id"]=1
done

commit="$(git rev-parse HEAD)"
assert_source() {
  local current dirty
  current="$(git rev-parse HEAD)" || return 3
  dirty="$(git status --porcelain=v1 --untracked-files=all)" || return 3
  if [[ "$current" != "$commit" || -n "$dirty" ]]; then
    echo "Acceptance requires the same clean commit throughout the series." >&2
    return 3
  fi
}
assert_source
mkdir -p log/tools/journal log/runs
# The lock is held across the quiet-host wait, flight, inspection and cleanup.
exec 9>log/tools/series.lock
if ! flock -n 9; then
  echo "Another acceptance series owns this repository's simulation." >&2
  exit 3
fi
./scripts/prune_sim_logs.sh
touch log/tools/journal/.keep
out="log/tools/journal/series_$(git rev-parse --short HEAD)_${profile}.txt"
flight_pid=""
current_run=""

finish() {
  local status="$1" cleanup_status=0
  trap - EXIT INT TERM
  if [[ -n "$flight_pid" ]]; then
    kill -TERM "$flight_pid" 2>/dev/null || true
    ./scripts/stop_sim.sh > "log/runs/$current_run/cleanup.log" 2>&1 || cleanup_status=$?
    wait "$flight_pid" 2>/dev/null || true
    python3 tools/acceptance_result.py "$current_run" --commit "$commit" \
      --profile "$profile" --command-exit "$status" --cleanup-exit "$cleanup_status" \
      >> "$out" 2>&1 || true
  fi
  exit "$status"
}
trap 'finish "$?"' EXIT
trap 'finish 130' INT
trap 'finish 143' TERM

for id in "$@"; do
  assert_source
  # Stop stale owned simulations before waiting; otherwise an old GUI flight
  # would keep the quiet-host gate closed forever.
  if ! ./scripts/stop_sim.sh; then
    echo "Simulation cleanup failed before $id." >&2
    exit 3
  fi
  echo "SERIES waiting for a quiet host: run=$id commit=$commit profile=$profile"
  quiet=0
  while [[ "$quiet" -lt 60 ]]; do
    busy="$(ps -eo pcpu,comm | grep -E 'rustc|clippy|cargo' | awk '$1>5{c++} END{print c+0}' || true)"
    load="$(cut -d' ' -f1 /proc/loadavg)"
    sim="$(pgrep -c -f '[g]z sim' || true)"
    if [[ "$busy" -eq 0 && "${load%.*}" -lt 3 && "$sim" -eq 0 ]]; then
      quiet=$((quiet + 10))
    else
      quiet=0
    fi
    sleep 10
  done
  assert_source
  current_run="$id"
  run_directory="log/runs/$id"
  mkdir "$run_directory"
  printf 'Acceptance flight on %s (%s).\n' "$commit" "$profile" > "$run_directory/.keep"
  # Preserve the familiar live log path, but keep the same bytes under .keep.
  : > "$run_directory/acceptance.log"
  ln "$run_directory/acceptance.log" "log/tools/run_$id.log"
  echo "== $id start $(date -u +%H:%M:%S) commit=$commit profile=$profile target=${TARGET:-sim-urban-point-to-point-headless}" | tee -a "$out"
  env "${envs[@]}" DRONE_GAZEBO_RUN_ID="$id" SMOKE_DURATION_S="${SMOKE_DURATION_S:-600}" \
    ./scripts/dev_shell.sh make "${TARGET:-sim-urban-point-to-point-headless}" \
    > "log/tools/run_$id.log" 2>&1 &
  flight_pid=$!
  command_status=0
  wait "$flight_pid" || command_status=$?
  cleanup_status=0
  ./scripts/stop_sim.sh > "$run_directory/cleanup.log" 2>&1 || cleanup_status=$?
  flight_pid=""
  grep -E '^FAIL|^(OK|NOTE): (mean flight|crash|no crash|post-bootstrap|persistent planner p95|mission monitor|execution ownership|the true position|real-time factor|the goal was given up|the vehicle returned|the vehicle is whole|production tick wall time|no vehicle collided)' \
    "$run_directory/acceptance.log" | tee -a "$out" || true
  result_status=0
  python3 tools/acceptance_result.py "$id" --commit "$commit" --profile "$profile" \
    --command-exit "$command_status" --cleanup-exit "$cleanup_status" \
    | tee -a "$out" || result_status=$?
  echo "== $id end $(date -u +%H:%M:%S) exit=$result_status" >> "$out"
  if [[ "$result_status" -ne 0 ]]; then
    echo "SERIES STOPPED at $id; inspect $run_directory/acceptance.json before another flight." >&2
    exit "$result_status"
  fi
done
echo "SERIES PASSED $(date -u +%H:%M:%S) commit=$commit profile=$profile count=$#" | tee -a "$out"
