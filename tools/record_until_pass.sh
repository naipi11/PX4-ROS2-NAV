#!/usr/bin/env bash
# record_until_pass.sh SCENARIO FIRST_RUN ATTEMPTS DIRECTORY: records SCENARIO (tools/record_flight.sh) until one flight
# passes its check with a recording that is one, at most ATTEMPTS times, runs numbered from FIRST_RUN (an integer).
# Only a host-void flight or a bad recording is retried. A valid mission failure,
# missing evidence or an interrupted/stalled flight stops the sequence.
# The watchdog still stops host-slowed flights (specification A4, A7).
# One line per flight in log/tools/record_until_pass.txt, then END.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
scenario=$1; first=$2; attempts=$3; directory=$4
mkdir -p log/tools "$directory" || exit 3
flag=log/tools/record_until_pass.txt; rm -f "$flag"
for number in $(seq "$first" $((first + attempts - 1))); do
  run="r$number"; log="log/runs/$run/ros_drone_nav.log"; stalled=""
  ./tools/record_flight.sh "$scenario" "$run" "$directory" > /dev/null 2>&1 &
  flight=$!
  while kill -0 "$flight" 2> /dev/null; do
    sleep 20
    [ -f "$log" ] || continue
    if [ -f "log/runs/$run/resources.csv" ] &&
      ! verdict=$(python3 tools/rtf_watchdog.py "$run" --factor "${REAL_TIME_FACTOR:-0.6}"); then
      stalled="WATCHDOG ${verdict#WATCHDOG slow $run }"
    fi
    rest=$(grep "mission_monitor_node\]: Mission summary" "$log" | tail -12 |
      awk '{for (i = 1; i <= NF; i++) if ($i ~ /^speed=/) s = substr($i, 7); if (s + 0 < 0.05) c++} END {print c + 0}')
    if [ "$rest" -ge 12 ] && [ "$(grep -c "moved=true" "$log")" -gt 20 ]; then
      stalled=STALLED
    fi
    if [ -n "$stalled" ]; then
      # The owned wrapper forwards TERM to its series and recorder only.
      kill -TERM "$flight" 2>/dev/null || true
      break
    fi
  done
  flight_status=0
  wait "$flight" 2>/dev/null || flight_status=$?
  if [[ "$stalled" == WATCHDOG* ]]; then
    echo "$run $stalled" >> "$flag"
    continue
  fi
  result_status=0
  python3 tools/record_index.py "$scenario" "$run" "$directory" > /dev/null || result_status=$?
  if [[ "$result_status" -eq 0 && "$flight_status" -ne 0 ]]; then result_status=3; fi
  echo "$run RESULT exit=$result_status ${stalled:+$stalled }record=log/runs/$run/acceptance.json" >> "$flag"
  if [[ "$result_status" -ne 2 ]]; then
    echo END >> "$flag"
    exit "$result_status"
  fi
done
echo END >> "$flag"
exit 2
