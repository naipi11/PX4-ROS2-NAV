#!/usr/bin/env bash
# mxw.sh RUN: wait up to 9.5 min; DONE/REFUSED propagates the matrix exit code.
cd "$(dirname "${BASH_SOURCE[0]}")/.."
for i in $(seq 1 57); do
  if [[ -f log/tools/mx_flag.txt ]] &&
    read -r state line run result < log/tools/mx_flag.txt &&
    [[ "$run" == "$1" && ( "$state" == DONE || "$state" == REFUSED ) ]] &&
    [[ "$result" =~ ^exit=([0-9]+)$ ]]; then
    status="${BASH_REMATCH[1]}"
    echo "$state exit=$status"
    exit "$status"
  fi
  sleep 10
done
echo RUNNING
