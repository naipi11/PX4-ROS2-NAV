"""How long a published stop survives before a planned horizon replaces it.

A stop is the stack's answer to a route it can no longer justify, and it brakes
along the motion it was certified for. Whether it gets to brake depends on the
next publication: a planned horizon of the same vehicle replaces it, and
`STOP_EXECUTION published=true` carries the speed at which the stop was
certified, so the two lines together say whether the stop had time to matter.

Reads only the run's `ros_drone_nav.log`. On the 2026-11-13 flights every
published stop was replaced within a second, most within a tenth of it, at 1.3
to 2.2 m/s - a stop that begins braking and is overwritten 20 to 160 ms later.

  python3 tools/stop_supersession.py                 # every run in log/runs
  python3 tools/stop_supersession.py 20261113T155419Z-1573 --window 0.5
"""

import argparse
import glob
import os
import re
import sys

STOP = re.compile(
    r"\[(\d{10}\.\d+)\].*STOP_EXECUTION published=true.*?speed_mps=([\d.]+)")
HORIZON = re.compile(
    r"\[(\d{10}\.\d+)\].*EXECUTION_HORIZON published=true.*?mode=(\w+)")


def read_run(run_directory):
    """Returns (stop stamps with their certified speed, planned stamps)."""
    path = os.path.join(run_directory, "ros_drone_nav.log")
    if not os.path.exists(path):
        return [], []
    stops = []
    planned = []
    with open(path, errors="replace") as stream:
        for line in stream:
            match = STOP.search(line)
            if match:
                stops.append((float(match.group(1)), float(match.group(2))))
                continue
            match = HORIZON.search(line)
            if match and match.group(2) == "planned":
                planned.append(float(match.group(1)))
    planned.sort()
    return stops, planned


def supersessions(stops, planned, window):
    """Pairs every stop with the first planned publication inside the window."""
    replaced = []
    for stamp, speed in stops:
        after = [t for t in planned if stamp <= t <= stamp + window]
        if after:
            replaced.append((after[0] - stamp, speed))
    return replaced


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", nargs="?", help="a run id or path under log/runs")
    parser.add_argument("--window", type=float, default=1.0,
                        help="seconds after a stop to look for its replacement")
    arguments = parser.parse_args()

    if arguments.run:
        runs = [arguments.run if os.path.isdir(arguments.run)
                else os.path.join("log/runs", arguments.run)]
    else:
        runs = sorted(glob.glob("log/runs/2026*"))

    print(f"{'run':<26} {'stops':>5} {'replaced':>8} {'median':>7} "
          f"{'fastest':>8}  speeds (replacement gap)")
    for run in runs:
        if not os.path.isdir(run):
            continue
        stops, planned = read_run(run)
        if not stops:
            continue
        replaced = supersessions(stops, planned, arguments.window)
        if not replaced:
            print(f"{os.path.basename(run):<26} {len(stops):>5} {0:>8} "
                  f"{'-':>7} {'-':>8}")
            continue
        gaps = sorted(delta for delta, _ in replaced)
        median = gaps[len(gaps) // 2]
        speeds = " ".join(f"{speed:.2f}@{delta * 1000:.0f}ms"
                          for delta, speed in replaced[:6])
        print(f"{os.path.basename(run):<26} {len(stops):>5} {len(replaced):>8} "
              f"{median * 1000:>6.0f}ms {gaps[0] * 1000:>7.0f}ms  {speeds}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
