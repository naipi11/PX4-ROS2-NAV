"""What happens to a stop that no successor replaces.

A stop is published with a certified braking trajectory, and normally a rerouted
plan replaces it about a tenth of a second later (`stop_supersession.py`). This
asks the other question - the minority that no planned horizon follows within a
second - because those are the stops that have to do the whole job themselves.
For each it reports whether the stop reached `STOP_EXECUTION completed=true` (the
vehicle brought to rest on it and its execution revoked), whether a horizon
followed later, and whether the run ended in a contact.

Over the runs kept under `log/runs` on 2026-11-14 (88 of them; older runs are
pruned weekly, so re-running this over a different set gives different counts):
1665 stops, 14 with no successor within a second, and of those 14 nine rested on
the stop and thirteen had a successor within five seconds. The two runs among them
that ended in a contact are the flights whose autopilot was killed on purpose to
test the record, so the stop is not what destroyed those vehicles.

  python3 tools/stop_without_successor.py            # every run in log/runs
  python3 tools/stop_without_successor.py --window 3 20261114T130802Z-1898
"""

import argparse
import glob
import os
import re
import sys

STOP = re.compile(
    r"\[(\d{10}\.\d+)\].*STOP_EXECUTION published=true"
    r".*?trajectory_revision=(\d+).*?speed_mps=([\d.]+)")
COMPLETED = re.compile(r"\[(\d{10}\.\d+)\].*STOP_EXECUTION completed=true")
HORIZON = re.compile(
    r"\[(\d{10}\.\d+)\].*EXECUTION_HORIZON published=true"
    r".*?sequence=(\d+) mode=(\w+)")
RESULT = re.compile(r"MISSION_RESULT success=(\w+) reason='(\w+)'")


def read_run(path):
    """Returns (stops with their speed, stop-completed stamps, planned stamps, result)."""
    stops = []
    completed = []
    planned = []
    result = None
    stop_revisions = set()
    with open(path, errors="replace") as stream:
        for line in stream:
            match = STOP.search(line)
            if match:
                stops.append((float(match.group(1)), float(match.group(3))))
                stop_revisions.add(match.group(2))
                continue
            match = COMPLETED.search(line)
            if match:
                completed.append(float(match.group(1)))
                continue
            match = HORIZON.search(line)
            if match and match.group(3) == "planned" and match.group(2) not in stop_revisions:
                planned.append(float(match.group(1)))
                continue
            match = RESULT.search(line)
            if match:
                result = (match.group(1), match.group(2))
    return stops, completed, sorted(planned), result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", nargs="?", help="a run id or path under log/runs")
    parser.add_argument("--window", type=float, default=1.0,
                        help="seconds after a stop in which a successor counts")
    parser.add_argument("--reach", type=float, default=5.0,
                        help="seconds afterwards to look for a rest or a successor")
    arguments = parser.parse_args()

    runs = ([arguments.run if os.path.isdir(arguments.run)
             else os.path.join("log/runs", arguments.run)]
            if arguments.run else sorted(glob.glob("log/runs/2026*")))

    print(f"{'run':<26} {'stops':>5} {'alone':>6} {'rested':>6} {'later':>6} "
          f"{'contact':>7}  alone at m/s")
    totals = {"stops": 0, "alone": 0, "rested": 0, "later": 0, "contacts": 0}
    for run in runs:
        path = os.path.join(run, "ros_drone_nav.log")
        if not os.path.exists(path):
            continue
        stops, completed, planned, result = read_run(path)
        if not stops:
            continue
        alone = [(stamp, speed) for stamp, speed in stops
                 if not any(stamp <= t <= stamp + arguments.window for t in planned)]
        rested = sum(1 for stamp, _ in alone
                     if any(stamp <= t <= stamp + arguments.reach for t in completed))
        later = sum(1 for stamp, _ in alone
                    if any(stamp <= t <= stamp + arguments.reach for t in planned))
        contact = result is not None and result[1] == "vehicle_destroyed"
        totals["stops"] += len(stops)
        totals["alone"] += len(alone)
        totals["rested"] += rested
        totals["later"] += later
        totals["contacts"] += 1 if contact else 0
        speeds = " ".join(f"{speed:.2f}" for _stamp, speed in alone[:8])
        print(f"{os.path.basename(run):<26} {len(stops):>5} {len(alone):>6} {rested:>6} "
              f"{later:>6} {('yes' if contact else 'no'):>7}  {speeds}")
    print(f"\ntotal: stops={totals['stops']} alone={totals['alone']} "
          f"rested_on_the_stop={totals['rested']} "
          f"successor_within_reach={totals['later']} "
          f"runs_with_a_contact={totals['contacts']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
