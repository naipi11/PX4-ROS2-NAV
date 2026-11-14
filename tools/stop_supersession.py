"""How long a published stop survives, and what put the stack into it.

A stop is the stack's answer to a route it can no longer justify, and it brakes
along a trajectory certified against the world. Whether it gets to brake depends
on what the next publication is, so this reads two things out of a run's
`ros_drone_nav.log`:

- the supersession: for every `STOP_EXECUTION published=true`, the first
  `EXECUTION_HORIZON published=true mode=planned` after it, with the gap and the
  speed the stop was certified at. A stop is itself published as a horizon with
  `mode=planned` under its own `trajectory_revision`, so those publications are
  excluded: counting them against the stop would pair every stop with itself.
- the trigger: the `EXECUTION_HORIZON_COMMIT committed=false stage=...` nearest
  before each stop. A stop is the moving vehicle's answer to a commit it could
  not make, and the stage names what it could not make it against.

  python3 tools/stop_supersession.py 20261113T155419Z-1573
  python3 tools/stop_supersession.py --window 0.5 <run>
"""

import argparse
import glob
import os
import re
import sys

STOP = re.compile(
    r"\[(\d{10}\.\d+)\].*STOP_EXECUTION published=true"
    r".*?trajectory_revision=(\d+).*?speed_mps=([\d.]+)")
HORIZON = re.compile(
    r"\[(\d{10}\.\d+)\].*EXECUTION_HORIZON published=true"
    r".*?sequence=(\d+) mode=(\w+)")
REFUSAL = re.compile(
    r"\[(\d{10}\.\d+)\].*EXECUTION_HORIZON_COMMIT committed=false stage=(\w+)")


def read_run(run_directory):
    """Returns (stops with their speed, planned horizon stamps, commit refusals)."""
    path = os.path.join(run_directory, "ros_drone_nav.log")
    if not os.path.exists(path):
        return [], [], []
    stops = []
    planned = []
    refusals = []
    stop_revisions = set()
    with open(path, errors="replace") as stream:
        for line in stream:
            match = STOP.search(line)
            if match:
                stops.append((float(match.group(1)), float(match.group(3))))
                stop_revisions.add(match.group(2))
                continue
            match = REFUSAL.search(line)
            if match:
                refusals.append((float(match.group(1)), match.group(2)))
                continue
            match = HORIZON.search(line)
            if (match and match.group(3) == "planned"
                    and match.group(2) not in stop_revisions):
                planned.append(float(match.group(1)))
    planned.sort()
    return stops, planned, refusals


def supersessions(stops, planned, window):
    """Pairs every stop with the first planned publication inside the window."""
    replaced = []
    for stamp, speed in stops:
        after = [t for t in planned if stamp <= t <= stamp + window]
        if after:
            replaced.append((after[0] - stamp, speed))
    return replaced


def stop_trigger(refusals, stamp, window=0.10):
    """The stage of the commit refusal nearest before a stop, if one is close.

    A stop is the moving vehicle's answer to a commit it could not make, and the
    refusal is logged on the same tick with the stage it failed at.
    """
    before = [stage for when, stage in refusals if stamp - window <= when <= stamp]
    return before[-1] if before else "none_in_window"


def describe(run, window):
    stops, planned, refusals = read_run(run)
    if not stops:
        return None
    replaced = supersessions(stops, planned, window)
    triggers = {}
    for stamp, _speed in stops:
        stage = stop_trigger(refusals, stamp)
        triggers[stage] = triggers.get(stage, 0) + 1
    return stops, replaced, triggers


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
          f"{'fastest':>8}  speeds (gap)")
    for run in runs:
        if not os.path.isdir(run):
            continue
        described = describe(run, arguments.window)
        if described is None:
            continue
        stops, replaced, triggers = described
        causes = " ".join(f"{stage}:{count}" for stage, count in
                          sorted(triggers.items(), key=lambda item: -item[1]))
        if not replaced:
            print(f"{os.path.basename(run):<26} {len(stops):>5} {0:>8} "
                  f"{'-':>7} {'-':>8}")
        else:
            gaps = sorted(delta for delta, _ in replaced)
            median = gaps[len(gaps) // 2]
            speeds = " ".join(f"{speed:.2f}@{delta * 1000:.0f}ms"
                              for delta, speed in replaced[:5])
            print(f"{os.path.basename(run):<26} {len(stops):>5} {len(replaced):>8} "
                  f"{median * 1000:>6.0f}ms {gaps[0] * 1000:>7.0f}ms  {speeds}")
        print(f"{'':<26}   triggers: {causes}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
