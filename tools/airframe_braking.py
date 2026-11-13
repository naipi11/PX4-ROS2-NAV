"""The deceleration the airframe actually reaches, from a flight's true pose.

The braking laws budget a stop at the horizontal acceleration the controller is
allowed to command (4 m/s^2 on the urban set). What the airframe achieves is
another number, and it is the one a stopping distance is made of: v^2 / 2a. This
reads the flight's `gz_pose.csv` - the simulator's true pose, no estimator in
between - finds the episodes in which the vehicle sheds speed, and reports the
deceleration those episodes reached and the distance they took.

It measures the simulated x500 in the urban world. It does not measure a target
aircraft: the real vehicle's figured braking distance stays unestablished until
one is flown (docs/hardware_builds.md).

  python3 tools/airframe_braking.py 20261113T183559Z-1573
  python3 tools/airframe_braking.py --all-today
"""

import argparse
import glob
import math
import os
import statistics
import sys

# The pose is sampled at 50 Hz; a speed from neighbouring samples is noise, so
# speed is measured across a window and the deceleration across two of them.
SPEED_WINDOW_S = 0.10
EPISODE_STEP_S = 0.20
# A braking episode: the vehicle is moving, then it sheds at least a third of its
# speed within the window below, without regaining it.
MIN_ENTRY_SPEED_MPS = 1.5
# A body that loses speed faster than this is not braking: an impact moves the
# pose without a force the airframe could apply, and reading those as braking
# put 46 m/s^2 among the episodes of a flight whose fastest entry was 6.4 m/s.
MAX_PLAUSIBLE_DECEL_MPS2 = 8.0
MAX_PLAUSIBLE_SPEED_MPS = 12.0
MIN_COVERED_DISTANCE_SHARE = 0.6
MIN_SPEED_LOSS_RATIO = 0.33
MAX_EPISODE_S = 3.0


def read_truth(run_directory):
    path = os.path.join(run_directory, "gz_pose.csv")
    if not os.path.exists(path):
        return []
    samples = []
    with open(path, errors="replace") as stream:
        for line in stream:
            parts = line.strip().split(",")
            if len(parts) < 4:
                continue
            try:
                samples.append((float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])))
            except ValueError:
                continue
    samples.sort()
    return samples


def speed_at(samples, index, window):
    """Speed across the window centred on a sample, in m/s."""
    lo = max(0, index - window)
    hi = min(len(samples) - 1, index + window)
    if hi <= lo:
        return None
    dt = samples[hi][0] - samples[lo][0]
    if dt <= 0:
        return None
    # Horizontal only: the braking contract the stopping distance is made of is the
    # horizontal acceleration, and a climb or a descent is not part of it.
    distance = math.hypot(samples[hi][1] - samples[lo][1], samples[hi][2] - samples[lo][2])
    return distance / dt


def braking_episodes(samples):
    if len(samples) < 10:
        return []
    window = max(1, int(round(SPEED_WINDOW_S / (samples[1][0] - samples[0][0] or 1.0))))
    step = max(1, int(round(EPISODE_STEP_S / (samples[1][0] - samples[0][0] or 1.0))))
    episodes = []
    index = window
    while index < len(samples) - window - step:
        entry = speed_at(samples, index, window)
        if entry is None or entry < MIN_ENTRY_SPEED_MPS:
            index += step
            continue
        best = None
        probe = index + step
        while probe < len(samples) - window and samples[probe][0] - samples[index][0] <= MAX_EPISODE_S:
            exit_speed = speed_at(samples, probe, window)
            if exit_speed is not None and exit_speed <= entry * (1.0 - MIN_SPEED_LOSS_RATIO):
                elapsed = samples[probe][0] - samples[index][0]
                distance = math.hypot(
                    samples[probe][1] - samples[index][1],
                    samples[probe][2] - samples[index][2],
                )
                decel = (entry - exit_speed) / elapsed if elapsed > 0 else 0.0
                # A deceleration covers the distance its two speeds average to; a
                # pose that jumps at an impact sheds speed without covering it, and
                # that is not the airframe braking.
                expected_metres = 0.5 * (entry + exit_speed) * elapsed
                plausible = (
                    elapsed > 0
                    and entry <= MAX_PLAUSIBLE_SPEED_MPS
                    and decel <= MAX_PLAUSIBLE_DECEL_MPS2
                    and (expected_metres <= 0.0
                         or distance >= MIN_COVERED_DISTANCE_SHARE * expected_metres)
                )
                if plausible:
                    best = {
                        "entry_mps": entry,
                        "exit_mps": exit_speed,
                        "seconds": elapsed,
                        "metres": distance,
                        "decel_mps2": (entry - exit_speed) / elapsed,
                    }
                    break
            probe += step
        if best is not None:
            episodes.append(best)
            index = probe + step
        else:
            index += step
    return episodes


def report(runs):
    print(f"{'run':<26} {'episodes':>8} {'max decel':>10} {'median':>8} "
          f"{'worst entry':>12} {'that stop':>10}")
    for run in runs:
        samples = read_truth(run)
        episodes = braking_episodes(samples)
        if not episodes:
            print(f"{os.path.basename(run):<26} {0:>8} {'-':>10} {'-':>8} {'-':>12} {'-':>10}")
            continue
        decels = [episode["decel_mps2"] for episode in episodes]
        worst = max(episodes, key=lambda episode: episode["entry_mps"])
        print(f"{os.path.basename(run):<26} {len(episodes):>8} {max(decels):>9.2f} "
              f"{statistics.median(decels):>7.2f} {worst['entry_mps']:>11.2f} "
              f"{worst['metres']:>9.2f}m")
        print(f"{'':<26}   from {worst['entry_mps']:.2f} to {worst['exit_mps']:.2f} m/s "
              f"in {worst['seconds']:.2f} s at {worst['decel_mps2']:.2f} m/s^2")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", nargs="?", help="a run id or a path under log/runs")
    parser.add_argument("--all-today", action="store_true",
                        help="every run under log/runs")
    arguments = parser.parse_args()
    if arguments.all_today:
        runs = sorted(glob.glob("log/runs/2026*"))
    elif arguments.run:
        runs = [arguments.run if os.path.isdir(arguments.run)
                else os.path.join("log/runs", arguments.run)]
    else:
        parser.error("name a run or pass --all-today")
    return report(runs)


if __name__ == "__main__":
    sys.exit(main())
