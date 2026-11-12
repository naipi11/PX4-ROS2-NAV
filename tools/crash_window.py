#!/usr/bin/env python3
"""Extract the 8 s around a flight's contact from its own bag and records.

  crash_window.py RUN_ID [SECONDS_BEFORE] [SECONDS_AFTER]

Prints, on the simulation clock, what the stack commanded, what the airframe
did, and what the autopilot's mode was, second by second, so a pose change can
be attributed to a commanded motion, a lost mode, or something else.

Reads only the run's own files: stack_bag (mcap), gz_pose.csv, ros_drone_nav.log,
pilot.log. Diagnostic; no stack dependency beyond px4_msgs messages in the bag.
"""
from __future__ import annotations

import bisect
import csv
import re
import sys
from pathlib import Path

import rosbag2_py
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus
from rclpy.serialization import deserialize_message

def _resolve(argument: str) -> Path:
    """A run id (r1234), a directory in log/runs, or an explicit path."""
    candidate = Path(argument)
    if candidate.is_dir():
        return candidate
    for root in (Path("log/runs"), Path("/runs")):
        named = root / argument
        if named.is_dir():
            return named
        matches = sorted(root.glob(f"{argument}*")) if root.is_dir() else []
        if matches:
            return matches[-1]
    raise SystemExit(f"no run directory for {argument}")


RUN = _resolve(sys.argv[1])
BEFORE = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
AFTER = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0

# PX4 nav states we care to name.
NAV = {0: "MANUAL", 4: "LOITER", 14: "OFFBOARD", 17: "AUTO_TAKEOFF",
       18: "AUTO_LAND", 19: "AUTO_LOITER?19", 12: "DESCEND", 13: "TERMINATION"}


def contact_sim_time() -> float | None:
    """Simulation clock at the first VEHICLE_DESTROYED, from gz_pose's epoch."""
    log = (RUN / "ros_drone_nav.log").read_text(errors="replace")
    match = re.search(r"\[(\d+\.\d+)\] \[mppi_offboard_node\][^\n]*VEHICLE_DESTROYED latched",
                      log)
    if not match:
        return None
    epoch = float(match.group(1))
    # gz_pose carries sim time (col 0) and wall epoch (col 8): interpolate.
    points = []
    for row in csv.reader(open(RUN / "gz_pose.csv")):
        try:
            points.append((float(row[8]), float(row[0])))
        except (ValueError, IndexError):
            continue
    if not points:
        return None
    epochs = [p[0] for p in points]
    index = bisect.bisect_left(epochs, epoch)
    index = min(max(index, 0), len(points) - 1)
    return points[index][1]


def read_bag(lo_ns: int, hi_ns: int):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(RUN / "stack_bag"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    topics = ("/fmu/in/trajectory_setpoint", "/fmu/out/vehicle_status_v1")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    setpoints, statuses = [], []
    while reader.has_next():
        topic, payload, stamp = reader.read_next()
        if stamp < lo_ns or stamp > hi_ns:
            continue
        if topic.endswith("trajectory_setpoint"):
            setpoints.append((stamp, deserialize_message(payload, TrajectorySetpoint)))
        else:
            statuses.append((stamp, deserialize_message(payload, VehicleStatus)))
    return setpoints, statuses


def truth_rows(lo: float, hi: float):
    rows = []
    for row in csv.reader(open(RUN / "gz_pose.csv")):
        try:
            values = [float(v) for v in row[:4]]
        except ValueError:
            continue
        if lo <= values[0] <= hi:
            rows.append(values)
    return rows


def main() -> int:
    contact = contact_sim_time()
    if contact is None:
        print("no VEHICLE_DESTROYED in the log; nothing to window")
        return 1
    lo, hi = contact - BEFORE, contact + AFTER
    print(f"run {sys.argv[1]}  contact at sim {contact:.2f} s  window {lo:.2f}..{hi:.2f}")
    rows = truth_rows(lo, hi)
    if not rows:
        print("no truth rows in the window")
    print("\n sim    truth x      y      z")
    for t, x, y, z in rows[:: max(1, len(rows) // 40)]:
        print(f"{t:7.2f} {x:8.2f} {y:6.2f} {z:6.2f}")

    # The bag stores receive stamps on the simulation clock, in ns.
    if not (RUN / "stack_bag").is_dir():
        print("\nno stack_bag in this run: the commanded motion for the window is not"
              " recoverable (only the truth and the logs are)")
        pilot = (RUN / "pilot.log").read_text(errors="replace").strip().splitlines()
        print("pilot tail:")
        for line in pilot[-4:]:
            print("  " + line)
        return 0
    setpoints, statuses = read_bag(int(lo * 1e9), int(hi * 1e9))
    print(f"\nsetpoints in window: {len(setpoints)}   statuses: {len(statuses)}")
    previous_mode = None
    statuses_iter = iter(statuses)
    next_status = next(statuses_iter, None)
    last_kind = None
    for index, (stamp, message) in enumerate(setpoints):
        while next_status is not None and next_status[0] <= stamp:
            state = next_status[1]
            mode = NAV.get(state.nav_state, str(state.nav_state))
            if mode != previous_mode:
                print(f"\n  {stamp / 1e9:7.2f} MODE {mode} armed={state.arming_state} "
                      f"failsafe={state.failsafe} user_intention="
                      f"{NAV.get(state.nav_state_user_intention, state.nav_state_user_intention)}")
                previous_mode = mode
            next_status = next(statuses_iter, None)
        velocity = [float(v) for v in message.velocity]
        acceleration = [float(v) for v in message.acceleration]
        finite_v = [v for v in velocity if v == v]
        finite_a = [v for v in acceleration if v == v]
        speed_cmd = (sum(v * v for v in finite_v) ** 0.5) if len(finite_v) == 3 else None
        accel_cmd = (sum(v * v for v in finite_a) ** 0.5) if len(finite_a) == 3 else None
        kind = "hold" if (speed_cmd is not None and speed_cmd < 1e-6
                          and accel_cmd is not None and accel_cmd > 0.05) else "planned"
        if kind != last_kind or index % 40 == 0:
            print(f"  {stamp / 1e9:7.2f} {kind:7s} |v_cmd|="
                  f"{'-' if speed_cmd is None else f'{speed_cmd:.2f}'} |a_cmd|="
                  f"{'-' if accel_cmd is None else f'{accel_cmd:.2f}'} "
                  f"pos=({message.position[0]:.2f},{message.position[1]:.2f},"
                  f"{message.position[2]:.2f})")
            last_kind = kind
    pilot = (RUN / "pilot.log").read_text(errors="replace").strip().splitlines()
    print("\npilot tail:")
    for line in pilot[-4:]:
        print("  " + line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
