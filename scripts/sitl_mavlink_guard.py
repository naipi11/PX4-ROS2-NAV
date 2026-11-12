#!/usr/bin/env python3
"""Keep foreign MAVLink clients off the PX4 SITL links, and report any that try.

PX4 SITL's GCS link listens on 18570 and sends its stream to 127.0.0.1:14550;
the offboard link listens on 14580 and sends to 14540
(ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink). A process on the host that
binds either remote port receives that stream and can command the vehicle over
it. On 2026-11-12 exactly that happened: a pymavlink client on this host sent
DO_SET_MODE, ARM_DISARM and NAV_TAKEOFF into a running flight, which put PX4
into AUTO_TAKEOFF and destroyed the vehicle
(log/runs/nav_state17_root_cause_20261112.txt, four crashes, three successes
without it).

The navigation stack does not use MAVLink - it drives PX4 over uXRCE/DDS - so
holding these ports costs the flight nothing while it denies them to anyone
else. The guard never sends anything, so PX4 does not see a ground station and
behaves exactly as it does with nobody listening.

  sitl_mavlink_guard.py --probe             # exit 0 when the ports are free,
                                            # 1 when another process holds one
  sitl_mavlink_guard.py --log PATH [--seconds N]

`--probe` binds each port without SO_REUSEADDR and releases it, so its verdict is
exactly "could this run take exclusive hold of the port". SO_REUSEADDR is not
used anywhere here: with it, a second bind succeeds even while another process
holds the port, which would make the check useless.

Frames are read for their header only (magic, message id, source system and
component), so no MAVLink library is needed: every message whose source system
is not PX4's own 1 is reported as foreign, and the command-carrying types are
named. A guard run prints and appends one MAVLINK_GUARD summary line.

Exit codes: 0 ok, 1 a guarded port is held by another process, 2 usage.
"""
from __future__ import annotations

import argparse
import os
import signal
import socket
import sys
import time
from pathlib import Path

# The remote ports PX4 sends its streams to: the guard holds these.
DEFAULT_PORTS = (14550, 14540)
# The ports PX4 SITL itself listens on (ROMFS/px4fmu_common/init.d-posix/
# px4-rc.mavlink, instance 0). Our own PX4 binds them a moment after the launch,
# so they must be free beforehand: if another PX4 - a second simulator on the
# same host, as on 2026-11-12 - already holds them, the two instances would
# fight over the same sockets and neither flight's evidence would mean anything.
PX4_LISTEN_PORTS = (18570, 14580, 14280, 13030)
PX4_SYSTEM_ID = 1
MAGIC_V2 = 0xFD
MAGIC_V1 = 0xFE
# Types that carry a mode, an arming action, a command or a setpoint: the ones
# that can take a vehicle away from the stack.
COMMAND_MESSAGE_IDS = {
    11: "SET_MODE",
    75: "COMMAND_INT",
    76: "COMMAND_LONG",
    84: "SET_POSITION_TARGET_LOCAL_NED",
    86: "SET_POSITION_TARGET_GLOBAL_INT",
}


def guarded_ports(override: str | None) -> tuple[int, ...]:
    if not override:
        return DEFAULT_PORTS
    return tuple(int(part) for part in override.split() if part)


def bind_exclusive(port: int) -> socket.socket:
    """Bind 127.0.0.1:port without SO_REUSEADDR, so a held port is an error."""
    held = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        held.bind(("127.0.0.1", port))
    except OSError:
        held.close()
        raise
    return held


def probe(ports: tuple[int, ...], listen_ports: tuple[int, ...] = ()) -> int:
    for port in tuple(ports) + tuple(listen_ports):
        try:
            bind_exclusive(port).close()
        except OSError as error:
            print(f"MAVLINK_GUARD probe=occupied port={port} error={error.strerror}")
            return 1
    held = ",".join(str(p) for p in ports)
    listened = ",".join(str(p) for p in listen_ports) or "-"
    print(f"MAVLINK_GUARD probe=free ports={held} px4_listen_ports={listened}")
    return 0


def frame_header(data: bytes) -> tuple[int, int, int, int] | None:
    """(message_id, source_system, source_component, payload_length) or None."""
    if len(data) < 8:
        return None
    magic = data[0]
    if magic == MAGIC_V2:
        if len(data) < 10:
            return None
        return (int.from_bytes(data[7:10], "little"), data[5], data[6], data[1])
    if magic == MAGIC_V1:
        return (data[5], data[3], data[4], data[1])
    return None


def hold(ports: tuple[int, ...], log_path: Path | None, seconds: float,
         exit_with_parent: bool) -> int:
    sockets = []
    try:
        for port in ports:
            sockets.append(bind_exclusive(port))
    except OSError as error:
        for held in sockets:
            held.close()
        print(f"MAVLINK_GUARD held=false port_error={error.strerror}")
        return 1
    for held in sockets:
        held.settimeout(0.25)
    log = None
    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log = log_path.open("a")
        log.write(f"MAVLINK_GUARD start={time.strftime('%Y-%m-%dT%H:%M:%S')} "
                  f"ports={','.join(str(p) for p in ports)}\n")
        log.flush()

    def record(line: str) -> None:
        if log is not None:
            log.write(line + "\n")
            log.flush()

    stop = False

    def on_signal(_signum: int, _frame: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    # The flight wrapper starts this guard in the background, so the guard has
    # to notice on its own when that wrapper goes away: nothing else would free
    # the ports, and a leftover holder would refuse the next flight.
    parent = os.getppid()
    deadline = time.monotonic() + seconds if seconds > 0 else None
    datagrams = 0
    foreign = 0
    foreign_types: dict[str, int] = {}
    undecodable = 0
    started = time.monotonic()
    while not stop and (deadline is None or time.monotonic() < deadline):
        if exit_with_parent and os.getppid() != parent:
            break
        for held in sockets:
            try:
                data, address = held.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                continue
            datagrams += 1
            header = frame_header(data)
            if header is None:
                undecodable += 1
                continue
            message_id, source_system, source_component, _ = header
            if source_system == PX4_SYSTEM_ID:
                continue  # PX4's own outbound stream arrives here too
            foreign += 1
            name = COMMAND_MESSAGE_IDS.get(message_id, f"id{message_id}")
            foreign_types[name] = foreign_types.get(name, 0) + 1
            record(f"MAVLINK_GUARD foreign type={name} src=({source_system},"
                   f"{source_component}) from={address[0]}:{address[1]} "
                   f"elapsed_s={time.monotonic() - started:.1f}")
    for held in sockets:
        held.close()
    summary = (f"MAVLINK_GUARD freed datagrams={datagrams} foreign_intents={foreign} "
               f"foreign_types={dict(sorted(foreign_types.items()))} "
               f"undecodable={undecodable} seconds={time.monotonic() - started:.1f}")
    print(summary)
    if log is not None:
        log.write(summary + "\n")
        log.close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", action="store_true",
                        help="report whether the guarded ports are free")
    parser.add_argument("--log", type=Path, default=None,
                        help="append the guard's record to this file")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="hold for this long; 0 holds until SIGTERM or SIGINT")
    parser.add_argument("--ports", default=None,
                        help="space-separated remote ports to hold; default 14550 14540")
    parser.add_argument("--listen-ports", default=None,
                        help="space-separated PX4 listen ports that must be free "
                             "before the launch; default 18570 14580 14280 13030, "
                             "'none' to skip")
    parser.add_argument("--keep-parent", action="store_true",
                        help="hold even after the process that started this one exits")
    args = parser.parse_args()
    ports = guarded_ports(args.ports)
    if not ports:
        print("MAVLINK_GUARD no ports to guard", file=sys.stderr)
        return 2
    if args.listen_ports is None:
        listen_ports = PX4_LISTEN_PORTS
    elif args.listen_ports.strip().lower() in ("", "none"):
        listen_ports = ()
    else:
        listen_ports = tuple(int(p) for p in args.listen_ports.split() if p)
    if args.probe:
        return probe(ports, listen_ports)
    return hold(ports, args.log, args.seconds, not args.keep_parent)


if __name__ == "__main__":
    raise SystemExit(main())
