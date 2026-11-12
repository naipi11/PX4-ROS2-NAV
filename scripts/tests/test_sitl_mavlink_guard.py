"""The MAVLink guard: it must refuse a held port, and count only foreign traffic.

A foreign client that binds PX4 SITL's MAVLink port can command the vehicle in
flight; on 2026-11-12 that destroyed four flights
(log/runs/nav_state17_root_cause_20261112.txt). The guard is the flight gate for
that, so its two behaviours are asserted here: a held port is reported and
refused, and a free one is held while PX4's own outbound stream is not mistaken
for an intruder.
"""

from __future__ import annotations

import socket
import subprocess
import sys
import time
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
GUARD = REPOSITORY / "scripts" / "sitl_mavlink_guard.py"
COMMAND_LONG = 76
PX4_SYSTEM_ID = 1


def free_port() -> int:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


def command_long(source_system: int, source_component: int,
                 message_id: int = COMMAND_LONG) -> bytes:
    """A MAVLink v2 COMMAND_LONG frame; the guard reads only its header."""
    payload = bytes(33)
    return (bytes([0xFD, len(payload), 0, 0, 0, source_system, source_component])
            + message_id.to_bytes(3, "little") + payload + bytes(2))


class MavlinkGuardTest(unittest.TestCase):
    def test_probe_reports_a_free_port(self) -> None:
        port = free_port()
        result = subprocess.run(
            [sys.executable, str(GUARD), "--probe", "--ports", str(port)],
            capture_output=True, text=True, check=False)
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)
        self.assertIn("probe=free", result.stdout)

    def test_probe_refuses_a_port_another_process_holds(self) -> None:
        holder = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        holder.bind(("127.0.0.1", 0))
        try:
            port = holder.getsockname()[1]
            result = subprocess.run(
                [sys.executable, str(GUARD), "--probe", "--ports", str(port)],
                capture_output=True, text=True, check=False)
        finally:
            holder.close()
        self.assertEqual(1, result.returncode, result.stdout + result.stderr)
        self.assertIn(f"probe=occupied port={port}", result.stdout)
        self.assertIn("Address already in use", result.stdout)

    def test_probe_refuses_when_another_px4_holds_a_listen_port(self) -> None:
        # Two PX4 SITL instances on one host claim the same sockets; a flight
        # must not start into that, because neither instance's evidence would
        # then describe a controlled vehicle.
        remote, listen_port = free_port(), free_port()
        px4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        px4.bind(("0.0.0.0", listen_port))
        try:
            result = subprocess.run(
                [sys.executable, str(GUARD), "--probe", "--ports", str(remote),
                 "--listen-ports", str(listen_port)],
                capture_output=True, text=True, check=False)
        finally:
            px4.close()
        self.assertEqual(1, result.returncode, result.stdout + result.stderr)
        self.assertIn(f"probe=occupied port={listen_port}", result.stdout)

    def test_probe_accepts_free_remote_and_listen_ports(self) -> None:
        remote, listen_port = free_port(), free_port()
        result = subprocess.run(
            [sys.executable, str(GUARD), "--probe", "--ports", str(remote),
             "--listen-ports", str(listen_port)],
            capture_output=True, text=True, check=False)
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)
        self.assertIn(f"px4_listen_ports={listen_port}", result.stdout)

    def test_probe_refuses_promptly_when_the_holder_is_not_our_guard(self) -> None:
        # A guard of ours that is winding down may hold the ports for a moment;
        # any other holder must be refused at once, or a real intruder would be
        # waited out and forgotten.
        import tempfile
        holder = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        holder.bind(("127.0.0.1", 0))
        port = holder.getsockname()[1]
        other = subprocess.Popen(["sleep", "30"])
        try:
            with tempfile.TemporaryDirectory() as directory:
                pidfile = Path(directory) / "mavlink_guard.pid"
                pidfile.write_text(f"{other.pid}\n")
                started = time.monotonic()
                result = subprocess.run(
                    [sys.executable, str(GUARD), "--probe", "--ports", str(port),
                     "--log", str(Path(directory) / "mavlink_guard.log")],
                    capture_output=True, text=True, check=False)
                elapsed = time.monotonic() - started
        finally:
            holder.close()
            other.kill()
            other.wait()
        self.assertEqual(1, result.returncode, result.stdout + result.stderr)
        self.assertLess(elapsed, 3.0, "probe waited on a holder that is not our guard")

    def test_hold_publishes_and_removes_its_pidfile(self) -> None:
        import tempfile
        port = free_port()
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "mavlink_guard.log"
            guard = subprocess.Popen(
                [sys.executable, str(GUARD), "--seconds", "3", "--ports", str(port),
                 "--log", str(log)],
                stdout=subprocess.PIPE, text=True)
            pidfile = log.with_suffix(".pid")
            try:
                time.sleep(1.0)
                self.assertTrue(pidfile.is_file(), "no pidfile while the guard holds")
                self.assertEqual(guard.pid, int(pidfile.read_text().strip()))
                guard.communicate(timeout=30)
            finally:
                if guard.poll() is None:
                    guard.kill()
                    guard.communicate()
            self.assertFalse(pidfile.exists(), "pidfile left behind after the guard freed")

    def test_hold_refuses_a_held_port(self) -> None:
        holder = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        holder.bind(("127.0.0.1", 0))
        try:
            port = holder.getsockname()[1]
            result = subprocess.run(
                [sys.executable, str(GUARD), "--seconds", "1", "--ports", str(port)],
                capture_output=True, text=True, check=False)
        finally:
            holder.close()
        self.assertEqual(1, result.returncode, result.stdout + result.stderr)
        self.assertIn("held=false", result.stdout)

    def test_hold_counts_foreign_intents_and_ignores_px4(self) -> None:
        port = free_port()
        guard = subprocess.Popen(
            [sys.executable, str(GUARD), "--seconds", "4", "--ports", str(port)],
            stdout=subprocess.PIPE, text=True)
        try:
            time.sleep(1.0)
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sender.sendto(command_long(PX4_SYSTEM_ID, 1), ("127.0.0.1", port))
                sender.sendto(command_long(255, 0), ("127.0.0.1", port))
                time.sleep(0.2)
                sender.sendto(command_long(255, 0), ("127.0.0.1", port))
                time.sleep(0.3)
            finally:
                sender.close()
            output, _ = guard.communicate(timeout=30)
        finally:
            if guard.poll() is None:
                guard.kill()
                guard.communicate()
        self.assertEqual(0, guard.returncode, output)
        self.assertIn("foreign_intents=2", output)
        self.assertIn("'COMMAND_LONG': 2", output)

    def test_another_simulator_telemetry_is_not_counted_as_an_intent(self) -> None:
        # A second PX4 on this host has its own MAV_SYS_ID and also sends to
        # 14550. Its telemetry must not read as an intrusion; only frames that
        # can command the vehicle may.
        port = free_port()
        guard = subprocess.Popen(
            [sys.executable, str(GUARD), "--seconds", "4", "--ports", str(port)],
            stdout=subprocess.PIPE, text=True)
        try:
            time.sleep(1.0)
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                # Heartbeat (id 0) from another simulator's system 3.
                sender.sendto(command_long(3, 1, message_id=0), ("127.0.0.1", port))
                sender.sendto(command_long(3, 1, message_id=32), ("127.0.0.1", port))
                time.sleep(0.3)
                sender.sendto(command_long(3, 1), ("127.0.0.1", port))
                time.sleep(0.3)
            finally:
                sender.close()
            output, _ = guard.communicate(timeout=30)
        finally:
            if guard.poll() is None:
                guard.kill()
                guard.communicate()
        self.assertEqual(0, guard.returncode, output)
        self.assertIn("foreign_intents=1", output)
        self.assertIn("'COMMAND_LONG': 1", output)

    def test_the_guard_frees_its_ports_when_its_parent_exits(self) -> None:
        # The flight wrapper starts the guard in the background, so a wrapper
        # that dies early must not leave the ports held: the next flight would
        # refuse to start. The guard notices the parent going away by itself.
        port = free_port()
        launcher = subprocess.run(
            ["sh", "-c", f'"{sys.executable}" "{GUARD}" --seconds 0 '
                         f'--ports {port} >/dev/null 2>&1 & sleep 0.8'],
            check=False)
        self.assertEqual(0, launcher.returncode)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            probe = subprocess.run(
                [sys.executable, str(GUARD), "--probe", "--ports", str(port)],
                capture_output=True, text=True, check=False)
            if probe.returncode == 0:
                return
            time.sleep(0.3)
        self.fail(f"port {port} stayed held after the guard's parent exited")


if __name__ == "__main__":
    unittest.main()
