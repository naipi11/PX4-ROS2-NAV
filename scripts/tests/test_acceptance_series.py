from __future__ import annotations

import json
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import tempfile
import unittest

REPOSITORY = Path(__file__).resolve().parents[2]

# These are real child processes in an isolated Git repository. They replace
# the expensive simulator/host load, not the series runner or its verdict code.
FLIGHT = r'''#!/usr/bin/env python3
import atexit, hashlib, json, os, signal, subprocess
from pathlib import Path
run = os.environ['DRONE_GAZEBO_RUN_ID']
mode = os.environ.get('SERIES_FIXTURE_MODE', 'passed')
with open('log/launched.txt', 'a') as stream:
    stream.write(run + '\n')
active = Path('log/active_pid')
active.write_text(str(os.getpid()))
atexit.register(lambda: active.unlink(missing_ok=True))
signal.signal(signal.SIGTERM, lambda *_: exit(143))
if mode == 'wait':
    with open('log/ready', 'w') as stream:
        stream.write('ready\n')
    signal.pause()
directory = Path('log/runs') / run
directory.mkdir(parents=True, exist_ok=True)
commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
sensor = os.environ.get('NAVIGATION_SENSOR_PROFILE', 'stereo_tof')
manifest = {'schema': 'drone_city_nav_runtime_manifest_v1', 'run_id': run,
            'repository': {'commit': commit, 'dirty': False},
            'effective_overrides': {'NAVIGATION_SENSOR_PROFILE': sensor,
                'LOCALIZATION_PROFILE': 'lidar_inertial' if sensor == 'lidar' else 'visual_inertial'}}
if mode == 'wrong_profile':
    manifest['effective_overrides']['LOCALIZATION_PROFILE'] = 'gnss'
payload = json.dumps(manifest).encode()
(directory / 'manifest.json').write_bytes(payload)
failed = mode in ('failed', 'void')
report = {'schema': 'drone_city_nav_headless_result_v1',
          'runtime_manifest': {'sha256': hashlib.sha256(payload).hexdigest()},
          'mission_check': True, 'allow_mission_failure': False,
          'passed': not failed, 'errors': ['FAIL: mission did not finish'] if failed else [],
          'notes': ['NOTE: a diagnostic is outside its reference']}
if mode == 'stale_check':
    report['runtime_manifest']['sha256'] = '0' * 64
if mode != 'missing_check':
    (directory / 'mission_check.json').write_text(json.dumps(report))
factor = 0.4 if mode in ('void', 'host_void') else 1.0
(directory / 'resources.csv').write_text(f'stamp_s,real_time_factor\n1,{factor}\n2,{factor}\n')
(directory / 'ros_drone_nav.log').write_text('VISUAL_INERTIAL_ODOMETRY healthy=true pose_age_max_ms=5.0\n')
if mode == 'missing_ros':
    (directory / 'ros_drone_nav.log').unlink()
if mode == 'source_changed':
    Path('source.txt').write_text('changed during flight')
if mode == 'commit_changed':
    subprocess.run(['git', 'commit', '--allow-empty', '-qm', 'intervening commit'], check=True)
print('FAIL: mission did not finish' if failed else 'OK: mission monitor verifies complete flight')
raise SystemExit(17 if failed or mode == 'command_failed' else 0)
'''

STOP = r'''#!/usr/bin/env python3
import os, signal
from pathlib import Path
active = Path('log/active_pid')
if active.exists():
    try:
        os.kill(int(active.read_text()), signal.SIGTERM)
    except ProcessLookupError:
        pass
'''

RECORDER = r'''#!/usr/bin/env python3
import os, signal
from pathlib import Path
if os.environ.get('SERIES_FIXTURE_MODE') == 'wait':
    Path('log/recorder_pid').write_text(str(os.getpid()))
    with open('log/recorder_ready', 'w') as stream:
        stream.write('ready\n')
    signal.pause()
print('RECORDING ok\nRECORDING ok')
'''


class AcceptanceSeriesTest(unittest.TestCase):
    def setUp(self) -> None:
        temporary = tempfile.TemporaryDirectory(prefix='acceptance-series-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        for directory in ('tools', 'scripts', 'bin', 'log'):
            (self.root / directory).mkdir()
        for relative in ('tools/series2.sh', 'tools/acceptance_result.py',
                         'tools/host_verdict.py', 'tools/record_index.py',
                         'tools/mxw.sh', 'tools/record_flight.sh',
                         'scripts/runtime_manifest.py'):
            shutil.copy2(REPOSITORY / relative, self.root / relative)
        helpers = {
            'scripts/dev_shell.sh': FLIGHT,
            'bin/gnome-session-inhibit': RECORDER,
            'scripts/stop_sim.sh': STOP,
            'scripts/prune_sim_logs.sh': '#!/bin/sh\nexit 0\n',
            'bin/sleep': '#!/bin/sh\nexit 0\n',
            'bin/ps': '#!/bin/sh\nexit 0\n',
            'bin/pgrep': "#!/bin/sh\nprintf '0\\n'\nexit 1\n",
            'bin/cut': "#!/bin/sh\nprintf '0.0\\n'\n",
        }
        for name, content in helpers.items():
            path = self.root / name
            path.write_text(content, encoding='utf-8')
            path.chmod(0o755)
        (self.root / '.gitignore').write_text('log/\n__pycache__/\n', encoding='utf-8')
        (self.root / 'source.txt').write_text('original source', encoding='utf-8')
        for arguments in (['init', '-q'], ['config', 'user.name', 'Fixture'],
                          ['config', 'user.email', 'fixture@example.invalid'],
                          ['add', '.'], ['commit', '-qm', 'fixture']):
            subprocess.run(['git', *arguments], cwd=self.root, check=True,
                           capture_output=True, text=True)
        self.environment = dict(os.environ, PATH=str(self.root / 'bin') + ':' + os.environ['PATH'])
        for name in ('NAVIGATION_SENSOR_PROFILE', 'LOCALIZATION_PROFILE', 'CAMERA_PROFILE'):
            self.environment.pop(name, None)

    def invoke(self, *runs: str, mode: str = 'passed') -> subprocess.CompletedProcess:
        return subprocess.run(['bash', 'tools/series2.sh', *runs], cwd=self.root,
                              env=dict(self.environment, SERIES_FIXTURE_MODE=mode),
                              capture_output=True, text=True, timeout=20)

    def launched(self) -> list[str]:
        path = self.root / 'log/launched.txt'
        return path.read_text().splitlines() if path.exists() else []

    def result(self, run: str = 'first') -> dict:
        return json.loads((self.root / 'log/runs' / run / 'acceptance.json').read_text())

    def test_valid_flight_failure_stops_the_series(self) -> None:
        result = self.invoke('first', 'second', mode='failed')
        self.assertEqual(1, result.returncode, result.stdout + result.stderr)
        self.assertEqual(['first'], self.launched())
        self.assertEqual('failed', self.result()['status'])
        self.assertEqual(17, self.result()['command_exit_code'])

    def test_host_void_does_not_become_a_valid_flight_failure(self) -> None:
        result = self.invoke('first', 'second', mode='void')
        self.assertEqual(2, result.returncode, result.stdout + result.stderr)
        self.assertEqual(['first'], self.launched())
        self.assertEqual('void', self.result()['status'])
        self.assertTrue(self.result()['host_reasons'])

    def test_a_successful_checker_cannot_hide_a_failed_command(self) -> None:
        result = self.invoke('first', mode='command_failed')
        self.assertEqual(1, result.returncode, result.stdout + result.stderr)
        self.assertTrue(self.result()['check_passed'])
        self.assertEqual('failed', self.result()['status'])

    def test_missing_or_differently_bound_evidence_stops_the_series(self) -> None:
        for mode in ('missing_check', 'stale_check', 'missing_ros'):
            with self.subTest(mode=mode):
                run = 'run_' + mode
                result = self.invoke(run, 'unused_' + mode, mode=mode)
                self.assertEqual(3, result.returncode, result.stdout + result.stderr)
                self.assertEqual('error', self.result(run)['status'])
                self.assertFalse((self.root / 'log/runs' / ('unused_' + mode)).exists())
                self.assertNotIn('HOST valid', result.stdout)

    def test_changed_source_or_commit_invalidates_the_flight(self) -> None:
        result = self.invoke('first', 'second', mode='source_changed')
        self.assertEqual(3, result.returncode, result.stdout + result.stderr)
        self.assertEqual(['first'], self.launched())
        self.assertEqual('error', self.result()['status'])

    def test_an_intervening_commit_invalidates_the_flight(self) -> None:
        result = self.invoke('first', 'second', mode='commit_changed')
        self.assertEqual(3, result.returncode, result.stdout + result.stderr)
        self.assertEqual(['first'], self.launched())
        self.assertEqual('error', self.result()['status'])

    def test_dirty_source_never_starts_a_flight(self) -> None:
        (self.root / 'source.txt').write_text('uncommitted change')
        result = self.invoke('first')
        self.assertEqual(3, result.returncode, result.stdout + result.stderr)
        self.assertEqual([], self.launched())

    def test_setup_failure_is_not_a_measured_flight_failure(self) -> None:
        (self.root / 'log/tools').mkdir()
        (self.root / 'log/tools/journal').write_text('occupied path')
        result = self.invoke('first')
        self.assertEqual(3, result.returncode, result.stdout + result.stderr)
        self.assertEqual([], self.launched())

    def test_existing_run_evidence_is_not_overwritten(self) -> None:
        directory = self.root / 'log/runs/first'
        directory.mkdir(parents=True)
        proof = directory / 'acceptance.log'
        proof.write_text('original flight evidence')
        result = self.invoke('first', 'second')
        self.assertEqual(3, result.returncode, result.stdout + result.stderr)
        self.assertEqual([], self.launched())
        self.assertEqual('original flight evidence', proof.read_text())

    def test_invalid_or_repeated_ids_never_start_a_flight(self) -> None:
        for arguments in ((), ('../outside',), ('first', 'first')):
            with self.subTest(arguments=arguments):
                result = self.invoke(*arguments)
                self.assertEqual(3, result.returncode, result.stdout + result.stderr)
                self.assertEqual([], self.launched())

    def test_success_continues_and_retains_evidence_without_the_loose_log(self) -> None:
        result = self.invoke('first', 'second')
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)
        self.assertEqual(['first', 'second'], self.launched())
        for run in ('first', 'second'):
            self.assertEqual('passed', self.result(run)['status'])
            loose = self.root / 'log/tools' / f'run_{run}.log'
            content = loose.read_bytes()
            loose.unlink()
            directory = self.root / 'log/runs' / run
            self.assertTrue((directory / '.keep').is_file())
            self.assertEqual(content, (directory / 'acceptance.log').read_bytes())

    def test_a_different_recorded_localization_profile_is_not_accepted(self) -> None:
        result = self.invoke('--lidar', 'first', mode='wrong_profile')
        self.assertEqual(3, result.returncode, result.stdout + result.stderr)
        self.assertEqual('error', self.result()['status'])

    def test_recording_cannot_hide_void_failed_or_unproved_flights(self) -> None:
        for mode, expected in (('host_void', 2), ('failed', 1), ('missing_check', 3)):
            with self.subTest(mode=mode):
                self.invoke(mode, mode=mode)
                (self.root / f'log/tools/record_{mode}.log').write_text(
                    'RECORDING ok\nRECORDING ok\n')
                result = subprocess.run(
                    ['python3', 'tools/record_index.py', 'point-to-point', mode, 'log'],
                    cwd=self.root, capture_output=True, text=True, timeout=10)
                self.assertEqual(expected, result.returncode, result.stdout + result.stderr)
                self.assertNotEqual('OK', result.stdout.strip())

    def test_matrix_observer_matches_run_identity_and_propagates_failure(self) -> None:
        (self.root / 'log/tools').mkdir()
        marker = self.root / 'log/tools/mx_flag.txt'
        marker.write_text('DONE 1 other_first exit=0\n')
        ignored = subprocess.run(['bash', 'tools/mxw.sh', 'first'], cwd=self.root,
                                 env=self.environment, capture_output=True, text=True, timeout=10)
        self.assertEqual('RUNNING', ignored.stdout.strip())
        marker.write_text('DONE 1 first exit=1\n')
        failed = subprocess.run(['bash', 'tools/mxw.sh', 'first'], cwd=self.root,
                                env=self.environment, capture_output=True, text=True, timeout=10)
        self.assertEqual(1, failed.returncode, failed.stdout + failed.stderr)
        self.assertEqual('DONE exit=1', failed.stdout.strip())

    def test_a_second_series_cannot_interrupt_the_owner_and_term_cleans_up(self) -> None:
        fifo = self.root / 'log/ready'
        os.mkfifo(fifo)
        descriptor = os.open(fifo, os.O_RDONLY | os.O_NONBLOCK)
        process = subprocess.Popen(['bash', 'tools/series2.sh', 'first'], cwd=self.root,
                                   env=dict(self.environment, SERIES_FIXTURE_MODE='wait'),
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            ready, _, _ = select.select([descriptor], [], [], 10)
            self.assertTrue(ready, 'flight child did not become ready')
            self.assertEqual(b'ready\n', os.read(descriptor, 128))
            second = self.invoke('second')
            self.assertEqual(3, second.returncode, second.stdout + second.stderr)
            self.assertIsNone(process.poll())
            self.assertEqual(['first'], self.launched())
            process.send_signal(signal.SIGTERM)
            stdout, stderr = process.communicate(timeout=10)
            self.assertEqual(143, process.returncode, stdout + stderr)
            self.assertEqual('interrupted', self.result()['status'])
            self.assertFalse((self.root / 'log/active_pid').exists())
        finally:
            os.close(descriptor)
            if process.poll() is None:
                process.kill()
                process.communicate(timeout=10)
            subprocess.run(['scripts/stop_sim.sh'], cwd=self.root, check=True,
                           capture_output=True, timeout=10)

    def test_recording_interrupt_reaps_its_owned_flight_and_recorder(self) -> None:
        descriptors = []
        for name in ('ready', 'recorder_ready'):
            fifo = self.root / 'log' / name
            os.mkfifo(fifo)
            descriptors.append(os.open(fifo, os.O_RDONLY | os.O_NONBLOCK))
        process = subprocess.Popen(
            ['bash', 'tools/record_flight.sh', 'point-to-point', 'recorded', 'log/video'],
            cwd=self.root, env=dict(self.environment, SERIES_FIXTURE_MODE='wait'),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            for descriptor in descriptors:
                ready, _, _ = select.select([descriptor], [], [], 10)
                self.assertTrue(ready, 'owned child did not become ready')
                self.assertEqual(b'ready\n', os.read(descriptor, 128))
            recorder_pid = int((self.root / 'log/recorder_pid').read_text())
            process.send_signal(signal.SIGTERM)
            stdout, stderr = process.communicate(timeout=10)
            self.assertEqual(143, process.returncode, stdout + stderr)
            self.assertEqual('interrupted', self.result('recorded')['status'])
            self.assertFalse((self.root / 'log/active_pid').exists())
            with self.assertRaises(ProcessLookupError):
                os.kill(recorder_pid, 0)
        finally:
            for descriptor in descriptors:
                os.close(descriptor)
            if process.poll() is None:
                process.kill()
                process.communicate(timeout=10)
            pid_path = self.root / 'log/recorder_pid'
            if pid_path.exists():
                try:
                    os.killpg(int(pid_path.read_text()), signal.SIGTERM)
                except ProcessLookupError:
                    pass
            subprocess.run(['scripts/stop_sim.sh'], cwd=self.root, check=True,
                           capture_output=True, timeout=10)


if __name__ == '__main__':
    unittest.main()
