"""Consumer-level tests for paired trajectory reports."""

import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[2] / "tools/vio/trajectory_report.py"
FIELDS = ("stamp_s px py pz qw qx qy qz tx ty tz tqw tqx tqy tqz").split()


def row(stamp, estimate, reference, q=(1, 0, 0, 0), tq=(1, 0, 0, 0)):
    return dict(zip(FIELDS, (stamp, *estimate, *q, *reference, *tq)))


class TrajectoryReportTest(unittest.TestCase):
    def run_report(self, rows, health=False, align=False, fields=None):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "replay.csv"
            output = Path(directory) / "report.json"
            with source.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields or FIELDS + (["healthy"] if health else []),
                                        extrasaction="ignore")
                writer.writeheader()
                writer.writerows(rows)
            command = [sys.executable, str(SCRIPT), str(source), "--output", str(output)]
            if align:
                command.append("--align-initial-yaw")
            process = subprocess.run(command, capture_output=True, text=True, check=False)
            result = json.loads(output.read_text(encoding="utf-8")) if output.exists() else None
            return process, result

    def test_constant_offset_and_health(self):
        rows = [row(10, (1, 2, 2), (0, 0, 0)),
                row(11, (4, 6, 2), (3, 4, 0)),
                row(12, (4, 6, 4), (3, 4, 2))]
        for sample, health in zip(rows, (1, 0, 1)):
            sample["healthy"] = health
        process, result = self.run_report(rows, health=True)
        self.assertEqual(process.returncode, 0, process.stderr)
        self.assertEqual(result["samples"], 3)
        self.assertEqual(result["duration_s"], 2)
        self.assertEqual(result["reference_path_length_3d_m"], 7)
        self.assertEqual(result["reference_path_length_horizontal_m"], 5)
        self.assertEqual(result["final_error_3d_m"], 3)
        self.assertAlmostEqual(result["final_error_horizontal_m"], math.sqrt(5))
        self.assertAlmostEqual(result["final_error_3d_percent_of_path"], 300 / 7)
        self.assertAlmostEqual(result["final_error_horizontal_percent_of_path"], 20 * math.sqrt(5))
        self.assertEqual(result["absolute_error_rmse_m"], 3)
        self.assertEqual(result["absolute_error_p95_m"], 3)
        self.assertEqual(result["absolute_error_max_m"], 3)
        self.assertEqual(result["health_coverage"], 3)
        self.assertAlmostEqual(result["healthy_fraction"], 2 / 3)
        self.assertEqual(result["alignment"], "none")
        process, result = self.run_report(rows, align=True)
        self.assertEqual(process.returncode, 0, process.stderr)
        self.assertEqual(result["absolute_error_max_m"], 0)
        self.assertIsNone(result["healthy_fraction"])
        self.assertEqual(result["health_coverage"], 0)

    def test_initial_yaw_alignment_and_scale_remains(self):
        q = (math.sqrt(0.5), 0, 0, math.sqrt(0.5))
        rows = [row(0, (10, 20, 3), (1, 2, 0), q=q),
                row(1, (10, 22, 3), (2, 2, 0), q=q),
                row(2, (8, 22, 3), (2, 3, 0), q=q)]
        process, result = self.run_report(rows, align=True)
        self.assertEqual(process.returncode, 0, process.stderr)
        self.assertEqual(result["alignment"], "initial_position_yaw")
        self.assertAlmostEqual(result["final_error_3d_m"], math.sqrt(2))
        self.assertAlmostEqual(result["absolute_error_rmse_m"], 1)
        self.assertAlmostEqual(result["absolute_error_p95_m"], 1 + 0.9 * (math.sqrt(2) - 1))
        self.assertAlmostEqual(result["absolute_error_max_m"], math.sqrt(2))
        process, unaligned = self.run_report(rows)
        self.assertEqual(process.returncode, 0, process.stderr)
        self.assertGreater(unaligned["final_error_3d_m"], 10)

    def test_rejects_invalid_input(self):
        valid = [row(0, (0, 0, 0), (0, 0, 0)), row(1, (1, 0, 0), (1, 0, 0))]
        cases = [
            (valid[:1], False, None),
            ([valid[0], dict(valid[1], stamp_s=0)], False, None),
            ([valid[0], dict(valid[1], stamp_s=-1)], False, None),
            ([valid[0], dict(valid[1], stamp_s="nan")], False, None),
            ([valid[0], dict(valid[1], px="inf")], False, None),
            ([valid[0], dict(valid[1], qw=0)], False, None),
            ([valid[0], dict(valid[1], tqw=2)], False, None),
            (valid, False, FIELDS[:-1]),
            ([dict(valid[0], healthy=1), dict(valid[1], healthy="")], True, None),
            ([dict(valid[0], healthy=1), dict(valid[1], healthy=2)], True, None),
        ]
        for rows, health, fields in cases:
            with self.subTest(rows=rows, fields=fields):
                process, result = self.run_report(rows, health=health, fields=fields)
                self.assertNotEqual(process.returncode, 0)
                self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
