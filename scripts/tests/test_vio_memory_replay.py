"""Behavioral tests for recorded VIO and stereo memory inputs."""
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[2] / "tools/vio/replay_memory.py"
SPEC = importlib.util.spec_from_file_location("replay_memory", SCRIPT)
REPLAY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPLAY)


def write_csv(path, fields, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


class MemoryReplayInputTest(unittest.TestCase):
    def test_only_healthy_estimator_pose_is_paired_at_px4_precision(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            calibration = {"width": 4, "height": 3, "focal_px": 5.0,
                           "principal_x_px": 2.0, "principal_y_px": 1.5,
                           "baseline_m": 0.2, "left_camera_to_body_frd_wxyz": [1, 0, 0, 0],
                           "left_camera_position_body_frd_m": [1, 2, 3]}
            (root / "calibration.json").write_text(json.dumps(calibration), encoding="utf-8")
            frames = [{"side": side, "stamp_s": stamp, "width": 4, "height": 3, "byte_offset": 0}
                      for stamp in ("1.000000128", "1.050000128") for side in ("left", "right")]
            write_csv(root / "frames.csv", ["side", "stamp_s", "width", "height", "byte_offset"], frames)
            fields = ["stamp_s", *REPLAY.ESTIMATE_FIELDS, "healthy", "dead_reckoning", "tx"]
            values = dict(px=1, py=2, pz=-3, qw=1, qx=0, qy=0, qz=0,
                          vx=0, vy=0, vz=0, heading_variance_rad2=0.001, tx="not a measurement")
            estimates = [dict(values, stamp_s="1.000000128", healthy=1, dead_reckoning=0),
                         dict(values, stamp_s="1.050000128", healthy=0, dead_reckoning=0)]
            estimate = root / "estimate.csv"
            write_csv(estimate, fields, estimates)
            cal, pairs, health, frame_count, available = REPLAY.read_inputs(root, estimate, None)
            self.assertEqual((frame_count, available, len(pairs)), (2, 1, 1))
            self.assertEqual(pairs[0][0], 11_000_000)
            self.assertEqual((health["healthy"], health["unhealthy"]), (1, 1))
            self.assertNotIn("tx", pairs[0][2])
            _, memory = REPLAY.configuration(cal, pairs)
            self.assertEqual(memory["lidar_extrinsic_quaternion_lidar_flu_to_body_frd"],
                             [0.5, 0.5, -0.5, 0.5])

    def test_duplicate_frame_after_microsecond_quantization_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "calibration.json").write_text('{"width":4,"height":3}', encoding="utf-8")
            write_csv(root / "frames.csv", ["side", "stamp_s", "width", "height", "byte_offset"],
                      [{"side": "left", "stamp_s": stamp, "width": 4, "height": 3, "byte_offset": 0}
                       for stamp in ("1.000000100", "1.000000200")])
            with self.assertRaises(ValueError):
                REPLAY.read_inputs(root, root / "unused.csv", None)


if __name__ == "__main__":
    unittest.main()
