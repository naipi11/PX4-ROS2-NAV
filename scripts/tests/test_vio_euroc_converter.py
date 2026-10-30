"""Consumer-level tests for conversion of synthetic ASL stereo sequences."""

import csv
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[2] / "tools/vio/convert_euroc.py"
EPOCH = 1403715273262142976
WIDTH, HEIGHT = 32, 24


def transform(rotation, translation):
    return {"rows": 4, "cols": 4, "data": [
        *rotation[0], translation[0], *rotation[1], translation[1],
        *rotation[2], translation[2], 0, 0, 0, 1]}


IDENTITY = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
QUARTER_TURN = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]


def write_rows(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        csv.writer(stream).writerows(rows)


def read_rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def fixture(root):
    mav = root / "mav0"
    camera = {"camera_model": "pinhole", "distortion_model": "radial-tangential",
              "resolution": [WIDTH, HEIGHT], "intrinsics": [24, 24, 15.5, 11.5],
              "distortion_coefficients": [0.2, 0, 0, 0]}
    sensors = {
        "cam0": dict(camera, T_BS=transform(IDENTITY, [1, 2, 3])),
        "cam1": dict(camera, T_BS=transform(IDENTITY, [1.2, 2, 3])),
        "imu0": {"T_BS": transform(QUARTER_TURN, [0.5, 1, 1]),
                 "gyroscope_noise_density": 0.01, "accelerometer_noise_density": 0.02,
                 "gyroscope_random_walk": 0.003, "accelerometer_random_walk": 0.004},
        "vicon0": {"T_BS": transform(IDENTITY, [0, 1, 0])},
    }
    for name, sensor in sensors.items():
        directory = mav / name
        directory.mkdir(parents=True)
        (directory / "sensor.yaml").write_text(json.dumps(sensor), encoding="utf-8")
    # A planar ramp makes the result of radial distortion and bilinear interpolation
    # independently predictable without importing OpenCV in the test process.
    pixels = bytes(x + 4 * y for y in range(HEIGHT) for x in range(WIDTH))
    stamps = [EPOCH - 100, EPOCH + 123, EPOCH + 1000000123, EPOCH + 2000000123]
    for camera_name in ("cam0", "cam1"):
        data = mav / camera_name / "data"
        data.mkdir()
        rows = []
        for index, stamp in enumerate(stamps):
            filename = f"{index}.pgm"
            (data / filename).write_bytes(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode() + pixels)
            rows.append([stamp, filename])
        write_rows(mav / camera_name / "data.csv", rows)
    write_rows(mav / "imu0/data.csv", [[EPOCH, 1, 2, 3, 4, 5, 6],
                                       [EPOCH + 1000000001, -1, -2, -3, -4, -5, -6]])
    q = math.sqrt(0.5)
    write_rows(mav / "vicon0/data.csv", [[EPOCH, 10, 20, 30, q, 0, 0, q],
                                         [EPOCH + 2000000000, 10, 20, 30, q, 0, 0, q]])
    return mav


def run_converter(mav, out):
    env = os.environ.copy()
    system_packages = "/usr/lib/python3/dist-packages"
    if Path(system_packages).is_dir():
        env["PYTHONPATH"] = system_packages + os.pathsep + env.get("PYTHONPATH", "")
    return subprocess.run([sys.executable, str(SCRIPT), str(mav), str(out)],
                          capture_output=True, text=True, env=env, check=False)


class EurocConverterTest(unittest.TestCase):
    def assertVectorAlmostEqual(self, actual, expected, places=6):
        self.assertEqual(len(actual), len(expected))
        for value, target in zip(actual, expected):
            self.assertAlmostEqual(float(value), target, places=places)

    def test_calibrated_conversion_and_time_association(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mav = fixture(root)
            out = root / "converted"
            process = run_converter(mav, out)
            self.assertEqual(process.returncode, 0, process.stderr)
            calibration = json.loads((out / "calibration.json").read_text(encoding="utf-8"))
            self.assertEqual(calibration["time_origin_ns"], str(EPOCH))
            self.assertEqual(calibration["reference"], "raw_vicon0_to_imu")
            self.assertEqual((calibration["stereo_pairs"], calibration["imu_samples"],
                              calibration["reference_samples"]), (2, 2, 2))
            self.assertAlmostEqual(calibration["baseline_m"], 0.2)
            self.assertVectorAlmostEqual(calibration["left_camera_position_body_frd_m"], [1, 0.5, -2])
            self.assertVectorAlmostEqual(calibration["right_camera_position_body_frd_m"], [1, 0.7, -2])
            # Quaternion sign is arbitrary, so compare its rotation via the outer product.
            q = calibration["left_camera_to_body_frd_wxyz"]
            expected = [0, math.sqrt(0.5), math.sqrt(0.5), 0]
            for i in range(4):
                for j in range(4):
                    self.assertAlmostEqual(q[i] * q[j], expected[i] * expected[j], places=6)
            imu = read_rows(out / "imu_px4.csv")
            self.assertEqual([row["stamp_s"] for row in imu], ["0.000000000", "1.000000001"])
            self.assertVectorAlmostEqual([imu[0][key] for key in ("gx", "gy", "gz", "ax", "ay", "az")],
                                         [1, -2, -3, 4, -5, -6])
            truth = read_rows(out / "truth.csv")
            self.assertEqual([row["stamp_s"] for row in truth], ["0.000000000", "2.000000000"])
            self.assertVectorAlmostEqual([truth[0][key] for key in ("x", "y", "z")], [10, 20.5, 31])
            truth_q = [float(truth[0][key]) for key in ("qw", "qx", "qy", "qz")]
            self.assertAlmostEqual(abs(truth_q[3]), 1)
            self.assertVectorAlmostEqual(truth_q[:3], [0, 0, 0])
            frames = read_rows(out / "frames.csv")
            self.assertEqual([(r["side"], r["stamp_s"], r["byte_offset"]) for r in frames],
                             [("left", "0.000000123", "0"), ("right", "0.000000123", "0"),
                              ("left", "1.000000123", str(WIDTH * HEIGHT)),
                              ("right", "1.000000123", str(WIDTH * HEIGHT))])
            for side in ("left", "right"):
                image = (out / f"{side}.bin").read_bytes()
                self.assertEqual(len(image), 2 * WIDTH * HEIGHT)
                self.assertEqual(image[:WIDTH * HEIGHT], image[WIDTH * HEIGHT:])
                # Rectified pixel (8, 7) maps through the radial model to the
                # source ramp. OpenCV quantizes interpolation to 1/32 pixel.
                x, y = 8, 7
                nx = (x - calibration["principal_x_px"]) / calibration["focal_px"]
                ny = (y - calibration["principal_y_px"]) / calibration["focal_px"]
                radial = 1 + 0.2 * (nx * nx + ny * ny)
                sx, sy = 24 * nx * radial + 15.5, 24 * ny * radial + 11.5
                self.assertGreater(abs((sx + 4 * sy) - (x + 4 * y)), 0.5)
                self.assertAlmostEqual(image[y * WIDTH + x], sx + 4 * sy, delta=1)

    def test_rejects_invalid_and_mismatched_inputs(self):
        def set_sensor(mav, name, key, value):
            path = mav / name / "sensor.yaml"
            sensor = json.loads(path.read_text(encoding="utf-8"))
            sensor[key] = value
            path.write_text(json.dumps(sensor), encoding="utf-8")

        cases = [
            ("mismatched timestamps", lambda m: write_rows(m / "cam1/data.csv", [[EPOCH + 123, "1.pgm"], [EPOCH + 1000000124, "2.pgm"]])),
            ("duplicate timestamps", lambda m: write_rows(m / "imu0/data.csv", [[EPOCH, 1, 2, 3, 4, 5, 6], [EPOCH, 1, 2, 3, 4, 5, 6]])),
            ("resolution mismatch", lambda m: set_sensor(m, "cam1", "resolution", [31, 24])),
            ("invalid rotation", lambda m: set_sensor(m, "imu0", "T_BS", transform([[2, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0]))),
            ("unsupported camera", lambda m: set_sensor(m, "cam0", "distortion_model", "equidistant")),
            ("no reference overlap", lambda m: write_rows(m / "vicon0/data.csv", [[EPOCH + 3000000000, 0, 0, 0, 1, 0, 0, 0], [EPOCH + 4000000000, 0, 0, 0, 1, 0, 0, 0]])),
            ("missing image", lambda m: (m / "cam0/data/1.pgm").unlink()),
        ]
        for name, mutate in cases:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                mav = fixture(root)
                mutate(mav)
                process = run_converter(mav, root / "converted")
                self.assertNotEqual(process.returncode, 0)


if __name__ == "__main__":
    unittest.main()
