"""Behavioral tests for Middlebury reference preparation."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import cv2
import numpy as np


SCRIPT = Path(__file__).resolve().parents[2] / "tools/stereo/prepare_middlebury.py"


def write_pfm(path, values, scale):
    endian = "<f4" if scale < 0 else ">f4"
    with path.open("wb") as stream:
        stream.write(f"Pf\n{values.shape[1]} {values.shape[0]}\n{scale}\n".encode())
        stream.write(np.flipud(values).astype(endian).tobytes())


def fixture(root, scale):
    left = np.tile(np.arange(6, dtype=np.uint8) * 20, (2, 1))
    right = left.copy()
    mask = np.full((2, 6), 255, dtype=np.uint8)
    mask[0, 4] = 0
    mask[0, 3] = 128  # known but stereo-occluded
    for name, image in (("im0.png", left), ("im1.png", right), ("mask0nocc.png", mask)):
        if not cv2.imwrite(str(root / name), image):
            raise RuntimeError("could not write fixture")
    # Middlebury's PFM scale magnitude does not multiply disparity pixels.
    disparity = np.array([[0.5, 0.5, 0.5, 0.5, 0.5, np.inf],
                          [1.0, 0.0, np.nan, -0.5, 0.5, 0.5]], dtype=np.float32)
    write_pfm(root / "disp0GT.pfm", disparity, scale)
    (root / "calib.txt").write_text(
        "cam0=[8 0 2; 0 8 1; 0 0 1]\n"
        "cam1=[8 0 2.5; 0 8 1; 0 0 1]\n"
        "doffs=0.5\nbaseline=100\nwidth=6\nheight=2\n", encoding="utf-8")


def invoke(root):
    return subprocess.run([sys.executable, str(SCRIPT), "--scene", "synthetic",
                           "--left", str(root / "im0.png"), "--right", str(root / "im1.png"),
                           "--calibration", str(root / "calib.txt"),
                           "--disparity", str(root / "disp0GT.pfm"),
                           "--mask", str(root / "mask0nocc.png"),
                           "--output", str(root / "out.npz"), "--metadata", str(root / "meta.json")],
                          capture_output=True, text=True, check=False)


class MiddleburyPreparationTest(unittest.TestCase):
    def test_endianness_alignment_visibility_and_invalid(self):
        for scale in (-0.003922, 0.003922):
            with self.subTest(scale=scale), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                fixture(root, scale)
                result = invoke(root)
                self.assertEqual(result.returncode, 0, result.stderr)
                with np.load(root / "out.npz") as data:
                    np.testing.assert_array_equal(data["right"][0], [10, 30, 50, 70, 90, 50])
                    np.testing.assert_array_equal(data["reference_class"],
                                                  [[2, 1, 1, 2, 0, 0], [2, 1, 0, 0, 1, 1]])
                    self.assertAlmostEqual(float(data["reference_depth_m"][0, 2]), 0.8 / 1.0, places=6)
                    self.assertAlmostEqual(float(data["reference_depth_m"][1, 0]), 0.8 / 1.5, places=6)
                    self.assertTrue(np.isnan(data["reference_depth_m"][0, 5]))
                    self.assertTrue(np.isnan(data["reference_depth_m"][0, 4]))
                    self.assertTrue(np.isnan(data["reference_depth_m"][1, 2]))
                    self.assertEqual(float(data["principal_x_px"]), 2)
                    self.assertAlmostEqual(float(data["baseline_m"]), 0.1)
                metadata = json.loads((root / "meta.json").read_text(encoding="utf-8"))
                self.assertEqual(metadata["right_image_translation_px"], -0.5)
                self.assertNotEqual(invoke(root).returncode, 0)

    def test_output_cannot_overwrite_input(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fixture(root, -2.0)
            original = (root / "im0.png").read_bytes()
            result = subprocess.run([sys.executable, str(SCRIPT), "--scene", "s",
                                     "--left", str(root / "im0.png"), "--right", str(root / "im1.png"),
                                     "--calibration", str(root / "calib.txt"),
                                     "--disparity", str(root / "disp0GT.pfm"),
                                     "--mask", str(root / "mask0nocc.png"),
                                     "--output", str(root / "im0.png"), "--metadata", str(root / "meta.json")],
                                    capture_output=True, text=True, check=False)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual((root / "im0.png").read_bytes(), original)

    def test_rejects_unrepresentable_stereo_projection(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fixture(root, -0.003922)
            path = root / 'calib.txt'
            calibration = path.read_text(encoding='utf-8')
            path.write_text(calibration.replace('cam1=[8 0 2.5; 0 8 1;',
                                                'cam1=[8 0 2.5; 0 8 1.1;'), encoding='utf-8')
            result = invoke(root)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((root / 'out.npz').exists())


if __name__ == "__main__":
    unittest.main()
