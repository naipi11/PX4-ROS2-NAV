import importlib.util
from pathlib import Path
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[2] / "tools/stereo/evaluate_visibility.py"
spec = importlib.util.spec_from_file_location("evaluate_visibility", SCRIPT)
visibility = importlib.util.module_from_spec(spec)
spec.loader.exec_module(visibility)


def fixture():
    return {
        "left": np.zeros((2, 5), dtype=np.uint8),
        "right": np.zeros((2, 5), dtype=np.uint8),
        "reference_class": np.array([[0, 1, 1, 1, 1], [2, 2, 3, 3, 1]], dtype=np.uint8),
        "reference_depth_m": np.array([[np.nan, 1, 2, 4, 0.2], [2, 6, 8, 1, 1]], dtype=np.float32),
        "pred_kind": np.array([[1, 1, 2, 3, 0], [1, 2, 0, 1, 1]], dtype=np.uint8),
        "pred_depth_m": np.array([[10, 2, 2.5, np.nan, np.nan], [1, 6.1, np.nan, 1, 10]], dtype=np.float32),
        "sampled": np.array([[True, True, True, True, True], [True, True, True, True, False]]),
        "focal_px": np.array(100.0),
        "principal_x_px": np.array(2.0),
        "principal_y_px": np.array(1.0),
        "baseline_m": np.array(0.1),
    }


class VisibilityTest(unittest.TestCase):
    def test_metrics_and_bands(self):
        result = visibility.evaluate(fixture(), 0.25)
        self.assertEqual(result["coverage"], {"pixels": 10, "sampled": 9,
                                              "sampled_without_reference": 1,
                                              "sampled_with_reference": 8})
        one, two, three = (result["classes"][str(i)] for i in (1, 2, 3))
        self.assertEqual((one["references"], one["hits"], one["free_only"],
                          one["unobservable"], one["no_return"]), (4, 1, 1, 1, 1))
        self.assertEqual((one["hit_overreach"], one["free_only_overreach"]), (1, 1))
        self.assertEqual(one["absolute_depth_error_m"]["median"], 1.0)
        self.assertEqual(one["absolute_disparity_error_px"]["p99"], 5.0)
        self.assertEqual(one["below_0_3_m"]["references"], 1)
        self.assertEqual([band["references"] for band in one["depth_bands"]], [0, 1, 1, 1, 0, 0])
        self.assertEqual((two["references"], two["hit_overreach"], two["free_only_overreach"]), (2, 0, 0))
        self.assertEqual(two["absolute_depth_error_m"]["median"], 1.0)
        self.assertEqual(two["absolute_disparity_error_px"]["median"], 5.0)
        self.assertEqual(two["signed_depth_error_m"]["median"], -1.0)
        self.assertEqual(two["signed_disparity_error_px"]["median"], 5.0)
        self.assertEqual(one["signed_depth_error_m"]["median"], 1.0)
        self.assertEqual(one["signed_disparity_error_px"]["median"], -5.0)
        self.assertEqual((three["references"], three["hits"], three["no_return"]), (2, 1, 1))
        self.assertEqual(three["depth_bands"][-1]["references"], 1)
        empty = one["depth_bands"][0]
        self.assertEqual(empty["references"], 0)
        self.assertTrue(all(value is None for value in empty["absolute_depth_error_m"].values()))
        self.assertTrue(all(value is None for value in empty["absolute_disparity_error_px"].values()))

    def test_sample_mask_and_threshold_boundary(self):
        data = fixture()
        data["sampled"][:] = False
        result = visibility.evaluate(data)
        self.assertEqual(result["coverage"]["sampled"], 0)
        self.assertTrue(all(group["references"] == 0 for group in result["classes"].values()))
        data["sampled"][0, 2] = True
        data["pred_depth_m"][0, 2] = 2.25
        self.assertEqual(visibility.evaluate(data, 0.25)["classes"]["1"]["free_only_overreach"], 0)
        self.assertEqual(visibility.evaluate(data, 0.24)["classes"]["1"]["free_only_overreach"], 1)

    def test_invalid_contract(self):
        mutations = [
            ("right", np.zeros((1, 5), dtype=np.uint8)),
            ("left", np.zeros((2, 5), dtype=np.float32)),
            ("sampled", np.ones((2, 5), dtype=np.uint8)),
            ("reference_class", np.full((2, 5), 4, dtype=np.uint8)),
            ("pred_kind", np.full((2, 5), 4, dtype=np.uint8)),
            ("reference_depth_m", np.zeros((2, 5), dtype=np.float32)),
            ("pred_depth_m", np.zeros((2, 5), dtype=np.float32)),
            ("focal_px", np.array([100.0])),
            ("baseline_m", np.array(0.0)),
            ("principal_x_px", np.array(np.inf)),
        ]
        for name, value in mutations:
            with self.subTest(name=name):
                data = fixture()
                data[name] = value
                with self.assertRaises(ValueError):
                    visibility.evaluate(data)
        data = fixture()
        del data["right"]
        with self.assertRaises(ValueError):
            visibility.evaluate(data)
        for threshold in (-1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                visibility.evaluate(fixture(), threshold)



if __name__ == "__main__":
    unittest.main()
