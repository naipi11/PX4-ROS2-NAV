from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1]
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from deployment_baseline import SCHEMA, verify  # noqa: E402
from runtime_manifest import file_record  # noqa: E402


class BaselineIntegrityTest(unittest.TestCase):
    def create_snapshot(self, root: Path) -> Path:
        archive = root / "dependencies.tar.gz"
        archive.write_bytes(b"saved dependency payload")
        document = {
            "schema": SCHEMA,
            "repository": {"commit": "baseline-commit"},
            "artifacts": [file_record(archive, root)],
        }
        (root / "baseline.json").write_text(json.dumps(document))
        return archive

    def test_rejects_same_size_content_corruption(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            archive = self.create_snapshot(root)
            verify(root)
            original = archive.read_bytes()
            archive.write_bytes(bytes([original[0] ^ 1]) + original[1:])
            with self.assertRaisesRegex(ValueError, "artifact changed"):
                verify(root)

    def test_rejects_missing_dependency_archive(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            archive = self.create_snapshot(root)
            archive.unlink()
            with self.assertRaises(FileNotFoundError):
                verify(root)

    def test_rejects_schema_without_px4_build_downloads(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            self.create_snapshot(root)
            manifest = root / "baseline.json"
            document = json.loads(manifest.read_text())
            document["schema"] = "drone_city_nav_deployment_baseline_v1"
            manifest.write_text(json.dumps(document))
            with self.assertRaisesRegex(ValueError, "unsupported"):
                verify(root)


if __name__ == "__main__":
    unittest.main()
