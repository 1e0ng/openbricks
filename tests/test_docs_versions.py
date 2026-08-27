# SPDX-License-Identifier: MIT
"""Tests for scripts/docs-versions.py — the versions.json generator
behind the docs site's version switcher.

CPython-only (ci.yaml CPython list): the script under test drives the
docs deploy, which never runs under MicroPython.
"""

import json
import os
import shutil
import tempfile
import unittest
from importlib.machinery import SourceFileLoader

_SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts", "docs-versions.py")
docs_versions = SourceFileLoader("docs_versions", _SCRIPT).load_module()


class VersionKeyTests(unittest.TestCase):

    def test_numeric_not_lexicographic(self):
        ordered = sorted(["2.9.0", "2.10.0", "3.0.1", "2.8.2"],
                         key=docs_versions.version_key, reverse=True)
        self.assertEqual(ordered, ["3.0.1", "2.10.0", "2.9.0", "2.8.2"])


class CollectVersionsTests(unittest.TestCase):

    def setUp(self):
        self.root = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.root)

    def _add(self, name, with_index=True):
        d = os.path.join(self.root, name)
        os.mkdir(d)
        if with_index:
            with open(os.path.join(d, "index.html"), "w") as f:
                f.write("<html></html>")

    def test_collects_newest_first(self):
        for v in ("2.9.0", "3.0.1", "2.10.0"):
            self._add(v)
        self.assertEqual(docs_versions.collect_versions(self.root),
                         ["3.0.1", "2.10.0", "2.9.0"])

    def test_ignores_non_version_entries(self):
        self._add("3.0.1")
        self._add("not-a-version")
        self._add("1.2")            # not X.Y.Z
        with open(os.path.join(self.root, "README.md"), "w") as f:
            f.write("archive branch readme")
        self.assertEqual(docs_versions.collect_versions(self.root),
                         ["3.0.1"])

    def test_incomplete_archive_raises(self):
        self._add("3.0.1")
        self._add("2.9.0", with_index=False)
        with self.assertRaises(RuntimeError) as ctx:
            docs_versions.collect_versions(self.root)
        self.assertIn("2.9.0", str(ctx.exception))

    def test_missing_archive_dir_raises(self):
        with self.assertRaises(FileNotFoundError):
            docs_versions.collect_versions(
                os.path.join(self.root, "absent"))

    def test_empty_archive_is_empty_list(self):
        # First-ever deploy: branch exists, nothing archived yet.
        self.assertEqual(docs_versions.collect_versions(self.root), [])


class BuildManifestTests(unittest.TestCase):

    def test_manifest_shape(self):
        manifest = docs_versions.build_manifest(
            ["3.0.1", "2.9.0"], "3.0.1")
        self.assertEqual(manifest["latest"], "3.0.1")
        self.assertEqual(manifest["versions"], [
            {"version": "3.0.1", "url": "/3.0.1/",
             "pdf": "/3.0.1/openbricks-docs.pdf"},
            {"version": "2.9.0", "url": "/2.9.0/",
             "pdf": "/2.9.0/openbricks-docs.pdf"},
        ])

    def test_manifest_is_json_serializable(self):
        manifest = docs_versions.build_manifest(["2.9.0"], "3.0.1")
        self.assertEqual(json.loads(json.dumps(manifest)), manifest)

    def test_bad_latest_raises(self):
        with self.assertRaises(ValueError):
            docs_versions.build_manifest([], "v3.0.1")

    def test_current_firmware_version_is_valid_latest(self):
        # The deploy passes openbricks.__version__ straight in; pin
        # that it stays an acceptable X.Y.Z.
        import openbricks
        manifest = docs_versions.build_manifest(
            [], openbricks.__version__)
        self.assertEqual(manifest["latest"], openbricks.__version__)


if __name__ == "__main__":
    unittest.main()
