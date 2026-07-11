# SPDX-License-Identifier: MIT
"""Repo-hygiene check: the old CLI name must not creep back into docs.

The host package was published as ``openbricks-dev`` before the
unification; the command and PyPI name are now plain ``openbricks``.
Stale references linger in docstrings and guides for months because
nothing executes them — this test greps the user-facing surfaces so a
regression fails CI instead of confusing a reader.

Also catches ``pybricks-dev``: the Pybricks host tool we compare
ourselves to is spelled ``pybricksdev`` (one word), and the hyphenated
misspelling has slipped into docstrings before.

Skipped when the repo layout isn't present (running from an installed
sdist rather than a checkout).
"""

import pathlib
import re
import unittest

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]

# Surfaces that must be clean. The legacy ``tools/openbricks-dev``
# package, CHANGELOGs, and ``scripts/bump-version.py`` deliberately
# keep historical mentions and are not scanned.
_SCAN = [
    "README.md",
    "CONTRIBUTING.md",
    "openbricks",
    "tools/openbricks/README.md",
    "tools/openbricks/openbricks_dev",
    "docs",
    "examples",
    "native/frozen",
    "native/boards",
    "tests",
]
_EXTS = {".py", ".md", ".rst", ".toml", ".html"}
_EXCLUDE_DIR_NAMES = {"_build", "datasheets", "__pycache__"}

# A line may mention the old name when it's explicitly historical
# (deprecation notes) or names the codecov flag, which kept the old
# spelling.
_ALLOWED_SUBSTRINGS = ("codecov", "previously published", "frozen", "legacy")


def _iter_files():
    for rel in _SCAN:
        p = _REPO_ROOT / rel
        if p.is_file():
            yield p
        elif p.is_dir():
            for f in sorted(p.rglob("*")):
                if (f.is_file() and f.suffix in _EXTS
                        and not (_EXCLUDE_DIR_NAMES
                                 & set(part.name for part in f.parents))):
                    yield f


class NoStaleNamesTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not (_REPO_ROOT / "openbricks" / "__init__.py").is_file():
            raise unittest.SkipTest("not running from a repo checkout")

    def _find(self, needle):
        hits = []
        for f in _iter_files():
            try:
                text = f.read_text(encoding="utf-8")
            except UnicodeDecodeError:
                continue
            for i, line in enumerate(text.splitlines(), 1):
                if needle in line and not any(
                        a in line for a in _ALLOWED_SUBSTRINGS):
                    hits.append("{}:{}: {}".format(
                        f.relative_to(_REPO_ROOT), i, line.strip()))
        return hits

    def test_no_openbricks_dev_references(self):
        hits = self._find("openbricks-dev")
        self.assertEqual(hits, [],
                         "stale ``openbricks-dev`` references (the CLI is "
                         "``openbricks`` now):\n" + "\n".join(hits))

    def test_no_pybricks_dev_misspelling(self):
        hits = self._find("pybricks-dev")
        self.assertEqual(hits, [],
                         "``pybricks-dev`` is a misspelling of the Pybricks "
                         "tool ``pybricksdev``:\n" + "\n".join(hits))

    def test_no_stale_cli_invocations_in_workflows(self):
        """Workflows may legitimately keep the old *package/tag* name
        (deprecation job, tag namespaces, codecov flag) — but any
        ``openbricks-dev <subcommand>`` string is a user-facing CLI
        instruction and must use the current name. Caught the rolling
        release body telling users to run ``openbricks-dev flash``."""
        pattern = re.compile(
            r"openbricks-dev\s+(flash|list|run|upload|stop|log|download|sim)\b")
        workflows = _REPO_ROOT / ".github" / "workflows"
        hits = []
        for f in sorted(workflows.glob("*.y*ml")):
            for i, line in enumerate(
                    f.read_text(encoding="utf-8").splitlines(), 1):
                if pattern.search(line):
                    hits.append("{}:{}: {}".format(
                        f.relative_to(_REPO_ROOT), i, line.strip()))
        self.assertEqual(hits, [],
                         "workflows instruct users to run the retired "
                         "``openbricks-dev`` CLI:\n" + "\n".join(hits))


if __name__ == "__main__":
    unittest.main()
