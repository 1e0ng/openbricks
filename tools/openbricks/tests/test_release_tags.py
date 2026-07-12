# SPDX-License-Identifier: MIT
"""Pin the release-tag namespaces across the release plumbing.

The CLI/sim package publishes to PyPI on ``cli/v*`` tags (releases up
to 0.10.24 used ``openbricks/v*``); firmware releases use plain
``v*``. The tag pattern lives in three places that nothing executes
together — the CI workflow triggers, the job ``if`` conditions, and
``scripts/bump-version.py``'s tag hint — so a rename that misses one
produces a tag push that silently publishes nothing. This test greps
all three so the drift fails CI instead of a release.

Skipped when the repo layout isn't present (running from an installed
sdist rather than a checkout).
"""

import pathlib
import unittest

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_CI_YAML = _REPO_ROOT / ".github" / "workflows" / "ci.yaml"
_BUMP = _REPO_ROOT / "scripts" / "bump-version.py"


def _skip_unless_checkout(test):
    if not (_CI_YAML.exists() and _BUMP.exists()):
        raise unittest.SkipTest("repo checkout layout not present")
    return test


class ReleaseTagNamespaceTests(unittest.TestCase):
    def setUp(self):
        _skip_unless_checkout(self)
        self.ci = _CI_YAML.read_text()
        self.bump = _BUMP.read_text()

    def test_workflow_triggers_on_cli_tags(self):
        self.assertIn('- "cli/v*"', self.ci)
        self.assertIn('- "v*"', self.ci)

    def test_workflow_does_not_trigger_on_retired_namespace(self):
        self.assertNotIn('- "openbricks/v*"', self.ci)

    def test_publish_job_gated_on_cli_tags(self):
        self.assertIn("startsWith(github.ref, 'refs/tags/cli/v')", self.ci)

    def test_firmware_skip_conditions_cover_cli_tags(self):
        # Two jobs (firmware build, qemu smoke) skip host-tooling
        # tags; both must know the current namespace or a cli/v* tag
        # push wastes two ESP-IDF container builds per release.
        self.assertEqual(
            self.ci.count("!startsWith(github.ref, 'refs/tags/cli/')"), 2)
        # The retired namespace must not linger in job conditions
        # (a comment mentioning history is fine, an expression isn't).
        self.assertNotIn("'refs/tags/openbricks/'", self.ci)

    def test_bump_script_hints_cli_tag(self):
        self.assertIn("git tag cli/v{version}", self.bump)
        self.assertNotIn("git tag openbricks/v{version}", self.bump)


if __name__ == "__main__":
    unittest.main()
