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

    def test_bump_script_hints_both_tags_lockstep(self):
        # One bump, two tags (lockstep since 1.15.0). The hint must
        # be two ``git tag`` invocations: ``git tag A B`` parses B as
        # a commit-ish, not a second tag (bit us cutting 1.15.0).
        self.assertIn("git tag v{v} && git tag cli/v{v}", self.bump)
        self.assertNotIn("git tag v{v} cli/v{v}", self.bump)
        self.assertNotIn("git tag openbricks/v{version}", self.bump)

    def test_release_artifact_glob_cannot_match_wheel_artifacts(self):
        # The 1000-asset regression (2026-08-04): the release job's
        # download glob ran openbricks-*-<version>, and on main
        # pushes <version> is "latest" — which ALSO matched the wheel
        # artifacts (openbricks-wheels-ubuntu-latest etc., the
        # runner-OS suffix collides). Every push dumped 16 versioned
        # wheels onto the rolling release until GitHub's per-release
        # asset cap failed the job on every push. The glob must stay
        # chip-prefixed, and must genuinely exclude the wheel names.
        import fnmatch
        marker = "pattern: openbricks-esp32*-${{ needs.firmware.outputs.version }}"
        self.assertIn(marker, self.ci)
        self.assertNotIn(
            "pattern: openbricks-*-${{ needs.firmware.outputs.version }}",
            self.ci)
        glob = "openbricks-esp32*-latest"     # main-push substitution
        for fw in ("openbricks-esp32-latest", "openbricks-esp32s3-latest"):
            self.assertTrue(fnmatch.fnmatch(fw, glob), fw)
        for whl in ("openbricks-wheels-ubuntu-latest",
                    "openbricks-wheels-macos-latest",
                    "openbricks-wheels-windows-latest"):
            self.assertFalse(fnmatch.fnmatch(whl, glob), whl)

    def test_firmware_and_cli_versions_match(self):
        # The lockstep pin, host-suite side (the firmware suite pins
        # it too — whichever CI job runs first catches a desync).
        def _ver(rel):
            with open(str(_REPO_ROOT / rel)) as f:
                for line in f:
                    if line.startswith("__version__"):
                        return line.split('"')[1]
        fw = _ver("openbricks/__init__.py")
        cli = _ver("tools/openbricks/openbricks_dev/__init__.py")
        self.assertEqual(fw, cli)


if __name__ == "__main__":
    unittest.main()
