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


# -- The job graph behind the two publishes ---------------------------------
#
# The release gates live in ``needs`` lists and ``if`` expressions that
# GitHub only evaluates on a real push, and a wrong one fails silently:
# from 4.2.0 on, ``sim-rs`` (which needs ``openbricks-host``) was skipped
# on every main push because ``openbricks-host`` skips there by design,
# so ``release`` skipped too and the rolling ``latest`` stopped
# publishing; and ``c-unit``/``cpython-tests``/``openbricks-py`` were in
# neither publish's ``needs``, so a red one on a tag shipped anyway.
# These tests parse the workflow and play the graph out for each event.

_TEST_JOBS = ("test", "c-unit", "cpython-tests", "openbricks-py")
_MAIN = ("push", "refs/heads/main")
_PR = ("pull_request", "refs/pull/1/merge")
_FW_TAG = ("push", "refs/tags/v9.9.9")
_CLI_TAG = ("push", "refs/tags/cli/v9.9.9")


def _load_jobs():
    # PyYAML is installed next to [dev] by the openbricks-host CI job.
    import yaml
    return yaml.safe_load(_CI_YAML.read_text())["jobs"]


def _needs(job):
    n = job.get("needs", [])
    return [n] if isinstance(n, str) else list(n)


def _to_python(expr):
    """Translate the GitHub-expression subset ci.yaml uses to Python."""
    import re
    e = str(expr).strip()
    if e.startswith("${{") and e.endswith("}}"):
        e = e[3:-2]
    e = re.sub(r"needs\.([A-Za-z0-9_-]+)\.result", r"needs_result('\1')", e)
    e = e.replace("&&", " and ").replace("||", " or ")
    e = re.sub(r"!(?!=)", " not ", e)
    e = (e.replace("github.event_name", "event_name")
          .replace("github.ref", "ref")
          .replace("runner.os", "runner_os"))
    return " ".join(e.split())


def _eval(expr, env):
    return bool(eval(_to_python(expr), {"__builtins__": {}}, env))


def _simulate(jobs, event, ref, fail=()):
    """Each job's result for one event: 'success', 'failure' or 'skipped'.

    Every job that runs succeeds unless named in ``fail``. A job ``if``
    without a status function gets GitHub's implicit ``success()``.
    """
    results = {}

    def result(name):
        if name in results:
            return results[name]
        job = jobs[name]
        needs = _needs(job)
        dep = {n: result(n) for n in needs}
        env = {
            "event_name": event, "ref": ref,
            "needs_result": lambda n: dep[n],
            "startsWith": lambda s, p: s.startswith(p),
            "always": lambda: True,
            "cancelled": lambda: False,
            "failure": lambda: any(r == "failure" for r in dep.values()),
            "success": lambda: all(r == "success" for r in dep.values()),
        }
        cond = job.get("if", "true")
        if cond is True or cond == "true":
            cond = "success()"
        elif not any(f in str(cond) for f in
                     ("always()", "success()", "failure()", "cancelled()")):
            cond = "success() && (%s)" % str(cond).replace("${{", "").replace("}}", "")
        runs = _eval(cond, env)
        results[name] = (("failure" if name in fail else "success")
                         if runs else "skipped")
        return results[name]

    for name in jobs:
        result(name)
    return results


def _step_runs(step, event, ref, runner_os="Linux"):
    cond = step.get("if")
    if cond is None:
        return True
    return _eval(cond, {"event_name": event, "ref": ref,
                        "runner_os": runner_os,
                        "startsWith": lambda s, p: s.startswith(p)})


class ReleaseGateTests(unittest.TestCase):
    def setUp(self):
        _skip_unless_checkout(self)
        self.jobs = _load_jobs()

    def test_release_needs_every_test_job(self):
        self.assertEqual(
            set(_needs(self.jobs["release"])),
            set(_TEST_JOBS) | {"firmware", "sim-rs"})

    def test_publish_needs_every_test_job_and_no_firmware(self):
        needs = set(_needs(self.jobs["publish-openbricks"]))
        self.assertEqual(
            needs,
            set(_TEST_JOBS) | {"build-openbricks-sdist",
                               "build-openbricks-wheels"})
        # Plain success semantics: a skipped test job blocks it too.
        self.assertNotIn("always()", str(self.jobs["publish-openbricks"]["if"]))
        self.assertNotIn("cancelled()", str(self.jobs["publish-openbricks"]["if"]))

    def test_release_tolerates_test_jobs_skipped_on_main(self):
        # The four test jobs skip on a main push by design, so release
        # may only reject failed/cancelled, never demand success.
        clauses = {c.strip() for c in
                   str(self.jobs["release"]["if"]).split("&&")}
        self.assertIn("always()", clauses)
        for job in _TEST_JOBS:
            self.assertIn("needs.%s.result != 'failure'" % job, clauses)
            self.assertIn("needs.%s.result != 'cancelled'" % job, clauses)
            self.assertNotIn("needs.%s.result == 'success'" % job, clauses)
        for job in ("firmware", "sim-rs"):
            self.assertIn("needs.%s.result == 'success'" % job, clauses)

    def test_sim_rs_runs_when_openbricks_host_skipped(self):
        cond = _to_python(self.jobs["sim-rs"]["if"])
        self.assertIn("not cancelled()", cond)
        self.assertIn("needs_result('openbricks-host') != 'failure'", cond)

    def test_main_push_publishes_the_rolling_release(self):
        r = _simulate(self.jobs, *_MAIN)
        self.assertEqual(r["release"], "success")
        self.assertEqual(r["sim-rs"], "success")
        self.assertEqual(r["firmware"], "success")
        # Doctrine #345: a main push re-runs no test job.
        for job in _TEST_JOBS + ("openbricks-host", "qemu-smoke"):
            self.assertEqual(r[job], "skipped", job)
        self.assertEqual(r["publish-openbricks"], "skipped")

    def test_main_push_sim_rs_builds_but_does_not_retest(self):
        steps = {s.get("name") or s["uses"]: s
                 for s in self.jobs["sim-rs"]["steps"]}
        for name in ("taiki-e/install-action@cargo-llvm-cov",
                     "actions/setup-python@v7",
                     "The runtime for the end-to-end test",
                     "Format, lint, test under coverage"):
            self.assertFalse(_step_runs(steps[name], *_MAIN), name)
            self.assertTrue(_step_runs(steps[name], *_PR), name)
            self.assertTrue(_step_runs(steps[name], *_FW_TAG), name)
        for name in ("Linux build dependencies (winit, wgpu, file dialogs)",
                     "Build", "Package", "Upload sim artifact"):
            self.assertTrue(_step_runs(steps[name], *_MAIN), name)

    def test_main_push_firmware_failure_blocks_the_rolling_release(self):
        for job in ("firmware", "sim-rs"):
            r = _simulate(self.jobs, *_MAIN, fail=(job,))
            self.assertEqual(r["release"], "skipped", job)

    def test_pull_request_publishes_nothing(self):
        r = _simulate(self.jobs, *_PR)
        for job in _TEST_JOBS + ("openbricks-host", "sim-rs"):
            self.assertEqual(r[job], "success", job)
        self.assertEqual(r["release"], "skipped")
        self.assertEqual(r["publish-openbricks"], "skipped")

    def test_firmware_tag_release_is_blocked_by_every_test_job(self):
        self.assertEqual(_simulate(self.jobs, *_FW_TAG)["release"], "success")
        for job in _TEST_JOBS + ("openbricks-host", "firmware", "sim-rs"):
            r = _simulate(self.jobs, *_FW_TAG, fail=(job,))
            self.assertEqual(r["release"], "skipped", job)
        # qemu-smoke is continue-on-error by choice: it reports only.
        self.assertNotIn("qemu-smoke", _needs(self.jobs["release"]))
        self.assertIs(self.jobs["qemu-smoke"]["continue-on-error"], True)

    def test_cli_tag_publish_is_blocked_by_every_test_job(self):
        r = _simulate(self.jobs, *_CLI_TAG)
        self.assertEqual(r["publish-openbricks"], "success")
        self.assertEqual(r["firmware"], "skipped")
        self.assertEqual(r["release"], "skipped")
        for job in _TEST_JOBS + ("openbricks-host", "build-openbricks-sdist",
                                 "build-openbricks-wheels"):
            r = _simulate(self.jobs, *_CLI_TAG, fail=(job,))
            self.assertEqual(r["publish-openbricks"], "skipped", job)


if __name__ == "__main__":
    unittest.main()
