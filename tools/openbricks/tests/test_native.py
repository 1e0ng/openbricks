# SPDX-License-Identifier: MIT
"""``openbricks sim``: the native launcher — platform tags, the cache,
download + signature verification, and the command line it builds."""
import io
import os
import tarfile
import tempfile
import unittest
import zipfile
from unittest import mock

from openbricks_dev import __version__
from openbricks_sim import native
from openbricks_sim import cli as sim_cli


class _Resp(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def _targz(files):
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as tf:
        for name, data in files.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tf.addfile(info, io.BytesIO(data))
    return buf.getvalue()


def _zip(files):
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        for name, data in files.items():
            zf.writestr(name, data)
    return buf.getvalue()


class PlatformTests(unittest.TestCase):
    def test_tags(self):
        self.assertEqual(native.platform_tag("darwin", "arm64"), "macos-arm64")
        self.assertEqual(native.platform_tag("darwin", "x86_64"), "macos-x86_64")
        self.assertEqual(native.platform_tag("linux", "x86_64"), "linux-x86_64")
        self.assertEqual(native.platform_tag("linux", "aarch64"), "linux-aarch64")
        self.assertEqual(native.platform_tag("win32", "AMD64"), "windows-x86_64")
        with self.assertRaises(RuntimeError):
            native.platform_tag("plan9", "mips")
        self.assertIn(native.platform_tag().split("-")[0], ("macos", "linux", "windows"))

    def test_asset_names_and_urls(self):
        self.assertEqual(native.asset_name("4.2.0", "macos-arm64"), "openbricks-sim-4.2.0-macos-arm64.tar.gz")
        self.assertEqual(native.asset_name("4.2.0", "windows-x86_64"), "openbricks-sim-4.2.0-windows-x86_64.zip")
        self.assertEqual(native.download_url("4.2.0", "x.tar.gz"), "https://github.com/1e0ng/openbricks/releases/download/v4.2.0/x.tar.gz")

    def test_cache_dir_and_binary_path(self):
        with mock.patch.dict(os.environ, {"OPENBRICKS_SIM_DIR": "/tmp/simcache"}):
            self.assertEqual(str(native.binary_path("4.2.0", "linux-x86_64")), os.path.join("/tmp", "simcache", "4.2.0", "openbricks-sim"))
            self.assertTrue(str(native.binary_path("4.2.0", "windows-x86_64")).endswith("openbricks-sim.exe"))
        with mock.patch.dict(os.environ, {"XDG_CACHE_HOME": "/tmp/xdg"}, clear=False):
            os.environ.pop("OPENBRICKS_SIM_DIR", None)
            self.assertEqual(str(native.cache_dir()), os.path.join("/tmp", "xdg", "openbricks", "sim"))


class EnsureBinaryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.env = mock.patch.dict(os.environ, {"OPENBRICKS_SIM_DIR": self.tmp.name})
        self.env.start()
        os.environ.pop("OPENBRICKS_SIM_BIN", None)

    def tearDown(self):
        self.env.stop()
        self.tmp.cleanup()

    def opener_for(self, archive, sig=b"sig"):
        calls = []

        def opener(url):
            calls.append(url)
            if url.endswith(".sig"):
                return _Resp(sig)
            return _Resp(archive)
        return opener, calls

    def test_env_override_wins(self):
        exe = os.path.join(self.tmp.name, "mine")
        with open(exe, "w") as fh:
            fh.write("#!/bin/sh\n")
        with mock.patch.dict(os.environ, {"OPENBRICKS_SIM_BIN": exe}):
            self.assertEqual(str(native.ensure_binary("4.2.0", "linux-x86_64", download=False)), exe)
        with mock.patch.dict(os.environ, {"OPENBRICKS_SIM_BIN": exe + "-missing"}):
            with self.assertRaises(RuntimeError):
                native.ensure_binary("4.2.0", "linux-x86_64", download=False)

    def test_downloads_verifies_and_unpacks_a_tarball(self):
        archive = _targz({"openbricks-sim-4.2.0-linux-x86_64/openbricks-sim": b"#!/bin/sh\necho sim\n"})
        opener, calls = self.opener_for(archive)
        said = []
        path = native.ensure_binary("4.2.0", "linux-x86_64", opener=opener, verify=lambda d, s: s == b"sig", progress=said.append)
        self.assertTrue(path.exists())
        self.assertTrue(os.access(path, os.X_OK))
        self.assertEqual(calls, [native.download_url("4.2.0", "openbricks-sim-4.2.0-linux-x86_64.tar.gz"), native.download_url("4.2.0", "openbricks-sim-4.2.0-linux-x86_64.tar.gz") + ".sig"])
        self.assertTrue(any("verified" in s for s in said))
        # cached now: no further download
        again = native.ensure_binary("4.2.0", "linux-x86_64", opener=opener, verify=lambda d, s: False)
        self.assertEqual(again, path)
        self.assertEqual(len(calls), 2)

    def test_windows_zip(self):
        archive = _zip({"openbricks-sim.exe": b"MZ"})
        opener, _ = self.opener_for(archive)
        path = native.ensure_binary("4.2.0", "windows-x86_64", opener=opener, verify=lambda d, s: True)
        self.assertTrue(str(path).endswith("openbricks-sim.exe"))
        self.assertTrue(path.exists())

    def test_bad_signature_is_refused(self):
        archive = _targz({"openbricks-sim": b"x"})
        opener, _ = self.opener_for(archive)
        with self.assertRaises(RuntimeError) as cm:
            native.ensure_binary("4.2.0", "linux-x86_64", opener=opener, verify=lambda d, s: False)
        self.assertIn("signature", str(cm.exception))
        self.assertFalse(native.binary_path("4.2.0", "linux-x86_64").exists())

    def test_archive_without_the_binary(self):
        archive = _targz({"README": b"nothing"})
        opener, _ = self.opener_for(archive)
        with self.assertRaises(RuntimeError) as cm:
            native.ensure_binary("4.2.0", "linux-x86_64", opener=opener, verify=lambda d, s: True)
        self.assertIn("did not contain", str(cm.exception))

    def test_offline_without_a_cache_explains(self):
        with self.assertRaises(RuntimeError) as cm:
            native.ensure_binary("4.2.0", "linux-x86_64", download=False)
        self.assertIn("OPENBRICKS_SIM_BIN", str(cm.exception))

    def test_network_failure_is_reported(self):
        def opener(url):
            raise OSError("no route")
        with self.assertRaises(RuntimeError) as cm:
            native.ensure_binary("4.2.0", "linux-x86_64", opener=opener)
        self.assertIn("could not download", str(cm.exception))

    def test_defaults_use_the_package_version(self):
        opener, calls = self.opener_for(_targz({"openbricks-sim": b"x"}))
        native.ensure_binary(tag="linux-x86_64", opener=opener, verify=lambda d, s: True)
        self.assertIn("/v%s/" % __version__, calls[0])


class LaunchTests(unittest.TestCase):
    def test_command_line(self):
        calls = []
        rc = native.launch("/x/openbricks-sim", ["/a.zlib", "/b.json"], file="/r.json", run=lambda cmd: calls.append(cmd) or 0, python="/py")
        self.assertEqual(rc, 0)
        self.assertEqual(calls, [["/x/openbricks-sim", "--python", "/py", "--bricks", "/a.zlib", "--bricks", "/b.json", "/r.json"]])
        import sys
        native.launch("/x/openbricks-sim", [], run=lambda cmd: calls.append(cmd) or 3)
        self.assertEqual(calls[-1], ["/x/openbricks-sim", "--python", sys.executable])


class SimCliTests(unittest.TestCase):
    def test_bare_sim_launches_the_app_with_the_shipped_bundle(self):
        calls = []
        with mock.patch("openbricks_sim.native.ensure_binary", return_value="/cache/openbricks-sim") as ensure, \
                mock.patch("openbricks_sim.native.launch", side_effect=lambda b, bundles, file=None, **kw: calls.append((b, [str(x) for x in bundles], file)) or 0):
            self.assertEqual(sim_cli.main([]), 0)
        ensure.assert_called_once()
        self.assertEqual(calls[0][0], "/cache/openbricks-sim")
        self.assertTrue(calls[0][1][0].endswith("technic_bundle.json.zlib"))
        self.assertIsNone(calls[0][2])

    def test_app_options(self):
        calls = []
        with mock.patch("openbricks_sim.native.ensure_binary", return_value="/cache/openbricks-sim") as ensure, \
                mock.patch("openbricks_sim.native.launch", side_effect=lambda b, bundles, file=None, **kw: calls.append((b, [str(x) for x in bundles], file)) or 0):
            rc = sim_cli.main(["app", "robot.assembly.json", "--bricks", "more.json", "--bin", "/my/sim", "--no-download"])
        self.assertEqual(rc, 0)
        self.assertEqual(ensure.call_count, 0)
        self.assertEqual(calls[0], ("/my/sim", [calls[0][1][0], "more.json"], "robot.assembly.json"))
        self.assertTrue(calls[0][1][0].endswith("technic_bundle.json.zlib"))

    def _app_calls(self):
        # Patches both native steps; returns (ensure mock, launch calls).
        calls = []
        return calls, mock.patch("openbricks_sim.native.ensure_binary", return_value="/cache/openbricks-sim"), \
            mock.patch("openbricks_sim.native.launch", side_effect=lambda b, bundles, file=None, **kw: calls.append((b, [str(x) for x in bundles], file)) or 0)

    def test_bare_forms_reach_the_app(self):
        # docs/simulator.md: ``openbricks sim [robot.assembly.json]
        # [--bricks more.json] [--bin PATH] [--no-download]``. These
        # exited 2 with argparse's "invalid choice" / "unrecognized
        # arguments" because the ``app`` fallback ran only after a
        # parse that had already rejected the token.
        calls, ensure_p, launch_p = self._app_calls()
        with ensure_p as ensure, launch_p:
            self.assertEqual(sim_cli.main(["robot.assembly.json"]), 0)
            self.assertEqual(sim_cli.main(["--bricks", "more.json"]), 0)
            self.assertEqual(sim_cli.main(["--bin", "/my/sim", "--no-download"]), 0)
            self.assertEqual(sim_cli.main(["robot.assembly.json", "--no-download", "--bin", "/my/sim", "--bricks", "a.json", "--bricks", "b.json"]), 0)
        self.assertEqual(ensure.call_count, 2)          # the two forms without --bin
        self.assertEqual(calls[0][0], "/cache/openbricks-sim")
        self.assertEqual(calls[0][2], "robot.assembly.json")
        self.assertEqual(calls[1][1][1:], ["more.json"])
        self.assertIsNone(calls[1][2])
        self.assertEqual(calls[2], ("/my/sim", [calls[2][1][0]], None))
        self.assertEqual(calls[3], ("/my/sim", [calls[3][1][0], "a.json", "b.json"], "robot.assembly.json"))
        for c in calls:
            self.assertTrue(c[1][0].endswith("technic_bundle.json.zlib"))

    def test_host_cli_forwards_the_bare_form(self):
        from openbricks_dev import cli as dev_cli
        calls, ensure_p, launch_p = self._app_calls()
        with ensure_p, launch_p:
            self.assertEqual(dev_cli.main(["sim", "robot.assembly.json"]), 0)
            self.assertEqual(dev_cli.main(["sim", "--bricks", "more.json"]), 0)
        self.assertEqual(calls[0][2], "robot.assembly.json")
        self.assertEqual(calls[1][1][1:], ["more.json"])

    def test_subcommands_are_still_commands_not_files(self):
        calls, ensure_p, launch_p = self._app_calls()
        with ensure_p, launch_p, \
                mock.patch("openbricks_sim.cli.cmd_workbench", return_value=0) as wb, \
                mock.patch("openbricks_sim.cli.cmd_run", return_value=0) as run, \
                mock.patch("openbricks_sim.cli.cmd_preview", return_value=0) as preview:
            self.assertEqual(sim_cli.main(["workbench"]), 0)
            self.assertEqual(sim_cli.main(["run", "main.py"]), 0)
            self.assertEqual(sim_cli.main(["preview", "--headless"]), 0)
        self.assertEqual(wb.call_args[0][0].command, "workbench")
        self.assertEqual(run.call_args[0][0].script, "main.py")
        self.assertTrue(preview.call_args[0][0].headless)
        self.assertEqual(calls, [])

    def test_top_level_help_and_version_stay_at_the_top(self):
        import contextlib
        for argv in (["--help"], ["-h"]):
            out = io.StringIO()
            with self.assertRaises(SystemExit) as ctx, contextlib.redirect_stdout(out):
                sim_cli.main(argv)
            self.assertEqual(ctx.exception.code, 0)
            self.assertIn("COMMAND", out.getvalue())       # the sim's own usage, not app's
            self.assertIn("workbench", out.getvalue())
        out = io.StringIO()
        with self.assertRaises(SystemExit) as ctx, contextlib.redirect_stdout(out):
            sim_cli.main(["--version"])
        self.assertEqual(ctx.exception.code, 0)
        self.assertIn(__version__, out.getvalue())

    def test_download_failure_is_a_clean_error(self):
        import contextlib
        err = io.StringIO()
        with mock.patch("openbricks_sim.native.ensure_binary", side_effect=RuntimeError("could not download the sim")), contextlib.redirect_stderr(err):
            self.assertEqual(sim_cli.main([]), 1)
        self.assertIn("could not download the sim", err.getvalue())
        self.assertIn("openbricks sim workbench", err.getvalue())
