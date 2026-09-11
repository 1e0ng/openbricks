# SPDX-License-Identifier: MIT
"""The Assembly Workbench page and server, and the ``openbricks sim
workbench`` command that opens it."""
import base64
import json
import os
import re
import tempfile
import threading
import unittest
import urllib.request
import zlib
from unittest import mock

from openbricks_sim import bricks, workbench
from openbricks_sim import cli as sim_cli
from openbricks_dev import cli as dev_cli


def _embedded_bundle(page):
    m = re.search(r'id="brick-bundle" type="text/plain">([^<]*)</script>', page)
    return json.loads(zlib.decompress(base64.b64decode(m.group(1))).decode())


def _embedded_doc(page):
    m = re.search(r'id="initial-doc" type="application/json">(.*?)</script>', page, re.S)
    return json.loads(m.group(1).replace("<\\/", "</"))


class RenderTests(unittest.TestCase):
    def test_page_embeds_the_shipped_bundle(self):
        page = workbench.render_page()
        self.assertNotIn("__BUNDLE__", page)
        self.assertNotIn("__DOC__", page)
        self.assertIn("<title>Openbricks Assembly Workbench</title>", page)
        self.assertIn("ldraw.org", page)
        self.assertEqual(_embedded_bundle(page)["parts"].keys(), bricks.load_bundle()["parts"].keys())
        self.assertIsNone(_embedded_doc(page))

    def test_extra_bundles_and_a_document_are_embedded(self):
        extra = {"parts": {"9999": {"name": "Test", "mass_g": 1, "mesh": {}, "bbox": [[0, 0, 0], [1, 1, 1]], "com": [0, 0, 0], "inertia_per_g": [], "connectors": [], "volume_mm3": 1}}}
        doc = {"format": "openbricks-assembly/1", "robot": {"name": "x</script><b>"}}
        page = workbench.render_page(extra_bundles=[extra], doc=doc)
        self.assertIn("9999", _embedded_bundle(page)["parts"])
        self.assertEqual(_embedded_doc(page), doc)
        self.assertNotIn("x</script>", page)          # the closing tag never breaks out of the block

    def test_template_without_placeholders_is_refused(self):
        with mock.patch.object(workbench, "template", return_value="<title>x</title>"):
            with self.assertRaises(RuntimeError):
                workbench.render_page()


class ServerTests(unittest.TestCase):
    def test_serves_the_page_and_nothing_else(self):
        server = workbench.make_server("<title>t</title><p>hello", 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            url = "http://127.0.0.1:%d/" % server.server_address[1]
            with urllib.request.urlopen(url) as resp:
                self.assertEqual(resp.status, 200)
                self.assertTrue(resp.headers["Content-Type"].startswith("text/html"))
                self.assertIn(b"hello", resp.read())
            with self.assertRaises(urllib.error.HTTPError) as cm:
                urllib.request.urlopen(url + "other")
            self.assertEqual(cm.exception.code, 404)
        finally:
            server.shutdown()
            server.server_close()

    def test_serve_announces_the_url_and_stops_on_shutdown(self):
        said = []

        def ready(url, server):
            said.append(url)
            threading.Thread(target=server.shutdown).start()
        rc = workbench.serve("<title>t</title>", port=0, open_browser=False, say=said.append, ready=ready)
        self.assertEqual(rc, 0)
        self.assertTrue(any(s.startswith("http://127.0.0.1:") for s in said))
        self.assertTrue(any("Assembly Workbench at http://127.0.0.1:" in s for s in said))

    def test_serve_opens_the_browser_when_asked(self):
        opened = []

        def ready(url, server):
            threading.Thread(target=server.shutdown).start()
        with mock.patch("openbricks_sim.workbench.webbrowser.open", side_effect=opened.append):
            workbench.serve("<title>t</title>", port=0, open_browser=True, say=lambda s: None, ready=ready)
        # the browser is opened on a helper thread; give it a moment
        for _ in range(50):
            if opened:
                break
            threading.Event().wait(0.02)
        self.assertEqual(len(opened), 1)


class SimCliTests(unittest.TestCase):
    def test_workbench_command_opens_the_page(self):
        calls = []
        with mock.patch("openbricks_sim.workbench.serve", side_effect=lambda page, **kw: calls.append((page, kw)) or 0):
            self.assertEqual(sim_cli.main(["workbench"]), 0)
        page, kw = calls[0]
        self.assertEqual(kw, {"port": 0, "open_browser": True})
        self.assertIn("32278", _embedded_bundle(page)["parts"])

    def test_workbench_options_file_and_extra_bricks(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        doc_path = os.path.join(tmp.name, "robot.assembly.json")
        with open(doc_path, "w") as fh:
            json.dump({"format": "openbricks-assembly/1", "robot": {"name": "mine"}}, fh)
        extra_path = os.path.join(tmp.name, "more.json")
        with open(extra_path, "w") as fh:
            json.dump({"parts": {"4242": {"name": "Extra"}}}, fh)
        calls = []
        with mock.patch("openbricks_sim.workbench.serve", side_effect=lambda page, **kw: calls.append((page, kw)) or 0):
            rc = sim_cli.main(["workbench", doc_path, "--bricks", extra_path, "--port", "4321", "--no-browser"])
        self.assertEqual(rc, 0)
        page, kw = calls[0]
        self.assertEqual(kw, {"port": 4321, "open_browser": False})
        self.assertEqual(_embedded_doc(page)["robot"]["name"], "mine")
        self.assertIn("4242", _embedded_bundle(page)["parts"])

    def test_dev_cli_forwards_sim_workbench(self):
        calls = []
        with mock.patch("openbricks_sim.workbench.serve", side_effect=lambda page, **kw: calls.append(kw) or 0):
            self.assertEqual(dev_cli.main(["sim", "workbench", "--no-browser"]), 0)
        self.assertEqual(calls, [{"port": 0, "open_browser": False}])

    def test_other_sim_commands_still_parse(self):
        import contextlib
        import io
        with self.assertRaises(SystemExit) as cm, contextlib.redirect_stdout(io.StringIO()):
            sim_cli.main(["preview", "--help"])
        self.assertEqual(cm.exception.code, 0)


class ServeInterruptTests(unittest.TestCase):
    def test_ctrl_c_stops_the_server_cleanly(self):
        class Fake:
            server_address = ("127.0.0.1", 5)
            closed = False

            def serve_forever(self):
                raise KeyboardInterrupt

            def server_close(self):
                self.closed = True
        fake = Fake()
        said = []
        with mock.patch("openbricks_sim.workbench.make_server", return_value=fake):
            self.assertEqual(workbench.serve("<title>t</title>", port=5, open_browser=False, say=said.append), 0)
        self.assertIn("stopped", said)
        self.assertTrue(fake.closed)
