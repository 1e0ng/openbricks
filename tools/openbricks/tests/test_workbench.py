# SPDX-License-Identifier: MIT
"""The Assembly Workbench page and server, and the ``openbricks sim
workbench`` command that opens it."""
import base64
import json
import os
import re
import shutil
import subprocess
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


# The page's own script, run under node against a stand-in DOM where every element is inert but
# the ones a scenario reads: the two embedded blocks, the file status, the toast, and the buttons it
# clicks. It prints what the file status says once the page has booted (and clicked).
_HARNESS = r"""
// usage: node harness.js PAGE.html SCENARIO_JSON
const fs = require("fs"), zlib = require("zlib");
const html = fs.readFileSync(process.argv[2], "utf8");
const sc = JSON.parse(process.argv[3]);
const grab = id => { const m = html.match(new RegExp('<script id="' + id + '"[^>]*>([\\s\\S]*?)</script>')); return m ? m[1] : ""; };
const main = html.match(/<script>\n([\s\S]*?)<\/script>/)[1];
function stub() {
  const store = {};
  const f = function () { return p; };
  const p = new Proxy(f, {
    get(t, k) {
      if (k in store) return store[k];
      if (k === Symbol.toPrimitive) return () => 0;
      if (k === Symbol.iterator) return function* () {};
      if (k === "then") return undefined;
      if (k === "length") return 0;
      if (k === "value" || k === "textContent" || k === "innerHTML" || k === "className") return "";
      if (k === "checked") return false;
      return p;
    },
    set(t, k, v) { store[k] = v; return true; },
    apply() { return p; },
  });
  return p;
}
const els = {};
const real = (id, text) => { const e = stub(); e.textContent = text; els[id] = e; return e; };
real("brick-bundle", sc.bundle === "none" ? "" : grab("brick-bundle"));
real("initial-doc", sc.doc === undefined ? grab("initial-doc") : JSON.stringify(sc.doc));
const keydowns = [], clicks = {};
for (const id of sc.click || []) { const e = real(id, ""); e.addEventListener = (ev, fn) => { if (ev === "click") clicks[id] = fn; }; }
global.document = new Proxy({}, { get(t, k) {
  if (k === "getElementById") return id => els[id] || (els[id] = stub());
  return stub();
} });
global.window = { addEventListener: (ev, fn) => { if (ev === "keydown") keydowns.push(fn); }, matchMedia: null, devicePixelRatio: 1 };
global.getComputedStyle = () => ({ getPropertyValue: () => "" });
global.MutationObserver = class { constructor() {} observe() {} };
global.ResizeObserver = class { constructor() {} observe() {} };
global.requestAnimationFrame = () => 0;
global.atob = s => Buffer.from(s, "base64").toString("binary");
const stored = sc.draft === undefined ? {} : { "openbricks-assembly-draft": JSON.stringify(sc.draft) };
global.localStorage = {
  getItem: k => (k in stored ? stored[k] : null),
  setItem: (k, v) => { if (sc.quota) { const e = new Error("the quota has been exceeded"); e.name = "QuotaExceededError"; throw e; } stored[k] = v; },
};
if (sc.pako) global.pako = { inflate: (bin, o) => zlib.inflateSync(Buffer.from(bin)).toString() };
try { eval(main); } catch (e) { console.log(JSON.stringify({ crash: String(e && e.stack || e) })); process.exit(0); }
for (const id of sc.click || []) clicks[id]();
const fileSt = els["file-st"], toast = els["toast"];
console.log(JSON.stringify({ fileSt: String(fileSt.textContent), fileClass: String(fileSt.className), toast: toast ? String(toast.textContent) : "",
  draft: stored["openbricks-assembly-draft"] ? JSON.parse(stored["openbricks-assembly-draft"]).robot.name : null }));
"""


def _boot(page, **scenario):
    """Run ``page`` under node with ``scenario`` (``pako``: give it the
    inflater; ``bundle="none"``; ``doc``/``draft``: what the page was
    given and what the browser kept; ``quota``: every draft save fails;
    ``click``: button ids to press after boot). Returns what the file
    status shows."""
    with tempfile.TemporaryDirectory() as tmp:
        with open(os.path.join(tmp, "page.html"), "w") as fh:
            fh.write(page)
        with open(os.path.join(tmp, "harness.js"), "w") as fh:
            fh.write(_HARNESS)
        out = subprocess.run(["node", os.path.join(tmp, "harness.js"), os.path.join(tmp, "page.html"),
                              json.dumps(scenario)], capture_output=True, text=True, timeout=60)
    if out.returncode:
        raise AssertionError(out.stderr)
    result = json.loads(out.stdout.strip().splitlines()[-1])
    if "crash" in result:
        raise AssertionError(result["crash"])
    return result


def _carried_part_doc():
    """A build that carries the record of a part the library lacks, as
    a part fetched by number travels: ``ldraw`` set, the mesh with it."""
    import base64 as b64
    import struct
    pos = struct.pack("<9h", 0, 0, 0, 100, 0, 0, 0, 100, 0)
    rec = {"name": "Fetched", "ldraw": "99999", "mass_g": 2.0, "source": "placeholder", "fetched": True,
           "mesh": {"verts": 3, "tris": 1, "pos": b64.b64encode(pos).decode(), "nrm": b64.b64encode(bytes(9)).decode(),
                    "idx": b64.b64encode(struct.pack("<3H", 0, 1, 2)).decode(), "idx32": False, "scale": 0.01},
           "bbox": [[0, 0, 0], [1, 1, 0]], "com": [0.3, 0.3, 0], "inertia_per_g": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}
    return {"format": "openbricks-assembly/1", "parts": {"fetched": rec},
            "components": {"robot": {"children": [{"name": "f", "part": "fetched", "pos": [0, 0, 0]}]}},
            "robot": {"root": "robot", "name": "carried"}}


@unittest.skipUnless(shutil.which("node"), "node runs the page's script")
class PageBootTests(unittest.TestCase):
    """What the page says when it cannot do what it was asked."""

    def test_a_file_that_does_not_validate_is_refused_out_loud_never_swapped_for_the_draft(self):
        good_draft = _carried_part_doc()
        good_draft["robot"]["name"] = "the draft"
        bad = {"format": "openbricks-assembly/1", "parts": {"a": {"mass_g": 0, "shapes": []}},
               "components": {"r": {"children": []}}, "robot": {"root": "r", "name": "mine"}}
        got = _boot(workbench.render_page(doc=bad), pako=True, draft=good_draft)
        self.assertEqual(got["fileClass"], "st bad")
        self.assertIn("was not opened: brick a needs mass_g > 0.", got["fileSt"])
        self.assertIn("The example is open instead", got["fileSt"])
        self.assertIn("was not opened", got["toast"])
        # a draft that will not open is said too
        got = _boot(workbench.render_page(), pako=True, draft={"format": "nope", "robot": {"name": "old"}})
        self.assertEqual(got["fileClass"], "st bad")
        self.assertIn("The draft this browser kept was not opened: format must be", got["fileSt"])
        # a good file opens, and so does a draft when no file is given
        self.assertEqual(_boot(workbench.render_page(doc=_carried_part_doc()), pako=True),
                         {"fileSt": "1 bricks, 1 components", "fileClass": "st", "toast": "", "draft": None})
        self.assertEqual(_boot(workbench.render_page(), pako=True, draft=good_draft)["fileSt"], "1 bricks, 1 components")

    def test_a_build_carrying_a_fetched_part_opens_where_the_library_lacks_it(self):
        got = _boot(workbench.render_page(doc=_carried_part_doc()), pako=True)
        self.assertEqual((got["fileSt"], got["fileClass"]), ("1 bricks, 1 components", "st"))

    def test_a_library_that_did_not_load_is_said_and_its_bricks_weigh_nothing(self):
        got = _boot(workbench.render_page())                                  # no pako: the CDN is out of reach
        self.assertEqual(got["fileClass"], "st bad")
        self.assertTrue(got["fileSt"].startswith("The brick library did not load: the pako script from cdnjs did not load"), got)
        self.assertIn("brick lego_32278 needs LDraw part 32278, which this library does not carry", got["fileSt"])
        got = _boot(workbench.render_page(), pako=True, bundle="none")
        self.assertTrue(got["fileSt"].startswith("The page carries no brick library"), got)
        self.assertEqual(_boot(workbench.render_page(), pako=True)["fileClass"], "st", "the shipped library loads")

    def test_a_draft_the_browser_will_not_keep_is_said(self):
        got = _boot(workbench.render_page(), pako=True, quota=True, click=["reset-example"])
        self.assertEqual(got["fileClass"], "st bad")
        self.assertIn("Could not keep a draft (QuotaExceededError: the quota has been exceeded)", got["fileSt"])
        got = _boot(workbench.render_page(), pako=True, click=["reset-example"])
        self.assertEqual((got["fileClass"], got["draft"]), ("st", "et"))      # the example robot, kept


class PageTextTests(unittest.TestCase):
    def test_browser_chords_pass_through_the_key_handler(self):
        page = workbench.render_page()
        handler = page[page.index('window.addEventListener("keydown"'):]
        guard = handler.index("if (e.metaKey || e.ctrlKey || e.altKey) return;")
        self.assertLess(handler.index('e.key.toLowerCase() === "d") { e.preventDefault(); duplicateSelection()'), guard)
        self.assertLess(guard, handler.index('if (e.key === "Escape")'))
        self.assertLess(guard, handler.index('e.key.toLowerCase() === "r"'))
        self.assertNotIn("shiftKey) return", handler[:guard + 60], "Shift+arrow is the 1 mm nudge")

    def test_the_page_draws_a_mesh_at_the_step_it_was_packed_at(self):
        self.assertTrue("pos[i] = pos16[i] * step;" in workbench.render_page())


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
