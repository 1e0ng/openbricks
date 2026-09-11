# SPDX-License-Identifier: MIT
"""The Assembly Workbench: ``openbricks sim`` opens it in the browser.

One self-contained page (``index.html`` next to this module) with the
brick library embedded: LEGO Technic parts with exact LDraw geometry,
your own STL imports, components built from bricks, and the robot as
the top component, with mass, centre of mass and inertia computed at
every level. The page needs no network beyond the CDN-hosted three.js
and pako scripts it loads; the server here only hands the page out.
"""
import http.server
import json
import pathlib
import threading
import webbrowser

from openbricks_sim import bricks

try:
    from importlib.resources import files as _files
except ImportError:                     # pragma: no cover
    _files = None

_HERE = pathlib.Path(__file__).resolve().parent


def template():
    """The page template with ``__BUNDLE__`` / ``__DOC__`` placeholders."""
    if _files is not None:
        return (_files(__package__) / "index.html").read_text(encoding="utf-8")
    return (_HERE / "index.html").read_text(encoding="utf-8")    # pragma: no cover


def render_page(extra_bundles=(), doc=None):
    """The finished HTML: the shipped bundle (plus any extra bundles
    from ``openbricks bricks convert``) and, optionally, an assembly
    document to open instead of the browser's saved draft."""
    html = template()
    if "__BUNDLE__" not in html or "__DOC__" not in html:
        raise RuntimeError("the workbench template is missing its placeholders")
    if extra_bundles:
        b64 = bricks.encode_bundle(bricks.merge_bundles(bricks.load_bundle(), extra_bundles))
    else:
        b64 = bricks.bundle_b64()
    doc_json = "null" if doc is None else json.dumps(doc)
    # the document lands inside a <script type="application/json"> block
    doc_json = doc_json.replace("</", "<\\/")
    return html.replace("__BUNDLE__", b64).replace("__DOC__", doc_json)


class _Handler(http.server.BaseHTTPRequestHandler):
    page = b""

    def do_GET(self):
        if self.path.split("?", 1)[0] not in ("/", "/index.html"):
            self.send_error(404, "the workbench is at /")
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(self.page)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(self.page)

    def log_message(self, fmt, *args):     # quiet: the terminal shows the URL, not every GET
        pass


def make_server(page, port=0, host="127.0.0.1"):
    """An HTTP server bound to ``host:port`` (0 = any free port) that
    serves ``page`` at ``/``."""
    handler = type("WorkbenchHandler", (_Handler,), {"page": page.encode("utf-8")})
    return http.server.ThreadingHTTPServer((host, port), handler)


def serve(page, port=0, open_browser=True, say=print, ready=None):
    """Serve the page until Ctrl-C. ``ready(url)`` is called once the
    socket is bound (tests use it to stop the server)."""
    server = make_server(page, port)
    url = "http://127.0.0.1:%d/" % server.server_address[1]
    say("Assembly Workbench at %s  (Ctrl-C stops it)" % url)
    if open_browser:
        threading.Thread(target=webbrowser.open, args=(url,), daemon=True).start()
    if ready is not None:
        ready(url, server)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        say("stopped")
    finally:
        server.server_close()
    return 0
