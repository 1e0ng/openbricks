# SPDX-License-Identifier: MIT
"""``openbricks docs`` opens the real manual, offline.

It used to bundle the nine hand-written ``.md`` guides and re-render
them with a markdown library, so everything generated from docstrings
— the whole API reference — was absent from the CLI. It now ships the
SAME Sphinx build as docs.openbricks.dev as a single archive and opens
that, making parity structural rather than maintained.

These tests own the properties that make the claim true: the API
pages are there, topics resolve to real files, and nothing in the
bundle reaches for the network.
"""

import os
import unittest
import zipfile

from openbricks_dev import docs


class BundleContentTests(unittest.TestCase):
    def setUp(self):
        self.path = docs._bundle_path()

    def _names(self):
        with zipfile.ZipFile(self.path) as z:
            return z.namelist()

    def test_the_api_reference_is_present(self):
        # THE regression this change exists to fix.
        names = self._names()
        for page in ("robotics", "drivers_motors", "interfaces"):
            self.assertIn("api/%s.html" % page, names,
                          "api/%s.html missing" % page)

    def test_the_guides_are_present_too(self):
        names = self._names()
        for page in ("index", "install", "hardware", "cli"):
            self.assertIn("%s.html" % page, names)

    def test_autodoc_content_actually_made_it_in(self):
        # A page can exist and still be empty if automodule quietly
        # failed — which would restore the old gap while looking fixed.
        with zipfile.ZipFile(self.path) as z:
            body = z.read("api/robotics.html").decode("utf-8", "replace")
        self.assertIn("move_wheels", body)
        self.assertIn("check_motors", body)

    def test_nothing_in_the_bundle_fetches_from_the_network(self):
        # "Offline" must mean it renders with the network unplugged.
        # Remote LINKS are fine (clicked, not loaded); remote
        # stylesheets, scripts and fonts are not.
        bad = []
        with zipfile.ZipFile(self.path) as z:
            for name in z.namelist():
                if not name.endswith((".html", ".css")):
                    continue
                text = z.read(name).decode("utf-8", "replace")
                for marker in ('src="http', "@import url(http",
                               'rel="stylesheet" href="http'):
                    if marker in text:
                        bad.append((name, marker))
        self.assertEqual(bad, [], "remote assets would break offline use")

    def test_the_bundle_stays_small(self):
        # The design rests on this being cheap to ship: raw Sphinx
        # output is 19 MB, mostly build cache and five font formats
        # nothing reads. A regression means the trimming in
        # scripts/build-offline-docs.sh stopped working.
        mb = os.path.getsize(self.path) / (1024 * 1024.0)
        self.assertLess(mb, 1.0, "bundle grew to %.1f MB" % mb)


class TopicResolutionTests(unittest.TestCase):
    def test_no_topic_opens_the_index(self):
        self.assertEqual(docs._page_for(None), "index.html")

    def test_a_guide_resolves_at_the_top_level(self):
        self.assertEqual(docs._page_for("install"), "install.html")

    def test_an_api_page_resolves_under_api(self):
        # Users should not need to know which kind of page it is.
        self.assertEqual(docs._page_for("robotics"), "api/robotics.html")

    def test_an_unknown_topic_says_how_to_find_it(self):
        try:
            docs._page_for("no-such-page")
            self.fail("expected DocsError")
        except docs.DocsError as e:
            self.assertIn("no page", str(e))


class RunTests(unittest.TestCase):
    class _Args:
        def __init__(self, topic=None):
            self.topic = topic

    def setUp(self):
        self.opened = []
        self._real = docs.webbrowser.open
        docs.webbrowser.open = lambda url: (self.opened.append(url), True)[1]

    def tearDown(self):
        docs.webbrowser.open = self._real

    def test_run_opens_a_local_file_url(self):
        self.assertEqual(docs.run(self._Args()), 0)
        self.assertTrue(self.opened[0].startswith("file://"), self.opened)
        self.assertTrue(self.opened[0].endswith("index.html"), self.opened)

    def test_run_opens_the_requested_api_page(self):
        docs.run(self._Args("robotics"))
        self.assertTrue(self.opened[0].endswith("api/robotics.html"),
                        self.opened)

    def test_a_headless_session_is_told_where_the_files_are(self):
        docs.webbrowser.open = lambda url: False
        try:
            docs.run(self._Args())
            self.fail("expected DocsError")
        except docs.DocsError as e:
            # Useless advice would be "install a browser"; the useful
            # thing is the path it already extracted to.
            self.assertIn("extracted at", str(e))


class BundleFailureTests(unittest.TestCase):
    """The two ways the bundle itself can be broken, each with its
    own remedy in the message — a missing or corrupt data file must
    never surface as a bare traceback."""

    def test_missing_bundle_names_the_build_script(self):
        real = docs._BUNDLE
        docs._BUNDLE = "no-such-bundle.zip"
        try:
            docs._bundle_path()
            self.fail("expected DocsError")
        except docs.DocsError as e:
            self.assertIn("scripts/build-offline-docs.sh", str(e))
        finally:
            docs._BUNDLE = real

    def test_a_stale_partial_extraction_is_replaced(self):
        # The pre-1.65.2 failure: an interrupted extraction left a
        # half-manual (no index.html) that every later invocation
        # reused forever. The atomic path must replace it.
        import hashlib
        import tempfile as tf
        src = docs._bundle_path()
        with open(src, "rb") as f:
            tag = hashlib.sha256(f.read()).hexdigest()[:12]
        with tf.TemporaryDirectory() as tmp:
            real = docs.tempfile.gettempdir
            docs.tempfile.gettempdir = lambda: tmp
            try:
                partial = os.path.join(tmp, "openbricks-docs-" + tag)
                os.makedirs(partial)
                with open(os.path.join(partial, "install.html"), "w") as f:
                    f.write("<html>half</html>")   # no index.html
                out = docs._extract()
                self.assertTrue(
                    os.path.exists(os.path.join(out, "index.html")))
            finally:
                docs.tempfile.gettempdir = real

    def test_losing_the_extraction_race_reuses_the_winner(self):
        # A COMPLETE extraction already sits at the content-hash path
        # (another invocation won): _extract must reuse it as-is —
        # same hash means byte-identical content — and must not
        # replace or re-extract it.
        import hashlib
        import tempfile as tf
        with tf.TemporaryDirectory() as tmp:
            real_gettmp = docs.tempfile.gettempdir
            docs.tempfile.gettempdir = lambda: tmp
            try:
                with open(docs._bundle_path(), "rb") as f:
                    tag = hashlib.sha256(f.read()).hexdigest()[:12]
                done = os.path.join(tmp, "openbricks-docs-" + tag)
                os.makedirs(done)
                marker = os.path.join(done, "index.html")
                with open(marker, "w") as f:
                    f.write("<html>winner</html>")
                self.assertEqual(docs._extract(), done)
                with open(marker) as f:            # untouched
                    self.assertIn("winner", f.read())
            finally:
                docs.tempfile.gettempdir = real_gettmp

    def test_race_lost_at_the_rename_reuses_the_winner(self):
        # The winner appears BETWEEN our extraction and our rename:
        # the rename fails, the winner is complete, our scratch copy
        # is discarded and the winner's dir is returned untouched.
        import hashlib
        import tempfile as tf
        with tf.TemporaryDirectory() as tmp:
            real_gettmp = docs.tempfile.gettempdir
            real_rename = docs.os.rename
            docs.tempfile.gettempdir = lambda: tmp
            with open(docs._bundle_path(), "rb") as f:
                tag = hashlib.sha256(f.read()).hexdigest()[:12]
            done = os.path.join(tmp, "openbricks-docs-" + tag)

            def winner_appears(a, b):
                os.makedirs(done)
                with open(os.path.join(done, "index.html"), "w") as f:
                    f.write("<html>winner</html>")
                raise OSError("target exists")
            docs.os.rename = winner_appears
            try:
                self.assertEqual(docs._extract(), done)
                with open(os.path.join(done, "index.html")) as f:
                    self.assertIn("winner", f.read())
            finally:
                docs.os.rename = real_rename
                docs.tempfile.gettempdir = real_gettmp

    def test_corrupt_bundle_without_index_is_named(self):
        import tempfile
        import zipfile
        with tempfile.TemporaryDirectory() as tmp:
            bad = os.path.join(tmp, "bad.zip")
            with zipfile.ZipFile(bad, "w") as z:
                z.writestr("not-index.html", "<html></html>")
            real = docs._bundle_path
            docs._bundle_path = lambda: bad
            try:
                docs._extract()
                self.fail("expected DocsError")
            except docs.DocsError as e:
                self.assertIn("corrupt", str(e))
            finally:
                docs._bundle_path = real


if __name__ == "__main__":
    unittest.main()
