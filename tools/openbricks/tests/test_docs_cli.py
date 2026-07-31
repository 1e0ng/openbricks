# SPDX-License-Identifier: MIT
"""Tests for ``openbricks docs`` — the offline documentation reader."""

import io
import unittest
from unittest import mock

from openbricks_dev import docs as docs_mod
from openbricks_dev.cli import main as cli_main


def _run_cli(argv):
    """Run the real CLI entry point, capturing stdout."""
    buf = io.StringIO()
    with mock.patch("sys.stdout", buf):
        rc = cli_main(argv)
    return rc, buf.getvalue()


class TopicResolutionTests(unittest.TestCase):
    def test_checkout_docs_dir_is_found(self):
        # Running from the repo (as tests do), the repo-root docs/
        # must resolve — the wheel path bundles _docs/ instead.
        d = docs_mod._docs_dir()
        self.assertTrue(d.endswith("_docs") or d.endswith("docs"))

    def test_every_listed_topic_loads(self):
        # The curated TOPICS list must stay in lockstep with the
        # actual pages (and with setup.py's _DOC_PAGES bundle list —
        # see test_bundle_list_matches_topics).
        for topic in docs_mod.TOPICS:
            text = docs_mod._load(topic)
            self.assertTrue(text.strip(), "empty page: %s" % topic)

    def test_unknown_topic_error_names_the_valid_ones(self):
        with self.assertRaises(docs_mod.DocsError) as ctx:
            docs_mod._load("no-such-page")
        msg = str(ctx.exception)
        self.assertIn("no-such-page", msg)
        for topic in docs_mod.TOPICS:
            self.assertIn(topic, msg)

    def test_bundle_list_matches_topics(self):
        # setup.py bundles _DOC_PAGES into the wheel; docs.py reads
        # TOPICS. If they drift, an installed wheel 404s on a topic
        # that works in the checkout. Parse setup.py's literal list
        # rather than importing it (it runs a build-time sync on
        # import).
        import ast
        import os
        setup_py = os.path.join(
            os.path.dirname(os.path.abspath(docs_mod.__file__)),
            "..", "setup.py")
        tree = ast.parse(open(setup_py).read())
        pages = None
        for node in ast.walk(tree):
            if (isinstance(node, ast.Assign)
                    and getattr(node.targets[0], "id", "") == "_DOC_PAGES"):
                pages = [c.value for c in node.value.elts]
        self.assertEqual(
            sorted(pages), sorted(t + ".md" for t in docs_mod.TOPICS))


class CleanupTests(unittest.TestCase):
    def test_front_matter_is_stripped(self):
        # hardware.md carries a myst front-matter block for SEO meta;
        # the terminal reader must not show it.
        text = docs_mod._load("hardware")
        self.assertFalse(text.startswith("---"))
        self.assertNotIn("html_meta", text)
        self.assertIn("# Hardware guide", text)

    def test_role_with_visible_text_keeps_only_the_text(self):
        cleaned = docs_mod._clean(
            "see the {doc}`MuJoCo-backed simulator <simulator>` page")
        self.assertEqual(cleaned, "see the MuJoCo-backed simulator page")

    def test_bare_role_keeps_code_styling(self):
        cleaned = docs_mod._clean(
            "raises {class}`openbricks.pins.ReservedPinError` naming")
        self.assertEqual(
            cleaned, "raises `openbricks.pins.ReservedPinError` naming")

    def test_plain_markdown_is_untouched(self):
        s = "code `x = {1: 2}` and **bold** stay as-is"
        self.assertEqual(docs_mod._clean(s), s)


class CliWiringTests(unittest.TestCase):
    def test_no_topic_lists_all_topics(self):
        rc, out = _run_cli(["docs"])
        self.assertEqual(rc, 0)
        for topic in docs_mod.TOPICS:
            self.assertIn(topic, out)
        self.assertIn("docs.openbricks.dev", out)

    def test_topic_prints_the_page(self):
        rc, out = _run_cli(["docs", "install"])
        self.assertEqual(rc, 0)
        self.assertIn("# Installation", out)
        self.assertIn("openbricks flash --name", out)

    def test_doc_alias_works(self):
        rc, out = _run_cli(["doc", "cli"])
        self.assertEqual(rc, 0)
        self.assertIn("# Command-line tool", out)

    def test_unknown_topic_is_a_clean_cli_error(self):
        err = io.StringIO()
        with mock.patch("sys.stderr", err):
            rc, _ = _run_cli(["docs", "nope"])
        self.assertEqual(rc, 1)
        self.assertIn("error:", err.getvalue())
        self.assertIn("nope", err.getvalue())


if __name__ == "__main__":
    unittest.main()
