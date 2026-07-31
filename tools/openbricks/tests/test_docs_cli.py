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


class DirResolutionBranchTests(unittest.TestCase):
    """Pin BOTH lookup branches + the broken-install error — which
    branch runs organically depends on whether a build has synced
    ``_docs/`` locally, so each is forced via mocks."""

    def test_bundled_dir_wins_when_present(self):
        with mock.patch("os.path.isdir", side_effect=lambda p: True):
            self.assertTrue(docs_mod._docs_dir().endswith("_docs"))

    def test_checkout_dir_used_when_no_bundle(self):
        with mock.patch("os.path.isdir",
                        side_effect=lambda p: not p.endswith("_docs")):
            self.assertTrue(docs_mod._docs_dir().endswith("docs"))

    def test_neither_location_is_a_clean_error(self):
        with mock.patch("os.path.isdir", side_effect=lambda p: False):
            with self.assertRaises(docs_mod.DocsError) as ctx:
                docs_mod._docs_dir()
        self.assertIn("broken installation", str(ctx.exception))


class EmitTests(unittest.TestCase):
    """The TTY/pager paths — never taken under a captured test
    stdout, so forced via mocks."""

    def _tty_stdout(self):
        out = io.StringIO()
        out.isatty = lambda: True
        return out

    def test_tty_uses_pager_with_the_text_as_input(self):
        out = self._tty_stdout()
        with mock.patch("sys.stdout", out), \
                mock.patch.dict("os.environ", {"PAGER": "mypager -x"}), \
                mock.patch("subprocess.run") as run:
            docs_mod._emit("hello page\n")
        run.assert_called_once_with(["mypager", "-x"],
                                    input=b"hello page\n")
        self.assertEqual(out.getvalue(), "")   # pager owned the output

    def test_missing_pager_binary_falls_through_to_plain_print(self):
        out = self._tty_stdout()
        with mock.patch("sys.stdout", out), \
                mock.patch("subprocess.run",
                           side_effect=FileNotFoundError):
            docs_mod._emit("hello page\n")
        self.assertEqual(out.getvalue(), "hello page\n")

    def test_text_without_trailing_newline_gets_one(self):
        out = io.StringIO()
        out.isatty = lambda: False
        with mock.patch("sys.stdout", out):
            docs_mod._emit("no newline")
        self.assertEqual(out.getvalue(), "no newline\n")


class TitleFallbackTests(unittest.TestCase):
    def test_page_without_heading_lists_under_its_stem(self):
        with mock.patch.object(docs_mod, "_load",
                               return_value="just prose, no heading"):
            self.assertEqual(docs_mod._title("mystery"), "mystery")


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
    def test_text_no_topic_lists_all_topics(self):
        rc, out = _run_cli(["docs", "--text"])
        self.assertEqual(rc, 0)
        for topic in docs_mod.TOPICS:
            self.assertIn(topic, out)
        self.assertIn("docs.openbricks.dev", out)

    def test_text_topic_prints_the_page(self):
        rc, out = _run_cli(["docs", "--text", "install"])
        self.assertEqual(rc, 0)
        self.assertIn("# Installation", out)
        self.assertIn("openbricks flash --name", out)

    def test_doc_alias_works(self):
        rc, out = _run_cli(["doc", "-t", "cli"])
        self.assertEqual(rc, 0)
        self.assertIn("# Command-line tool", out)

    def test_unknown_topic_is_a_clean_cli_error(self):
        err = io.StringIO()
        with mock.patch("sys.stderr", err), \
                mock.patch("webbrowser.open") as wb:
            rc, _ = _run_cli(["docs", "nope"])
        self.assertEqual(rc, 1)
        self.assertIn("error:", err.getvalue())
        self.assertIn("nope", err.getvalue())
        wb.assert_not_called()   # validated before any side effects


class BrowserModeTests(unittest.TestCase):
    """The default mode: render one styled offline HTML, open it."""

    def test_default_opens_browser_at_the_topic_anchor(self):
        with mock.patch("webbrowser.open", return_value=True) as wb:
            rc, out = _run_cli(["docs", "hardware"])
        self.assertEqual(rc, 0)
        (url,), _ = wb.call_args
        self.assertTrue(url.startswith("file://"))
        self.assertTrue(url.endswith("#hardware"))
        self.assertIn("opened", out)

    def test_no_topic_opens_at_the_top(self):
        with mock.patch("webbrowser.open", return_value=True) as wb:
            rc, _ = _run_cli(["docs"])
        self.assertEqual(rc, 0)
        (url,), _ = wb.call_args
        self.assertFalse("#" in url)

    def test_rendered_html_has_all_sections_tables_and_style(self):
        html = docs_mod._render_html()
        for topic in docs_mod.TOPICS:
            self.assertIn('<section id="%s">' % topic, html)
        self.assertIn("<table>", html)          # hardware parts list
        self.assertIn("<h1>", html)
        self.assertIn("prefers-color-scheme", html)   # dark mode
        self.assertIn("docs.openbricks.dev", html)    # footer pointer
        self.assertNotIn("{eval-rst}", html)

    def test_no_browser_available_is_a_clean_error_pointing_at_text(self):
        err = io.StringIO()
        with mock.patch("webbrowser.open", return_value=False), \
                mock.patch("sys.stderr", err):
            rc, _ = _run_cli(["docs", "install"])
        self.assertEqual(rc, 1)
        self.assertIn("--text", err.getvalue())


class EvalRstCleanupTests(unittest.TestCase):
    """Sphinx-only blocks must become useful offline pointers."""

    def test_argparse_block_points_at_help(self):
        text = docs_mod._load("cli")
        self.assertNotIn("{eval-rst}", text)
        self.assertNotIn(".. argparse::", text)
        self.assertIn("openbricks --help", text)

    def test_literalinclude_blocks_point_at_the_example_files(self):
        text = docs_mod._load("examples")
        self.assertNotIn("literalinclude", text)
        self.assertIn("examples/st3032_drivebase_square.py", text)
        self.assertIn("github.com/1e0ng/openbricks", text)

    def test_other_eval_rst_gets_generic_pointer(self):
        cleaned = docs_mod._clean(
            "```{eval-rst}\n.. some-directive::\n```")
        self.assertIn("docs.openbricks.dev", cleaned)


if __name__ == "__main__":
    unittest.main()
