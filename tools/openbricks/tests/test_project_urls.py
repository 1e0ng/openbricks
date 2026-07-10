# SPDX-License-Identifier: MIT
"""Pin the public-facing project URLs in ``pyproject.toml``.

openbricks.dev / docs.openbricks.dev are the official sites (served
from this repo's ``website/`` and ``docs/`` via GitHub Pages); the
PyPI sidebar links come from ``[project.urls]``, so a typo or an
accidental revert to the bare GitHub URL would only be noticed after
a release. Same tomllib/tomli skip dance as
``test_cibuildwheel_config``.
"""

import pathlib
import unittest

_PYPROJECT = pathlib.Path(__file__).resolve().parent.parent / "pyproject.toml"


def _load_pyproject():
    try:
        import tomllib
    except ImportError:
        try:
            import tomli as tomllib
        except ImportError:
            raise unittest.SkipTest(
                "neither tomllib (py311+) nor tomli is available")
    return tomllib.loads(_PYPROJECT.read_text())


class ProjectUrlsTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.urls = _load_pyproject().get("project", {}).get("urls", {})

    def test_homepage_is_the_official_website(self):
        self.assertEqual(self.urls.get("Homepage"), "https://openbricks.dev")

    def test_documentation_points_at_docs_site(self):
        self.assertEqual(self.urls.get("Documentation"),
                         "https://docs.openbricks.dev")

    def test_source_and_issues_stay_on_github(self):
        self.assertEqual(self.urls.get("Source"),
                         "https://github.com/1e0ng/openbricks")
        self.assertEqual(self.urls.get("Issues"),
                         "https://github.com/1e0ng/openbricks/issues")


if __name__ == "__main__":
    unittest.main()
