# SPDX-License-Identifier: MIT
"""Deprecation stub for the ``openbricks-dev`` PyPI project.

The host CLI was renamed to plain ``openbricks`` in v0.10, and the
former ``openbricks-sim`` simulator is now an extra
(``pip install 'openbricks[sim]'``). This package's only job is to
print a clear migration message when ``openbricks-dev`` is invoked
and exit non-zero.
"""

__version__ = "0.9.3"
