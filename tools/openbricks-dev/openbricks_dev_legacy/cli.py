# SPDX-License-Identifier: MIT
"""Deprecation stub. Prints a migration notice and exits non-zero."""

import sys


_MSG = """\
DEPRECATED: openbricks-dev has been replaced by ``openbricks``.

The host CLI (``openbricks-dev``) and the simulator (``openbricks-sim``)
are now a single PyPI package. Migrate with:

    pip uninstall openbricks-dev openbricks-sim
    pip install 'openbricks[sim]'

Then update your scripts and shell aliases:

    openbricks-dev <cmd>   →   openbricks <cmd>
    openbricks-sim <cmd>   →   openbricks sim <cmd>

Subcommands and arguments are unchanged. The Python module names on
the host (``openbricks_dev``, ``openbricks_sim``) are preserved for
the same reason — they don't shadow the firmware-side ``openbricks``
package on the hub.

This is the final ``openbricks-dev`` release. No further versions
will be published under this name.

CHANGELOG + migration guide:
  https://github.com/1e0ng/openbricks/blob/main/tools/openbricks/CHANGELOG.md
"""


def main(argv=None):
    sys.stderr.write(_MSG)
    return 1


if __name__ == "__main__":
    sys.exit(main())
