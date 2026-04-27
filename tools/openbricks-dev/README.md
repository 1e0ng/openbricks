# openbricks-dev (DEPRECATED)

This PyPI project has been **replaced by `openbricks`**.

The host CLI (`openbricks-dev`) and the simulator (`openbricks-sim`)
are now a single PyPI package. Migrate with:

```
pip uninstall openbricks-dev openbricks-sim
pip install 'openbricks[sim]'
```

Then update your scripts and shell aliases:

| Old                     | New                       |
|-------------------------|---------------------------|
| `openbricks-dev <cmd>`  | `openbricks <cmd>`        |
| `openbricks-sim <cmd>`  | `openbricks sim <cmd>`    |

Subcommands and arguments are unchanged. The Python module names
on the host (`openbricks_dev`, `openbricks_sim`) stay split so they
don't shadow the firmware-side `openbricks` package on the hub.

This 0.9.3 release is the final `openbricks-dev` release — the
binary it ships prints the migration message and exits non-zero.
No further versions will be published under this name.

For the full migration breakdown see the [CHANGELOG](https://github.com/1e0ng/openbricks/blob/main/tools/openbricks/CHANGELOG.md).
