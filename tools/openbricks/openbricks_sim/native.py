# SPDX-License-Identifier: MIT
"""``openbricks sim``: launch the sim, the native desktop application.

The sim is a Rust program built per platform in CI and attached to the
firmware release of the same version (``openbricks-sim-<version>-
<platform>.tar.gz``, ``.zip`` on Windows) with an Ed25519 signature
the CLI checks against its baked-in project key — the same key that
signs firmware images. The first ``openbricks sim`` on a machine
downloads the binary for its platform into the cache
(``~/.cache/openbricks/sim/<version>/``); ``$OPENBRICKS_SIM_BIN``
points at a build of your own instead.
"""
import io
import os
import pathlib
import platform
import shutil
import stat
import subprocess
import sys
import tarfile
import urllib.request
import zipfile

from openbricks_dev import __version__, _signing

REPO = "1e0ng/openbricks"


def platform_tag(system=None, machine=None):
    """``macos-arm64``, ``macos-x86_64``, ``linux-x86_64``,
    ``linux-aarch64`` or ``windows-x86_64``."""
    system = (system or sys.platform).lower()
    machine = (machine or platform.machine()).lower()
    arch = "arm64" if machine in ("arm64", "aarch64") else "x86_64" if machine in ("x86_64", "amd64") else machine
    if system.startswith("darwin"):
        return "macos-" + arch
    if system.startswith("linux"):
        return "linux-" + ("aarch64" if arch == "arm64" else arch)
    if system.startswith("win"):
        return "windows-" + arch
    raise RuntimeError("no sim build for %s/%s" % (system, machine))


def asset_name(version, tag):
    return "openbricks-sim-%s-%s.%s" % (version, tag, "zip" if tag.startswith("windows") else "tar.gz")


def download_url(version, name):
    return "https://github.com/%s/releases/download/v%s/%s" % (REPO, version, name)


def cache_dir():
    env = os.environ.get("OPENBRICKS_SIM_DIR")
    if env:
        return pathlib.Path(env).expanduser()
    cache = os.environ.get("XDG_CACHE_HOME") or os.path.join(os.path.expanduser("~"), ".cache")
    return pathlib.Path(cache) / "openbricks" / "sim"


def binary_path(version, tag):
    exe = "openbricks-sim.exe" if tag.startswith("windows") else "openbricks-sim"
    return cache_dir() / version / exe


def _fetch(opener, url):
    with opener(url) as resp:
        return resp.read()


def ensure_binary(version=None, tag=None, download=True, opener=urllib.request.urlopen, verify=None, progress=None):
    """The sim binary for this machine, downloading and verifying it
    on first use. ``$OPENBRICKS_SIM_BIN`` wins when set."""
    say = progress or (lambda s: None)
    verify = verify or _signing.verify
    env = os.environ.get("OPENBRICKS_SIM_BIN")
    if env:
        p = pathlib.Path(env).expanduser()
        if not p.exists():
            raise RuntimeError("OPENBRICKS_SIM_BIN points at %s, which does not exist" % p)
        return p
    version = version or __version__
    tag = tag or platform_tag()
    path = binary_path(version, tag)
    if path.exists():
        return path
    name = asset_name(version, tag)
    url = download_url(version, name)
    if not download:
        raise RuntimeError("the sim %s is not installed; run `openbricks sim` online once to fetch %s, or set OPENBRICKS_SIM_BIN" % (version, url))
    say("downloading %s" % url)
    try:
        data = _fetch(opener, url)
        sig = _fetch(opener, url + ".sig")
    except Exception as exc:  # noqa: BLE001 - any network failure reads the same to the user
        raise RuntimeError("could not download the sim %s for %s (%s): %s" % (version, tag, url, exc)) from exc
    if not verify(data, sig):
        raise RuntimeError("the downloaded sim does not carry a valid openbricks signature - refusing to run it")
    say("verified signature; unpacking")
    target = path.parent
    target.mkdir(parents=True, exist_ok=True)
    _unpack(data, name, target)
    if not path.exists():
        raise RuntimeError("the archive %s did not contain %s" % (name, path.name))
    if not tag.startswith("windows"):
        path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    say("sim %s ready at %s" % (version, path))
    return path


def _unpack(data, name, target):
    target = pathlib.Path(target)
    if name.endswith(".zip"):
        with zipfile.ZipFile(io.BytesIO(data)) as zf:
            for m in zf.infolist():
                dest = target / pathlib.Path(m.filename).name
                if m.is_dir():
                    continue
                with zf.open(m) as src, open(dest, "wb") as dst:
                    shutil.copyfileobj(src, dst)
    else:
        with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as tf:
            for m in tf.getmembers():
                if not m.isfile():
                    continue
                dest = target / pathlib.Path(m.name).name
                src = tf.extractfile(m)
                with open(dest, "wb") as dst:
                    shutil.copyfileobj(src, dst)


def launch(binary, bundles, file=None, run=subprocess.call):
    """Run the sim with the brick bundles and, optionally, an assembly file."""
    cmd = [str(binary)]
    for b in bundles:
        cmd += ["--bricks", str(b)]
    if file:
        cmd.append(str(file))
    return run(cmd)
