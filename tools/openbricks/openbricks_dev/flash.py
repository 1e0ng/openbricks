# SPDX-License-Identifier: MIT
"""
``openbricks flash`` — flash firmware + bake in the hub's BLE name.

Two-step flow:

  1. ``esptool ... write-flash`` writes the combined firmware image.
  2. ``mpremote ... exec`` pokes the BLE advertising name into NVS under
     namespace ``openbricks``, key ``hub_name`` (where ``openbricks``
     reads it via ``openbricks._read_hub_name``).

Step 1 is commodity tooling; step 2 is how we achieve per-device identity
without rebuilding the firmware per hub. The name is verified via an NVS
readback after the write, so a quiet failure on the device can't be
mistaken for success.
"""

import os
import re
import shutil
import subprocess
import sys
import time


_NVS_NAMESPACE = "openbricks"
_NVS_KEY       = "hub_name"

# --verbose: echo every subprocess command line and cache path. The
# default view is intent-level steps only — what a user needs to
# follow the flash, not the plumbing that performs it.
_verbose = False


def _vprint(msg):
    if _verbose:
        print(msg, flush=True)

# Provenance marker written after every flash: b"<version>:<verdict>"
# under this key. The next ``openbricks flash`` reads it back to label
# the CURRENT firmware (official/customized) without re-hashing flash.
# The version prefix guards staleness: firmware replaced behind the
# CLI's back (raw esptool, mpremote) no longer matches and degrades to
# "customized".
_NVS_SIG_KEY = "fw_sig"

# MicroPython's makeimg.py starts the merged ``firmware.bin`` at the
# chip's bootloader offset: 0x1000 on the classic ESP32, 0x0 on the
# ESP32-S3. The image must be written back at that same base or the ROM
# never finds the bootloader ("Invalid image block, can't boot"). The
# partition table always lives at flash address 0x8000, and every
# partition-table entry opens with the magic bytes ``AA 50`` — so the
# magic's *file* offset (0x8000 - base) tells us which base the image
# was built for.
_PT_MAGIC        = b"\xaa\x50"
_PT_FLASH_OFFSET = 0x8000
_IMAGE_BASES     = (0x0, 0x1000)


class FlashError(Exception):
    """Raised by ``run`` when any step of the flash / name-write fails."""


# The USB vendor-ID filter lives in ``_ports`` (shared with
# servo-id's adapter autodetection) — one table, no drift.

# ESP image extended-header chip IDs (bytes 12-13 of the bootloader
# image, little-endian) -> esptool --chip names. The merged
# firmware.bin starts with the bootloader image, so the file's first
# 16 bytes carry the chip the image was built for.
_IMAGE_CHIP_IDS = {
    0x0000: "esp32",
    0x0002: "esp32s2",
    0x0005: "esp32c3",
    0x0009: "esp32s3",
    0x000D: "esp32c6",
    0x0010: "esp32h2",
}
_IMAGE_MAGIC = 0xE9


def _autodetect_port():
    """Return the single connected ESP serial port, or die usefully.

    Shared filter in ``_ports`` (servo-id uses the same one): known
    USB-serial vendor IDs, so a modem or an Arduino on another port
    can't be grabbed by mistake. Exactly one match is required —
    flashing is destructive, so with two candidates we refuse to
    guess.
    """
    from openbricks_dev._ports import autodetect_port
    return autodetect_port(FlashError, "flashing is destructive")


def _detect_chip(esptool, port):
    """Ask the bootloader which chip this is. Returns an esptool
    ``--chip`` name (e.g. ``esp32``, ``esp32s3``) or ``None`` if the
    probe failed (device wedged mid-boot etc.) — callers must treat
    ``None`` as "unknown", not as any particular chip."""
    chip_cmd = "chip-id" if esptool.endswith("esptool") else "chip_id"
    try:
        out = subprocess.run(
            [esptool, "--port", port, chip_cmd],
            capture_output=True, text=True, timeout=30).stdout
    except Exception as e:
        print("warning: chip probe failed (%s)" % e, file=sys.stderr)
        return None
    # esptool v5 prints a column-padded "Chip type:          ESP32-S3
    # (QFN56)"; v4 printed "Chip is ESP32-S3 ...". Accept both — the
    # first ship only knew the v4 phrasing and the probe silently
    # failed on every v5 install (bench, 1.22.0).
    m = re.search(r"Chip (?:is|type:)\s+(ESP32)(?:-([A-Z0-9]+))?", out)
    if not m:
        tail = [l for l in out.splitlines() if l.strip()][-1:] or ["<empty>"]
        print("warning: could not identify the connected chip from "
              "esptool output (last line: %r) — skipping the "
              "image/chip match check" % tail[0], file=sys.stderr)
        return None
    # Canonicalize to esptool --chip family names. The suffix is only
    # a family marker for S2/S3/C-/H-/P-series; classic ESP32 modules
    # report variant suffixes (D0WD, D0WD-V3, PICO...) that must all
    # map to plain "esp32", not a bogus chip name.
    suffix = m.group(2) or ""
    if suffix in ("S2", "S3", "C2", "C3", "C5", "C6", "H2", "P4"):
        name = ("esp32" + suffix).lower()
    else:
        name = "esp32"
    pretty = m.group(1) + ("-" + m.group(2) if m.group(2) else "")
    print("detected chip: %s (%s)" % (pretty, name))
    return name


def _image_chip_name(firmware_path):
    """Read the chip the image was built for from the bootloader
    header at the start of the merged image. ``None`` if the header
    isn't recognizable (foreign image) — callers skip the check."""
    try:
        with open(firmware_path, "rb") as f:
            hdr = f.read(16)
    except OSError:
        return None
    if len(hdr) < 14 or hdr[0] != _IMAGE_MAGIC:
        return None
    chip_id = hdr[12] | (hdr[13] << 8)
    return _IMAGE_CHIP_IDS.get(chip_id)


_RELEASES_LATEST_URL = \
    "https://api.github.com/repos/1e0ng/openbricks/releases/latest"
_FIRMWARE_CACHE_DIR = os.path.expanduser("~/.cache/openbricks/firmware")


def _http_get(url):
    """GET ``url`` and return the response bytes. Split out as a seam
    for tests; stdlib-only so flash has no new dependencies."""
    import urllib.request
    req = urllib.request.Request(
        url, headers={"User-Agent": "openbricks-flash"})
    with urllib.request.urlopen(req, timeout=60) as resp:
        return resp.read()


def _fetch_asset(asset):
    """Download one release asset into the cache (or reuse it) and
    return the local path."""
    path = os.path.join(_FIRMWARE_CACHE_DIR, asset["name"])
    if os.path.exists(path) and os.path.getsize(path) == asset.get("size", -1):
        print("using cached %s" % asset["name"])
        _vprint("cache path: %s" % path)
        return path
    print("downloading %s ..." % asset["name"])
    try:
        data = _http_get(asset["browser_download_url"])
    except Exception as e:
        _die("firmware download failed (%s) — pass --firmware with a "
             "local .bin" % e)
    if asset.get("size") is not None and len(data) != asset["size"]:
        _die("firmware download truncated (%d of %d bytes) — retry, "
             "or pass --firmware" % (len(data), asset["size"]))
    os.makedirs(_FIRMWARE_CACHE_DIR, exist_ok=True)
    tmp = path + ".part"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)
    _vprint("saved to %s" % path)
    return path


def _latest_firmware_for(chip):
    """Download (or reuse from cache) the newest release's merged
    firmware image for ``chip`` ("esp32" / "esp32s3"), plus its
    detached ``.sig`` when the release carries one. Returns
    ``(bin_path, version_str)`` — the signature lands next to the
    image in the cache where ``_read_sig_for`` finds it. Dies with
    instructions on any failure — a flash must never proceed on a
    guessed image."""
    import json
    try:
        release = json.loads(_http_get(_RELEASES_LATEST_URL))
    except Exception as e:
        _die("--firmware not given and the latest-release lookup "
             "failed (%s) — offline? Pass --firmware with a local "
             ".bin from the Releases page." % e)
    prefix = "openbricks-%s-firmware-" % chip
    asset = None
    sig_asset = None
    for a in release.get("assets", []):
        name = a.get("name", "")
        if not name.startswith(prefix):
            continue
        if name.endswith(".sig"):
            sig_asset = a
        else:
            asset = a
    if asset is None:
        _die("release %s has no asset matching %s*.bin — pass "
             "--firmware explicitly"
             % (release.get("tag_name", "?"), prefix))
    version = _parse_version_text(release.get("tag_name", ""))
    path = _fetch_asset(asset)
    if sig_asset is not None:
        _fetch_asset(sig_asset)
    return path, version


def _parse_version_text(text):
    """The ``X.Y.Z`` inside ``text`` (a tag name or file name), or
    ``None``. ``v1.2.3`` and ``...-firmware-v1.2.3.bin`` both work."""
    m = re.search(r"v?(\d+\.\d+\.\d+)", text or "")
    return m.group(1) if m else None


def _version_tuple(version):
    return tuple(int(p) for p in version.split("."))


def _read_sig_for(firmware_path):
    """The detached signature bytes for ``firmware_path`` (its
    sibling ``.sig`` file), or ``None``."""
    try:
        with open(firmware_path + ".sig", "rb") as f:
            return f.read()
    except OSError:
        return None


def _firmware_verdict(firmware_path):
    """``"official"`` / ``"customized"`` for a local image, from its
    sibling ``.sig`` against the baked-in public key."""
    from openbricks_dev import _signing
    with open(firmware_path, "rb") as f:
        data = f.read()
    return _signing.verdict(data, _read_sig_for(firmware_path))


def _confirm(question, assume_yes, input_fn=input):
    """Ask the user a yes/no question on the terminal. ``--yes``
    short-circuits; a non-interactive stdin dies instead of hanging
    a CI pipeline on ``input()``."""
    if assume_yes:
        print("%s -- proceeding (--yes)" % question)
        return True
    if not sys.stdin.isatty():
        _die("%s — refusing without confirmation on a "
             "non-interactive stdin; re-run with --yes" % question)
    answer = input_fn("%s [y/N] " % question)
    return answer.strip().lower() in ("y", "yes")


def _die(msg):
    raise FlashError(msg)


def _require_tool(name):
    path = shutil.which(name)
    if path is None:
        _die("%s not found on PATH — install with: pip install esptool mpremote"
             % name)
    return path


def _esptool_paths_and_commands():
    """Return ``(binary_path, write_cmd, erase_cmd)`` for the esptool
    install on PATH.

    Prefer the v5 binary name (``esptool``) and v5 kebab-case
    commands (``write-flash`` / ``erase-flash``) when both are
    available — output is clean of "DEPRECATED" warnings. Fall
    back to the v4 names when v5 isn't installed (typical on
    Python 3.9, since esptool 5+ requires Python ≥ 3.10).
    """
    if shutil.which("esptool") is not None:
        return shutil.which("esptool"), "write-flash", "erase-flash"
    if shutil.which("esptool.py") is not None:
        return shutil.which("esptool.py"), "write_flash", "erase_flash"
    _die("esptool not found on PATH — install with: pip install esptool mpremote")


def _run(cmd, check=True):
    """Stream subprocess output; optionally raise on non-zero exit."""
    _vprint(">>> " + " ".join(cmd))
    rc = subprocess.call(cmd)
    if check and rc != 0:
        _die("command failed (rc=%d): %s" % (rc, " ".join(cmd)))
    return rc


def _mpremote_exec(mpremote, port, snippet):
    """Run a one-liner Python snippet on the device over mpremote.

    Returns ``(returncode, stdout, stderr)``. ``_run`` isn't used here
    because we need to capture stdout for readback verification.

    The ``resume`` argument is critical: without it, each
    ``mpremote exec`` invocation does a soft reset before entering
    raw REPL. Once the hub name has been written to NVS, the
    soft-reset re-runs frozen ``main.py`` which now activates BLE
    and blocks in ``launcher.run()`` — and the *next* mpremote
    invocation can't enter raw REPL, failing the readback step
    with "could not enter raw repl". ``resume`` flips
    ``_auto_soft_reset`` to ``False`` so the chip's REPL state
    persists across our flash-flow steps.
    """
    cmd = [mpremote, "connect", port, "resume", "exec", snippet]
    _vprint(">>> " + " ".join(cmd))
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=30)
    except subprocess.TimeoutExpired:
        return -1, "", "mpremote timed out after 30s"
    return proc.returncode, proc.stdout, proc.stderr


def _wait_for_repl(mpremote, port, timeout_s=20):
    """Poll mpremote until the device answers a trivial ``print``.

    After flashing, the device reboots and the serial link takes a second
    or two to come back; mpremote has no built-in retry. Idle-poll every
    500 ms.

    On timeout, surface mpremote's last rc / stdout / stderr so the
    user can see *why* it's failing — a bare "timed out" message is
    indistinguishable between "chip is stuck in user code that never
    yields to the REPL", "chip is in download mode after a flash
    failure", and "USB-Serial-JTAG never re-enumerated".
    """
    deadline = time.time() + timeout_s
    last_rc, last_out, last_err = -1, "", ""
    while time.time() < deadline:
        last_rc, last_out, last_err = _mpremote_exec(mpremote, port, "print('ok')")
        if last_rc == 0 and "ok" in last_out:
            return
        time.sleep(0.5)
    _die(
        "timed out waiting for device REPL on %s after %.0fs\n"
        "  last mpremote rc:     %d\n"
        "  last mpremote stdout: %r\n"
        "  last mpremote stderr: %r\n"
        "hint: if stderr mentions 'could not enter raw repl', the chip's\n"
        "      main.py is stuck in user code — power-cycle and try again,\n"
        "      or hold the BOOT button while pressing reset to enter the\n"
        "      ROM bootloader (then re-run with --skip-erase)."
        % (port, timeout_s, last_rc, last_out.strip(), last_err.strip())
    )


# Starter QTR calibration shipped by ``--with-qtr-init`` — a real
# bench sweep (2026-08-17, QTRX-HD-15A on the WRO mat, default pins
# 1-10; the per-element spans from that run's log). Format matches
# ``QTRArray.save_calibration``: {"pins", "min", "max"}. A starter
# file gets a fresh flash line-following immediately; heights, mats
# and lighting differ, so examples/qtr_calibrate.py remains the way
# to a calibration measured on YOUR rig.
_QTR_CAL_PATH = "/qtr.cal"
_QTR_STARTER_CAL = {
    "pins": [1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
    "min": [9346, 4048, 3888, 3728, 3984, 3888, 3824, 3968, 4593, 8658],
    "max": [44922, 37849, 36024, 35816, 36056, 36536, 32888, 36552,
            40057, 43834],
}


def _write_qtr_starter_cal(mpremote, port):
    """Store the starter QTR calibration on the hub filesystem and
    verify it parses back — a corrupt or truncated write must fail
    HERE, not as a RuntimeError in the user's first line-follow."""
    import json
    payload = json.dumps(_QTR_STARTER_CAL)
    snippet = (
        "f = open(%r, 'w'); f.write(%r); f.close(); "
        "import json; d = json.load(open(%r)); "
        "print('qtr-cal-ok', len(d['min']), len(d['max']))"
    ) % (_QTR_CAL_PATH, payload, _QTR_CAL_PATH)
    rc, out, err = _mpremote_exec(mpremote, port, snippet)
    if rc != 0 or "qtr-cal-ok 10 10" not in out:
        _die("failed to write starter QTR calibration:\n" + (err or out))
    print("starter QTR calibration written to %s (re-run "
          "examples/qtr_calibrate.py for your own mat/lighting)"
          % _QTR_CAL_PATH)


def _write_hub_name(mpremote, port, name):
    # The name already passed validation in ``_validate_name``; plain
    # bytes literal is safe.
    snippet = (
        "import esp32; "
        "nvs = esp32.NVS(%r); "
        "nvs.set_blob(%r, %r); "
        "nvs.commit(); "
        "print('wrote:', %r)"
    ) % (_NVS_NAMESPACE, _NVS_KEY, name.encode(), name)
    rc, out, err = _mpremote_exec(mpremote, port, snippet)
    if rc != 0:
        _die("failed to write hub name to NVS:\n" + (err or out))
    _vprint(out.strip())


def _read_hub_name(mpremote, port):
    snippet = (
        "import esp32; "
        "nvs = esp32.NVS(%r); "
        "buf = bytearray(64); "
        "n = nvs.get_blob(%r, buf); "
        "print(bytes(buf[:n]).decode())"
    ) % (_NVS_NAMESPACE, _NVS_KEY)
    rc, out, err = _mpremote_exec(mpremote, port, snippet)
    if rc != 0:
        _die("failed to read hub name back from NVS:\n" + (err or out))
    return out.strip()


_PROBE_SNIPPET = (
    "try:\n"
    "    import openbricks\n"
    "    print('ver=' + openbricks.__version__)\n"
    "except Exception:\n"
    "    print('ver=')\n"
    "try:\n"
    "    import esp32\n"
    "    nvs = esp32.NVS(%r)\n"
    "    buf = bytearray(96)\n"
    "    n = nvs.get_blob(%r, buf)\n"
    "    print('sig=' + bytes(buf[:n]).decode())\n"
    "except Exception:\n"
    "    print('sig=')\n"
) % (_NVS_NAMESPACE, _NVS_SIG_KEY)


def _last_nonempty_line(text):
    """The final non-blank line of ``text``, or '' — subprocess error
    output ends with the line that names the actual failure."""
    for line in reversed((text or "").splitlines()):
        if line.strip():
            return line.strip()
    return ""


def _read_current_firmware(mpremote, port):
    """The running firmware's ``(version, verdict)``, or
    ``(None, None)`` when the chip has no reachable openbricks REPL
    (foreign firmware, program running, bootloader mode).

    The verdict comes from the provenance marker the last
    ``openbricks flash`` stored; a marker whose version prefix does
    not match the running version is stale (firmware replaced behind
    the CLI's back) and degrades to "customized".

    A failed probe says WHY on stderr (mpremote's own message):
    "could not enter raw repl" and "failed to access <port>" are
    different bugs — hub-side state vs host-side port contention —
    and swallowing the reason forced a bench-instrumentation round
    to tell them apart (the 2026-08-13 flash-after-log hunt)."""
    from openbricks_dev import _signing
    rc, out, err = _mpremote_exec(mpremote, port, _PROBE_SNIPPET)
    if rc != 0:
        reason = _last_nonempty_line(err) or _last_nonempty_line(out)
        print("probe: mpremote rc=%d%s"
              % (rc, (": " + reason) if reason else ""),
              file=sys.stderr)
        return None, None
    version = marker = ""
    for line in out.splitlines():
        if line.startswith("ver="):
            version = line[4:].strip()
        elif line.startswith("sig="):
            marker = line[4:].strip()
    if not version:
        return None, None
    verdict = _signing.CUSTOMIZED
    if ":" in marker:
        marked_version, marked_verdict = marker.split(":", 1)
        if marked_version == version and marked_verdict == _signing.OFFICIAL:
            verdict = _signing.OFFICIAL
    return version, verdict


def _write_fw_marker(mpremote, port, verdict):
    """Store the provenance marker for the firmware just flashed;
    return the version read off the chip (None for a foreign image).

    The version half is read from the chip itself (never trusted
    from a file name); a chip whose fresh firmware has no importable
    ``openbricks`` is foreign — no marker, and the next flash will
    report "unknown"."""
    rc, out, _err = _mpremote_exec(
        mpremote, port,
        "import openbricks; print(openbricks.__version__)")
    version = out.strip() if rc == 0 else ""
    if not version:
        print("warning: flashed image has no importable openbricks — "
              "skipping the provenance marker", file=sys.stderr)
        return None
    marker = "%s:%s" % (version, verdict)
    snippet = (
        "import esp32; "
        "nvs = esp32.NVS(%r); "
        "nvs.set_blob(%r, %r); "
        "nvs.commit(); "
        "print('marker:', %r)"
    ) % (_NVS_NAMESPACE, _NVS_SIG_KEY, marker.encode(), marker)
    rc, out, err = _mpremote_exec(mpremote, port, snippet)
    if rc != 0:
        _die("failed to write the firmware provenance marker:\n"
             + (err or out))
    _vprint(out.strip())
    print("firmware marker: %s (%s)" % (version, verdict))
    return version


def _image_base_offset(firmware_path):
    """Return the flash address (as a ``"0x…"`` string) the merged image
    must be written at, derived from where the partition-table magic
    sits inside the file itself.

    Detecting from the image — rather than from ``--chip``, which
    defaults to ``auto`` — means the offset is right even when the user
    never names the chip. Exactly one candidate base must match; zero
    (not a merged MicroPython image?) or several (can't disambiguate)
    raise instead of guessing, because a wrong offset bricks the boot
    path until the next reflash.
    """
    try:
        with open(firmware_path, "rb") as f:
            head = f.read(_PT_FLASH_OFFSET + len(_PT_MAGIC))
    except OSError as e:
        _die("cannot read firmware image %s: %s" % (firmware_path, e))
    matches = [
        base for base in _IMAGE_BASES
        if head[_PT_FLASH_OFFSET - base:
                _PT_FLASH_OFFSET - base + len(_PT_MAGIC)] == _PT_MAGIC
    ]
    if len(matches) != 1:
        _die(
            "cannot determine the flash offset of %s: expected the "
            "partition-table magic at file offset 0x7000 (classic ESP32 "
            "image, flashed at 0x1000) or 0x8000 (ESP32-S3 image, flashed "
            "at 0x0); found it at %s. Is this a merged firmware.bin from "
            "scripts/build_firmware.sh / the Releases page?"
            % (firmware_path,
               ", ".join("0x%x" % (_PT_FLASH_OFFSET - b) for b in matches)
               or "neither")
        )
    return "0x%x" % matches[0]


def _validate_name(name):
    if not name:
        _die("--name cannot be empty")
    if len(name.encode()) > 29:
        _die("--name is %d bytes after UTF-8 encoding; BLE GAP caps at ~29"
             % len(name.encode()))
    if "\x00" in name:
        _die("--name cannot contain NUL bytes")


def run(args):
    """Subcommand entry. ``args`` is an argparse ``Namespace``."""
    global _verbose
    _verbose = bool(getattr(args, "verbose", False))
    _validate_name(args.name)

    # esptool v5 renamed the binary (``esptool.py`` → ``esptool``) and
    # switched commands to kebab-case (``write_flash`` → ``write-flash``,
    # ``erase_flash`` → ``erase-flash``). The legacy forms still work in
    # v5 but emit deprecation warnings on every flash. We can't pin v5
    # as a floor (it requires Python >= 3.10 and openbricks supports
    # >= 3.9), so detect which is on PATH at runtime and pick command
    # spelling accordingly.
    esptool, write_cmd, erase_cmd = _esptool_paths_and_commands()
    mpremote = _require_tool("mpremote")

    if args.port is None:
        args.port = _autodetect_port()

    # Read the RUNNING firmware first, before esptool's probes reset
    # the chip into the bootloader: version plus the provenance
    # marker the last flash stored.
    current_version, current_verdict = _read_current_firmware(
        mpremote, args.port)
    if current_version:
        print("current firmware: %s (%s)"
              % (current_version, current_verdict))
    else:
        print("current firmware: unknown (no reachable openbricks "
              "REPL — foreign firmware, or a program is running)")

    # Identify the connected chip up front: it gates the automatic
    # firmware download and the image/chip mismatch guard.
    detected_chip = _detect_chip(esptool, args.port)

    if args.firmware is None:
        if detected_chip is None:
            _die("--firmware not given and the connected chip could "
                 "not be identified — pass --firmware (and possibly "
                 "--chip) explicitly")
        args.firmware, target_version = _latest_firmware_for(detected_chip)
    else:
        target_version = _parse_version_text(
            os.path.basename(args.firmware))

    firmware_verdict = _firmware_verdict(args.firmware)
    print("target firmware:  %s (%s)"
          % (target_version or "unknown version", firmware_verdict))

    if current_version and target_version:
        cur = _version_tuple(current_version)
        tgt = _version_tuple(target_version)
        if tgt == cur:
            question = ("target %s is the SAME version as the current "
                        "firmware — reinstall?" % target_version)
        elif tgt < cur:
            question = ("target %s is OLDER than the current %s — "
                        "downgrade?" % (target_version, current_version))
        else:
            question = None
        if question is not None and not _confirm(question, args.yes):
            print("aborted — nothing was flashed.")
            return 0

    # Resolve the write offset from the image before touching the chip,
    # so an unrecognizable image fails the flash *before* the erase.
    write_offset = _image_base_offset(args.firmware)

    # Refuse a mismatched image — flashing an esp32 image onto an S3
    # (or vice versa) bricks the boot silently. Both sides degrade to
    # "unknown" rather than guessing: an unknown side skips the check
    # with a warning.
    image_chip = _image_chip_name(args.firmware)
    if image_chip and detected_chip and image_chip != detected_chip:
        _die("firmware image %s was built for %s but the connected "
             "chip is %s — download the matching "
             "openbricks-%s-firmware .bin from the Releases page"
             % (args.firmware, image_chip, detected_chip, detected_chip))
    if args.chip == "auto" and detected_chip:
        args.chip = detected_chip

    _vprint("=== openbricks flash: name=%r port=%s offset=%s chip=%s ==="
            % (args.name, args.port, write_offset, args.chip))

    if not args.skip_erase:
        print("erasing flash ...")
        _run([esptool, "--chip", args.chip, "--port", args.port, erase_cmd])

    print("writing firmware ...")
    _run([
        esptool, "--chip", args.chip, "--port", args.port,
        "--baud", args.baud, write_cmd, write_offset, args.firmware,
    ])

    # esptool leaves the device reset; give USB-CDC ports time to
    # re-enumerate before asking mpremote to reconnect.
    print("waiting for the hub to come back ...")
    time.sleep(2.0)
    _wait_for_repl(mpremote, args.port)

    _write_hub_name(mpremote, args.port, args.name)

    readback = _read_hub_name(mpremote, args.port)
    if readback != args.name:
        _die("verification failed: wrote %r, read back %r" % (args.name, readback))
    print("hub name %r written and verified" % readback)

    if getattr(args, "with_qtr_init", False):
        _write_qtr_starter_cal(mpremote, args.port)

    flashed_version = _write_fw_marker(mpremote, args.port,
                                       firmware_verdict)

    # Trigger a hardware reset so the freshly-flashed chip boots into
    # the BLE-active runtime state (now that hub_name is in NVS).
    #
    # Why ``resume exec --no-follow machine.reset()`` rather than the
    # ``mpremote ... reset`` alias: the alias expands to
    # ``exec --no-follow "machine.reset()"`` with the default
    # ``enter_raw_repl(soft_reset=True)``. That soft reset itself
    # boots the chip into the new state, blocking in
    # ``launcher.run()`` before mpremote can reach the raw REPL to
    # actually send ``machine.reset()`` — surfacing as
    # ``TransportError: could not enter raw repl``. ``resume`` flips
    # ``_auto_soft_reset`` off so we go straight from friendly REPL
    # into raw REPL once, send the snippet, and disconnect; the
    # snippet's own ``machine.reset()`` is what reboots the chip.
    print("rebooting hub ...")
    _run([mpremote, "connect", args.port, "resume",
          "exec", "--no-follow",
          "import machine; machine.reset()"], check=False)

    if flashed_version:
        print("done — hub %r is running openbricks %s (%s)."
              % (args.name, flashed_version, firmware_verdict))
    else:
        print("done — hub %r flashed on %s." % (args.name, args.port))
    if not args.skip_erase:
        # A silent loss is a bug: the full-chip erase above took the
        # staged program and every saved calibration with it. Without
        # this note the next button press does NOTHING — no program,
        # no run log, no visible reason (bench 2026-08-14).
        print("note: the flash erased the hub's filesystem — the "
              "staged program and saved calibrations are gone.\n"
              "      re-stage with:  openbricks upload <script.py> "
              "-n %s\n"
              "      (and re-run sensor calibration, e.g. "
              "examples/qtr_calibrate.py, if used)" % args.name)
    return 0


def main_standalone():
    """Called when someone invokes this module directly.

    Not the primary entry point (``cli.main`` is) — present so the module
    is runnable as ``python -m openbricks_dev.flash ...`` during
    development.
    """
    from openbricks_dev.cli import _build_parser
    parser = _build_parser()
    # Reparse argv with 'flash' prepended so the user can invoke without
    # typing the subcommand name.
    args = parser.parse_args(["flash"] + sys.argv[1:])
    try:
        return run(args)
    except FlashError as e:
        print("error: %s" % e, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main_standalone())
