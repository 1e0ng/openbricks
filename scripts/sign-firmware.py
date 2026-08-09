#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Sign every firmware ``.bin`` in a directory with the project key.

CI usage (the release job)::

    FIRMWARE_SIGNING_KEY="$(cat key.pem)" \
        python scripts/sign-firmware.py release-out/

For each ``*.bin`` a raw 64-byte Ed25519 signature is written to
``<name>.bin.sig`` next to it. The private key arrives as PEM text
in the FIRMWARE_SIGNING_KEY environment variable (a GitHub Actions
secret); a missing key or an empty directory is a hard error — a
release with unsigned images must fail loudly, not ship quietly.

The matching public key ships in the CLI
(``openbricks_dev/_signing.py``); this script asserts the pair
matches so a rotated secret can't silently sign with a key the CLI
will call "customized".
"""

import os
import sys

from cryptography.hazmat.primitives import serialization


def main(out_dir):
    pem = os.environ.get("FIRMWARE_SIGNING_KEY", "")
    if not pem.strip():
        print("error: FIRMWARE_SIGNING_KEY is empty or unset — "
              "signing key secret missing", file=sys.stderr)
        return 1

    key = serialization.load_pem_private_key(
        pem.encode(), password=None)

    sys.path.insert(0, os.path.join(
        os.path.dirname(__file__), "..", "tools", "openbricks"))
    from openbricks_dev import _signing
    pub_hex = key.public_key().public_bytes(
        serialization.Encoding.Raw,
        serialization.PublicFormat.Raw).hex()
    if pub_hex != _signing.PUBLIC_KEY_HEX:
        print("error: the signing key's public half (%s) does not "
              "match openbricks_dev/_signing.py (%s) — rotate both "
              "together" % (pub_hex, _signing.PUBLIC_KEY_HEX),
              file=sys.stderr)
        return 1

    bins = sorted(
        f for f in os.listdir(out_dir) if f.endswith(".bin"))
    if not bins:
        print("error: no .bin files in %s — nothing to sign"
              % out_dir, file=sys.stderr)
        return 1

    for name in bins:
        path = os.path.join(out_dir, name)
        with open(path, "rb") as f:
            data = f.read()
        sig = key.sign(data)
        with open(path + ".sig", "wb") as f:
            f.write(sig)
        if not _signing.verify(data, sig):
            print("error: self-check failed for %s — signature does "
                  "not verify against the shipped public key" % name,
                  file=sys.stderr)
            return 1
        print("signed %s (%d bytes) -> %s.sig" % (name, len(data), name))
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: sign-firmware.py <dir-with-bin-files>",
              file=sys.stderr)
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
