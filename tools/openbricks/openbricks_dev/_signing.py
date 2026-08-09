# SPDX-License-Identifier: MIT
"""Firmware signature verification.

Every firmware image published by CI is signed with the project's
Ed25519 key (``scripts/sign-firmware.py``, key held as a CI
secret); the matching public key is baked in here. A ``.bin`` whose
``.bin.sig`` verifies against it is labeled ``official``; anything
else — no signature, wrong signature, self-built image — is
``customized``. The label is display provenance, not a security
boundary: flashing customized firmware is a supported workflow, the
suffix just tells you which kind you're holding.
"""

# Raw 32-byte Ed25519 public key, hex. The private half lives in the
# repository's CI secret FIRMWARE_SIGNING_KEY (and nowhere in git).
PUBLIC_KEY_HEX = \
    "774b962b48017ab426a829589deed0d4f2236feb14dcdf81f68b327d2d8210b8"

OFFICIAL = "official"
CUSTOMIZED = "customized"


def verify(data, signature, public_key_hex=None):
    """True when ``signature`` is a valid Ed25519 signature of
    ``data`` under ``public_key_hex`` (the shipped project key when
    omitted — resolved at call time, not def time, so tests can
    swap the module attribute)."""
    from cryptography.exceptions import InvalidSignature
    from cryptography.hazmat.primitives.asymmetric.ed25519 import (
        Ed25519PublicKey,
    )
    if public_key_hex is None:
        public_key_hex = PUBLIC_KEY_HEX
    key = Ed25519PublicKey.from_public_bytes(
        bytes.fromhex(public_key_hex))
    try:
        key.verify(signature, data)
        return True
    except InvalidSignature:
        return False


def verdict(data, signature, public_key_hex=None):
    """``"official"`` when the signature verifies, ``"customized"``
    otherwise (including no signature at all)."""
    if not signature:
        return CUSTOMIZED
    if verify(data, signature, public_key_hex):
        return OFFICIAL
    return CUSTOMIZED
