# SPDX-License-Identifier: MIT
"""Firmware signature verification (``openbricks_dev._signing``).

The verify/verdict primitives are exercised with an ephemeral
Ed25519 pair (the real private key exists only as a CI secret); the
shipped PUBLIC_KEY_HEX is pinned for shape so a mangled paste can't
quietly turn every release "customized".
"""

import os
import tempfile
import unittest

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import (
    Ed25519PrivateKey,
)

from openbricks_dev import _signing
from openbricks_dev import flash


def _ephemeral_pair():
    key = Ed25519PrivateKey.generate()
    pub_hex = key.public_key().public_bytes(
        serialization.Encoding.Raw,
        serialization.PublicFormat.Raw).hex()
    return key, pub_hex


class VerifyTests(unittest.TestCase):
    def test_good_signature_verifies(self):
        key, pub = _ephemeral_pair()
        data = b"firmware bytes"
        self.assertTrue(_signing.verify(data, key.sign(data), pub))

    def test_wrong_data_fails(self):
        key, pub = _ephemeral_pair()
        sig = key.sign(b"firmware bytes")
        self.assertFalse(_signing.verify(b"other bytes", sig, pub))

    def test_wrong_key_fails(self):
        key, _pub = _ephemeral_pair()
        _other, other_pub = _ephemeral_pair()
        data = b"firmware bytes"
        self.assertFalse(_signing.verify(data, key.sign(data), other_pub))


class VerdictTests(unittest.TestCase):
    def test_valid_signature_is_official(self):
        key, pub = _ephemeral_pair()
        data = b"image"
        self.assertEqual(
            _signing.verdict(data, key.sign(data), pub),
            _signing.OFFICIAL)

    def test_missing_signature_is_customized(self):
        _key, pub = _ephemeral_pair()
        self.assertEqual(_signing.verdict(b"image", None, pub),
                         _signing.CUSTOMIZED)
        self.assertEqual(_signing.verdict(b"image", b"", pub),
                         _signing.CUSTOMIZED)

    def test_invalid_signature_is_customized(self):
        _key, pub = _ephemeral_pair()
        self.assertEqual(
            _signing.verdict(b"image", b"\x00" * 64, pub),
            _signing.CUSTOMIZED)


class ShippedKeyTests(unittest.TestCase):
    def test_public_key_is_a_32_byte_hex_string(self):
        raw = bytes.fromhex(_signing.PUBLIC_KEY_HEX)
        self.assertEqual(len(raw), 32)

    def test_shipped_key_loads_as_ed25519(self):
        from cryptography.hazmat.primitives.asymmetric.ed25519 import (
            Ed25519PublicKey,
        )
        Ed25519PublicKey.from_public_bytes(
            bytes.fromhex(_signing.PUBLIC_KEY_HEX))


class LocalFirmwareVerdictTests(unittest.TestCase):
    """``flash._firmware_verdict``: a local .bin with a sibling .sig."""

    def _bin(self, data=b"image bytes"):
        fd, path = tempfile.mkstemp(suffix=".bin")
        with os.fdopen(fd, "wb") as f:
            f.write(data)
        self.addCleanup(os.unlink, path)
        return path

    def test_no_sibling_sig_is_customized(self):
        self.assertEqual(flash._firmware_verdict(self._bin()),
                         _signing.CUSTOMIZED)

    def test_garbage_sig_is_customized(self):
        path = self._bin()
        with open(path + ".sig", "wb") as f:
            f.write(b"\x00" * 64)
        self.addCleanup(os.unlink, path + ".sig")
        self.assertEqual(flash._firmware_verdict(path),
                         _signing.CUSTOMIZED)

    def test_signature_under_the_shipped_key_is_official(self):
        # Sign with an ephemeral key and point the module's public
        # key at its public half for the duration.
        key, pub = _ephemeral_pair()
        path = self._bin()
        with open(path, "rb") as f:
            data = f.read()
        with open(path + ".sig", "wb") as f:
            f.write(key.sign(data))
        self.addCleanup(os.unlink, path + ".sig")
        orig = _signing.PUBLIC_KEY_HEX
        _signing.PUBLIC_KEY_HEX = pub
        try:
            self.assertEqual(flash._firmware_verdict(path),
                             _signing.OFFICIAL)
        finally:
            _signing.PUBLIC_KEY_HEX = orig


class SignScriptTests(unittest.TestCase):
    """``scripts/sign-firmware.py`` end to end with an ephemeral key
    whose public half is temporarily installed as the shipped key."""

    def _script_main(self):
        import importlib.util
        script = os.path.join(
            os.path.dirname(__file__), "..", "..", "..",
            "scripts", "sign-firmware.py")
        if not os.path.exists(script):
            self.skipTest("repo scripts/ not present in this checkout")
        spec = importlib.util.spec_from_file_location(
            "sign_firmware", script)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.main

    def test_signs_every_bin_and_self_checks(self):
        main = self._script_main()
        key, pub = _ephemeral_pair()
        pem = key.private_bytes(
            serialization.Encoding.PEM,
            serialization.PrivateFormat.PKCS8,
            serialization.NoEncryption()).decode()
        out = tempfile.mkdtemp()
        self.addCleanup(__import__("shutil").rmtree, out, True)
        for name in ("a.bin", "b.bin"):
            with open(os.path.join(out, name), "wb") as f:
                f.write(name.encode() * 10)
        orig = _signing.PUBLIC_KEY_HEX
        _signing.PUBLIC_KEY_HEX = pub
        os.environ["FIRMWARE_SIGNING_KEY"] = pem
        try:
            rc = main(out)
        finally:
            _signing.PUBLIC_KEY_HEX = orig
            del os.environ["FIRMWARE_SIGNING_KEY"]
        self.assertEqual(rc, 0)
        for name in ("a.bin", "b.bin"):
            with open(os.path.join(out, name), "rb") as f:
                data = f.read()
            with open(os.path.join(out, name + ".sig"), "rb") as f:
                sig = f.read()
            self.assertTrue(_signing.verify(data, sig, pub))

    def test_missing_key_env_fails(self):
        main = self._script_main()
        os.environ.pop("FIRMWARE_SIGNING_KEY", None)
        self.assertEqual(main(tempfile.mkdtemp()), 1)

    def test_mismatched_public_half_fails(self):
        # Key rotated in the secret but not in _signing.py: refuse.
        main = self._script_main()
        key, _pub = _ephemeral_pair()
        pem = key.private_bytes(
            serialization.Encoding.PEM,
            serialization.PrivateFormat.PKCS8,
            serialization.NoEncryption()).decode()
        out = tempfile.mkdtemp()
        self.addCleanup(__import__("shutil").rmtree, out, True)
        with open(os.path.join(out, "a.bin"), "wb") as f:
            f.write(b"data")
        os.environ["FIRMWARE_SIGNING_KEY"] = pem
        try:
            rc = main(out)
        finally:
            del os.environ["FIRMWARE_SIGNING_KEY"]
        self.assertEqual(rc, 1)


if __name__ == "__main__":
    unittest.main()
