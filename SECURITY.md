# Security Policy

## Reporting a vulnerability

Please report vulnerabilities **privately** through GitHub's advisory
form:

**<https://github.com/1e0ng/openbricks/security/advisories/new>**

(Repository → **Security** tab → **Report a vulnerability**.)

Do not open a public issue for a security problem — issues are visible
immediately, before a fix can ship. There is no email reporting
channel; the advisory form is the only intake, and it notifies the
maintainer directly.

You can expect an acknowledgement within a few days. Confirmed
vulnerabilities are fixed in the next release (openbricks ships
rolling releases, so that is typically days, not months), and you
will be credited in the advisory unless you ask not to be.

## Supported versions

Firmware and the `openbricks` PyPI package share one version number
and release together. Only the **latest release** receives security
fixes — the fix for any confirmed report is a new release, never a
backport. If you are behind, update with `pipx upgrade openbricks`
and reflash the firmware.

## Scope

Reports are welcome for anything in this repository, including:

- the ESP32 firmware and its BLE interface (device control, program
  upload/run),
- firmware release **signing and verification** (Ed25519) — a way to
  pass off unsigned or tampered firmware as official is the highest
  severity issue this project can have,
- the host CLI (`openbricks flash/run/...`) and simulator,
- the release pipeline (CI workflows, PyPI publishing, GitHub
  release assets).

Out of scope: vulnerabilities in upstream MicroPython itself (report
those to [micropython/micropython](https://github.com/micropython/micropython/security));
physical attacks on a hub you already hold (the hardware has no
secure element — anyone with USB access can reflash it, by design);
and denial of service against a robot on your own bench.
