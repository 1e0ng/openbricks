# Hardware datasheets

Vendor datasheets for components we ship drivers for. Checked in so
the bench / driver / firmware can always cite an authoritative spec
instead of relying on memory or web search.

| File                       | Component                                    | Driver                          |
|----------------------------|----------------------------------------------|---------------------------------|
| `feetech_sts3032.pdf`      | Feetech STS3032 / ST-3032-C062 serial bus servo (12 V, 10 kg·cm, 148 RPM no-load) | `openbricks.drivers.st3032`     |

When adding a new driver for off-the-shelf hardware, drop the
vendor datasheet here and cite the relevant section in the driver
docstring. PDFs are small enough to live in-tree; the alternative
("look it up on the vendor site") rots when vendors reorganise.
