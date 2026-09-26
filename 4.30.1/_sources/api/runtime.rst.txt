Runtime services
================

These modules run behind the scenes on the hub — the frozen ``main.py``
wires them up at boot. They're documented here because their behavior
(button semantics, log rotation, BLE persistence) is user-visible.

Reserved-GPIO guard
-------------------

.. automodule:: openbricks.pins
   :members:
   :undoc-members:

Program launcher
----------------

.. automodule:: openbricks.launcher
   :members:
   :undoc-members:

Run logs
--------

.. automodule:: openbricks.log
   :members:
   :undoc-members:

Bluetooth (BLE REPL & console)
------------------------------

.. automodule:: openbricks.bluetooth
   :members:
   :undoc-members:

.. automodule:: openbricks.ble_repl
   :members:
   :undoc-members:

BLE toggle button
-----------------

.. automodule:: openbricks.bluetooth_button
   :members:
   :undoc-members:
