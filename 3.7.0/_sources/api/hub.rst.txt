Hubs
====

Board-level peripherals — the onboard LED and buttons — behind one
``Hub`` object per supported devkit.

.. code-block:: python

    from openbricks.hub import ESP32S3DevkitHub

    hub = ESP32S3DevkitHub()
    hub.led.rgb(0, 80, 0)          # onboard NeoPixel: dim green
    if hub.bluetooth_button.pressed():
        print("button held at boot")

.. automodule:: openbricks.hub
   :members:
   :undoc-members:
   :show-inheritance:
