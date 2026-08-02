Sensor drivers
==============

All sensors construct against a ``machine.I2C`` bus (or a
:class:`~openbricks.drivers.tca9548a.TCA9548A` mux channel, which
quacks the same) except the GPIO-driven HC-SR04.

.. code-block:: python

    from machine import I2C, Pin
    from openbricks.drivers.bno055 import BNO055
    from openbricks.drivers.tcs34725 import TCS34725
    from openbricks.drivers.tca9548a import TCA9548A

    i2c = I2C(0, sda=Pin(15), scl=Pin(16), freq=400_000)  # ESP32-S3 pins
    mux = TCA9548A(i2c)

    color = TCS34725(mux[0])          # two same-address sensors ...
    color2 = TCS34725(mux[1])         # ... on separate mux channels
    imu = BNO055(i2c=mux[3], address=0x29)

    print(color.rgb())                # (r, g, b) each 0-255
    print(color.ambient())            # clear channel, 0-100
    print(imu.heading())              # degrees, CW-positive

BNO055 (IMU)
------------

.. automodule:: openbricks.drivers.bno055

The class itself is implemented in C (so the drivebase can read the
heading on its 1 kHz tick); its Python-facing API:

.. py:class:: BNO055(i2c, address=0x28)

   Bosch BNO055 9-axis IMU in 6-DOF fusion mode (accelerometer +
   gyro; the magnetometer is deliberately unused — motor magnets and
   steel in floors bend the local field, and a drive robot only
   needs *relative* heading). Heading zeroes where the robot points
   at construction.

   :param i2c: ``machine.I2C`` (or a mux channel).
   :param address: 0x28, or 0x29 on breakouts whose ADR pin straps
      high.

   .. py:method:: heading()

      Body heading in degrees, wrapped to [-180, 180). CW-positive:
      turning right (clockwise viewed from above) increases it —
      compass and Pybricks convention. This is what
      ``DriveBase(imu=...)`` reads.

   .. py:method:: euler()

      ``(heading, roll, pitch)`` tuple in degrees.

   .. py:method:: angular_velocity()

      ``(x, y, z)`` gyro rates in deg/s.

   .. py:method:: acceleration()

      ``(x, y, z)`` accelerometer in m/s².

TCS34725 (color)
----------------

.. automodule:: openbricks.drivers.tcs34725
   :members:
   :undoc-members:
   :show-inheritance:

HC-SR04 (ultrasonic distance)
-----------------------------

.. automodule:: openbricks.drivers.hcsr04
   :members:
   :undoc-members:
   :show-inheritance:

VL53L0X (laser distance)
------------------------

.. automodule:: openbricks.drivers.vl53l0x
   :members:
   :undoc-members:
   :show-inheritance:

VL53L1X (laser distance, long range)
------------------------------------

.. automodule:: openbricks.drivers.vl53l1x
   :members:
   :undoc-members:
   :show-inheritance:
