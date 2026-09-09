Motor drivers
=============

Every motor class implements the same ``Motor`` contract
(:doc:`interfaces`), so higher layers — and your code — swap motor
types without changes. Speeds are output-shaft degrees per second,
angles are degrees, ``dc()`` duty is -100..100.

.. code-block:: python

    from openbricks.drivers.st3032 import ST3032Motor

    m = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
    m.run_speed(120)          # wheel mode: spin at 120 deg/s
    print(m.angle())          # multi-turn accumulated degrees
    m.coast()

    from openbricks.drivers.jgb37_520 import JGB37Motor

    e = JGB37Motor(in1=12, in2=14, pwm=27, encoder_a=18, encoder_b=19)
    e.run_angle(360, 720)     # two turns at 360 deg/s, blocking

Serial bus servos (recommended)
-------------------------------

ST-3032
^^^^^^^

.. automodule:: openbricks.drivers.st3032
   :members:
   :undoc-members:
   :show-inheritance:

ST-3215
^^^^^^^

.. automodule:: openbricks.drivers.st3215
   :members:
   :undoc-members:
   :show-inheritance:

DC gear motors with encoders
----------------------------

JGB37-520
^^^^^^^^^

.. automodule:: openbricks.drivers.jgb37_520
   :members:
   :undoc-members:
   :show-inheritance:

MG370
^^^^^

.. automodule:: openbricks.drivers.mg370
   :members:
   :undoc-members:
   :show-inheritance:

H-bridge drivers (open loop)
----------------------------

L298N
^^^^^

.. automodule:: openbricks.drivers.l298n
   :members:
   :undoc-members:
   :show-inheritance:

TB6612FNG
^^^^^^^^^

.. automodule:: openbricks.drivers.tb6612
   :members:
   :undoc-members:
   :show-inheritance:
