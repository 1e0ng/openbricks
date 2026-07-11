# Examples

The repo ships runnable example programs in
[`examples/`](https://github.com/1e0ng/openbricks/tree/main/examples).
Push any of them to a hub with `openbricks run -n <name> <script>` or
try them in the simulator with `openbricks sim run <script>`.

Two representative ones are reproduced below.

## Drive a square (ST-3032 drivebase)

```{eval-rst}
.. literalinclude:: ../examples/st3032_drivebase_square.py
   :language: python
```

## Square up on a line (two color sensors)

```{eval-rst}
.. literalinclude:: ../examples/line_align.py
   :language: python
```

## Full robot (drivebase + IMU + color sensor array)

```{eval-rst}
.. literalinclude:: ../examples/full_robot.py
   :language: python
```
