# SPDX-License-Identifier: MIT
"""
Default chassis model for the openbricks sim.

Produces an MJCF **fragment** (no surrounding ``<mujoco>`` tags) that
can be spliced into any world MJCF by :mod:`openbricks_sim.world`. The
fragment covers:

* A rigid chassis body (box) with a ``<freejoint/>`` so the robot can
  move freely on the world's floor.
* Two drive wheels attached via hinge joints, with position/velocity
  sensors a motor-shim can read as encoder counts.
* A passive caster ball at the rear for balance.
* A downward-facing camera over the chassis front (for the color
  sensor shim, Phase D).
* An inertial sensor (accelerometer + gyro) at the chassis centre
  (for the BNO055 shim).
* Torque actuators on the two drive wheels, named ``motor_left`` and
  ``motor_right`` — the shim layer (Phase C) will set control
  values on these per tick.

The numbers follow a small-scale educational robot: 60 mm wheel
diameter, 150 mm axle length, 120 × 140 mm body, 0.5 kg total mass.
Any of them can be overridden when the user instantiates a DriveBase;
until Phase C wires that through, this default is what lands.
"""

from dataclasses import dataclass


@dataclass
class ChassisSpec:
    """Geometry + mass for the default chassis. All metres / kilograms."""

    # Body.
    body_length:   float = 0.140
    body_width:    float = 0.120
    body_height:   float = 0.050
    body_mass:     float = 0.400

    # Wheels.
    wheel_radius:  float = 0.030     # 60 mm diameter
    wheel_width:   float = 0.012
    wheel_mass:    float = 0.050     # per wheel
    axle_length:   float = 0.150     # wheel-to-wheel distance

    # Caster (rear stability).
    caster_radius: float = 0.012
    caster_offset: float = 0.060     # behind chassis centre
    caster_mass:   float = 0.020

    # Motor actuator limits (torque, Nm).
    motor_gear:    float = 1.0
    motor_ctrlrange_min: float = -0.5
    motor_ctrlrange_max: float =  0.5

    # Pose: where to drop the chassis in the world. Caller can
    # override by regenerating the fragment with a different origin.
    pos_x: float = 0.0
    pos_y: float = 0.0


def chassis_mjcf(spec: ChassisSpec = None, name: str = "chassis") -> str:
    """Return an MJCF snippet describing the default chassis.

    The snippet has **one** ``<worldbody>`` ``<body>`` at the top
    level (the chassis root) plus sibling ``<actuator>`` and
    ``<sensor>`` sections. :func:`openbricks_sim.world.load_world`
    splices them into the outer ``<worldbody>`` and the global
    ``<actuator>``/``<sensor>`` sections respectively.

    The returned string is pure XML fragments — no ``<mujoco>`` root
    element and no XML declaration.
    """
    if spec is None:
        spec = ChassisSpec()

    # Centre-of-mass z so the chassis sits on its wheels' bottoms
    # plus a 5 mm ground clearance at rest.
    ground_clearance = 0.005
    chassis_z = spec.wheel_radius + ground_clearance

    # Half-extents (MuJoCo's box ``size`` is half-lengths).
    bx = spec.body_length / 2
    by = spec.body_width  / 2
    bz = spec.body_height / 2

    # Wheel positions (on the side of the body).
    wheel_y = spec.axle_length / 2
    wheel_x = 0.0                             # axle through body centre
    # Caster: behind (negative X) the chassis.
    caster_x = -spec.caster_offset
    caster_z_local = -bz - spec.caster_radius + 0.001  # tangent with ground

    body = (
        '  <worldbody>\n'
        '    <!-- Default openbricks-sim chassis. Override geometry by\n'
        '         calling chassis_mjcf(ChassisSpec(...)) at load time. -->\n'
        '    <body name="{name}" pos="{px:.4f} {py:.4f} {cz:.4f}">\n'
        '      <freejoint name="{name}_free"/>\n'
        '      <!-- Inertial tag so MuJoCo doesn\'t derive mass from geoms alone. -->\n'
        '      <inertial pos="0 0 0" mass="{bm:.3f}"\n'
        '                diaginertia="0.002 0.002 0.002"/>\n'
        '      <geom name="{name}_body" type="box"\n'
        '            size="{bx:.4f} {by:.4f} {bz:.4f}"\n'
        '            rgba="0.10 0.50 0.90 1.0"/>\n'
        '      <!-- Left drive wheel -->\n'
        '      <body name="{name}_wheel_l" pos="{wx:.4f}  {wy:.4f} {wz_offset:.4f}">\n'
        '        <joint name="{name}_hinge_l" type="hinge" axis="0 1 0"\n'
        '               damping="0.001" frictionloss="0.0005"/>\n'
        '        <inertial pos="0 0 0" mass="{wm:.3f}"\n'
        '                  diaginertia="1e-5 1e-5 1e-5"/>\n'
        '        <geom type="cylinder" size="{wr:.4f} {ww:.4f}"\n'
        '              euler="90 0 0"\n'
        '              rgba="0.10 0.10 0.10 1.0"\n'
        '              friction="0.9 0.02 0.0001"/>\n'
        '      </body>\n'
        '      <!-- Right drive wheel -->\n'
        '      <body name="{name}_wheel_r" pos="{wx:.4f} -{wy:.4f} {wz_offset:.4f}">\n'
        '        <joint name="{name}_hinge_r" type="hinge" axis="0 1 0"\n'
        '               damping="0.001" frictionloss="0.0005"/>\n'
        '        <inertial pos="0 0 0" mass="{wm:.3f}"\n'
        '                  diaginertia="1e-5 1e-5 1e-5"/>\n'
        '        <geom type="cylinder" size="{wr:.4f} {ww:.4f}"\n'
        '              euler="90 0 0"\n'
        '              rgba="0.10 0.10 0.10 1.0"\n'
        '              friction="0.9 0.02 0.0001"/>\n'
        '      </body>\n'
        '      <!-- Rear caster (passive ball, no motor) -->\n'
        '      <body name="{name}_caster" pos="{cx:.4f} 0 {cz_local:.4f}">\n'
        '        <joint name="{name}_caster_x" type="hinge" axis="1 0 0"/>\n'
        '        <joint name="{name}_caster_y" type="hinge" axis="0 1 0"/>\n'
        '        <inertial pos="0 0 0" mass="{cm:.3f}"\n'
        '                  diaginertia="1e-6 1e-6 1e-6"/>\n'
        '        <geom type="sphere" size="{cr:.4f}"\n'
        '              rgba="0.60 0.60 0.60 1.0"\n'
        '              friction="0.3 0.02 0.0001"/>\n'
        '      </body>\n'
        '      <!-- Downward colour-sensor camera (for TCS34725 shim).\n'
        '           xyaxes: image-right = body -Y, image-up = body +X.\n'
        '           Cross product gives camera +Z = body +Z, so the\n'
        '           camera looks along body -Z (straight down). -->\n'
        '      <camera name="{name}_cam_down" pos="{front:.4f} 0 {cam_z:.4f}"\n'
        '              xyaxes="0 -1 0 1 0 0" fovy="20"/>\n'
        '      <!-- IMU sensor site for accel / gyro readouts. -->\n'
        '      <site name="{name}_imu" pos="0 0 0" size="0.005"/>\n'
        '    </body>\n'
        '  </worldbody>\n'
    ).format(
        name=name,
        px=spec.pos_x, py=spec.pos_y, cz=chassis_z,
        bm=spec.body_mass,
        bx=bx, by=by, bz=bz,
        wx=wheel_x, wy=wheel_y,
        wz_offset=-bz - (spec.wheel_radius - bz - ground_clearance),
        wm=spec.wheel_mass, wr=spec.wheel_radius, ww=spec.wheel_width / 2,
        cx=caster_x, cz_local=caster_z_local,
        cm=spec.caster_mass, cr=spec.caster_radius,
        front=bx - 0.010, cam_z=-bz + 0.002,
    )

    actuators = (
        '  <actuator>\n'
        '    <motor name="{name}_motor_l" joint="{name}_hinge_l"\n'
        '           gear="{g:.2f}" ctrllimited="true"\n'
        '           ctrlrange="{lo:.3f} {hi:.3f}"/>\n'
        '    <motor name="{name}_motor_r" joint="{name}_hinge_r"\n'
        '           gear="{g:.2f}" ctrllimited="true"\n'
        '           ctrlrange="{lo:.3f} {hi:.3f}"/>\n'
        '  </actuator>\n'
    ).format(
        name=name, g=spec.motor_gear,
        lo=spec.motor_ctrlrange_min, hi=spec.motor_ctrlrange_max,
    )

    sensors = (
        '  <sensor>\n'
        '    <!-- Encoder equivalents (wheel angle + angular velocity). -->\n'
        '    <jointpos name="{name}_enc_l"    joint="{name}_hinge_l"/>\n'
        '    <jointpos name="{name}_enc_r"    joint="{name}_hinge_r"/>\n'
        '    <jointvel name="{name}_encvel_l" joint="{name}_hinge_l"/>\n'
        '    <jointvel name="{name}_encvel_r" joint="{name}_hinge_r"/>\n'
        '    <!-- BNO055-equivalent accel + gyro at the body centre. -->\n'
        '    <accelerometer name="{name}_accel" site="{name}_imu"/>\n'
        '    <gyro          name="{name}_gyro"  site="{name}_imu"/>\n'
        '  </sensor>\n'
    ).format(name=name)

    return body + actuators + sensors


def standalone_mjcf(spec: ChassisSpec = None, name: str = "chassis") -> str:
    """Wrap :func:`chassis_mjcf` with a bare ``<mujoco>`` envelope and a
    ground plane so the chassis can be previewed in isolation without
    a world file.

    Useful for unit tests + quick "does the chassis sit upright?"
    sanity checks.
    """
    fragment = chassis_mjcf(spec, name=name)
    return (
        '<mujoco model="openbricks_sim_chassis_preview">\n'
        '  <option timestep="0.001" iterations="20" solver="Newton"/>\n'
        '  <asset>\n'
        '    <texture name="grid" type="2d" builtin="checker"\n'
        '             rgb1="0.8 0.8 0.8" rgb2="0.9 0.9 0.9"\n'
        '             width="300" height="300"/>\n'
        '    <material name="grid" texture="grid" texrepeat="4 4"/>\n'
        '  </asset>\n'
        + fragment.replace(
            '  <worldbody>\n',
            '  <worldbody>\n'
            '    <light pos="0 0 1.5" dir="0 0 -1"/>\n'
            '    <geom name="floor" type="plane" size="1 1 0.1" material="grid"/>\n',
            1)
        + '</mujoco>\n'
    )
