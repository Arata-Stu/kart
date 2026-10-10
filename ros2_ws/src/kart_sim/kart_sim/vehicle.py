"""TT-02 chassis and Ackermann actuators adapted from rc-sim; provisional dynamics."""
import math
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Parameters:
    # Tamiya 58682 specifications; tires differ between TT-02 kits.
    wheelbase: float = 0.257
    tire_radius: float = 0.033
    tire_width: float = 0.026
    # Prototype assumptions: replace with measurements of the actual car.
    track: float = 0.160
    chassis_mass: float = 1.20
    wheel_mass: float = 0.040
    max_steer: float = 0.45
    wheel_torque: float = 0.025
    tire_friction: float = 1.0


def make_xml(p=Parameters(), course=True, jetracer=False, map_name="duct"):
    if map_name not in ("duct", "empty"):
        raise ValueError(f"Unknown map: {map_name}")
    wheels, actuators = [], []
    for name, x, y in (
        ("fl", p.wheelbase / 2, p.track / 2),
        ("fr", p.wheelbase / 2, -p.track / 2),
        ("rl", -p.wheelbase / 2, p.track / 2),
        ("rr", -p.wheelbase / 2, -p.track / 2),
    ):
        steer = ""
        if name.startswith("f"):
            steer = f'<joint name="steer_{name}" axis="0 0 1" range="-0.65 0.65" damping="0.02"/>'
            actuators.append(f'<position name="steer_{name}" joint="steer_{name}" kp="5" kv="0.15" ctrlrange="-0.65 0.65" forcerange="-0.3 0.3"/>')
        wheels.append(f'''
        <body name="knuckle_{name}" pos="{x} {y} -0.015">
          {steer}
          <inertial pos="0 0 0" mass="0.01" diaginertia="0.000002 0.000002 0.000002"/>
          <body name="wheel_{name}">
            <joint name="spin_{name}" axis="0 1 0" damping="0.00015"/>
            <geom name="tire_{name}" type="cylinder" size="{p.tire_radius} {p.tire_width / 2}"
                  quat="0.70710678 0.70710678 0 0" mass="{p.wheel_mass}"
                  friction="{p.tire_friction} 0.002 0.0001" condim="6" rgba="0.08 0.08 0.09 1"/>
            <geom type="box" size="0.024 0.014 0.003" mass="0" contype="0" conaffinity="0" rgba="0.8 0.8 0.85 1"/>
          </body>
        </body>''')
        actuators.append(f'<motor name="drive_{name}" joint="spin_{name}" gear="{p.wheel_torque}" ctrlrange="-1 1"/>')
    ducts = ""
    if course and map_name == "duct":
        points = [(-1, -1), (3, -1), (3, 2), (-1, 2), (-1, -1)]
        for a, b in zip(points, points[1:]):
            ducts += f'<geom type="capsule" fromto="{a[0]} {a[1]} 0.05 {b[0]} {b[1]} 0.05" size="0.05" rgba="1 0.3 0.02 1"/>'
    xml = f'''
    <mujoco model="TT-02 prototype">
      <compiler angle="radian"/>
      <option timestep="0.001" integrator="implicitfast"/>
      <default><geom solref="0.01 1"/></default>
      <asset>
        <texture name="grid" type="2d" builtin="checker" rgb1="0.24 0.28 0.30" rgb2="0.30 0.34 0.36" width="512" height="512"/>
        <material name="floor" texture="grid" texrepeat="10 10" reflectance="0.1"/>
      </asset>
      <worldbody>
        <light pos="0 0 4" dir="0 0 -1"/>
        <geom name="floor" type="plane" size="10 10 0.1" material="floor"/>
        {ducts}
        <body name="chassis" pos="0 0 {p.tire_radius + 0.015 + 0.005}">
          <freejoint/>
          <geom type="box" size="0.16 0.055 0.012" mass="{p.chassis_mass}" rgba="0.08 0.2 0.65 1"/>
          <geom type="box" pos="0 0.028 0.024" size="0.065 0.019 0.012" mass="0" contype="0" conaffinity="0" rgba="0.2 0.2 0.22 1"/>
          <geom type="box" pos="-0.055 -0.028 0.022" size="0.025 0.019 0.012" mass="0" contype="0" conaffinity="0" rgba="0.7 0.7 0.73 1"/>
          <geom type="box" pos="0.15 0 0.018" size="0.012 0.04 0.006" mass="0" contype="0" conaffinity="0" rgba="1 0.8 0.1 1"/>
          {''.join(wheels)}
          
        </body>
      </worldbody>
      <actuator>{''.join(actuators)}</actuator>
    </mujoco>'''
    return xml

def control(model, data, throttle, steering, p=Parameters()):
    """Normalized inputs. Ideal Ackermann steering and equal wheel torques.

    This is not a motor/ESC or shaft/differential dynamics model.
    """
    delta = float(np.clip(steering, -1, 1)) * p.max_steer
    for side, sign in (("fl", 1), ("fr", -1)):
        angle = math.atan2(p.wheelbase * math.tan(delta),
                           p.wheelbase - sign * p.track / 2 * math.tan(delta))
        data.ctrl[model.actuator(f"steer_{side}").id] = angle
    for wheel in ("fl", "fr", "rl", "rr"):
        data.ctrl[model.actuator(f"drive_{wheel}").id] = np.clip(throttle, -1, 1)

