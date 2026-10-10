"""CAD-equipped vehicle adapted from rc-sim; positions and masses are provisional."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .vehicle import Parameters, make_xml


def numbers(values):
    return ' '.join(str(float(v)) for v in values)


def positive_mass(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('Part mass must be finite and positive')
    return str(value)


def vector(value, positive=False):
    value = np.asarray(value, dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all() or (positive and np.any(value <= 0)):
        raise ValueError('Expected three finite coordinates (positive for dimensions)')
    return value


def vehicle_xml(asset_root, p=Parameters()):
    ROOT = Path(asset_root)
    config = json.loads((ROOT / 'vehicle.json').read_text())
    manifest = json.loads((ROOT / 'tt02_cad/manifest.json').read_text())
    xml = ET.fromstring(make_xml(p, course=False, map_name="empty"))
    xml.set('model', 'Personal TT-02 with D455 and SilkyEvCam')
    chassis = xml.find("./worldbody/body[@name='chassis']")
    clearance = float(config['lower_deck_clearance_m'])
    height = float(config['standoff_length_m'])
    if not math.isfinite(clearance) or clearance <= 0 or not math.isfinite(height) or height <= 0:
        raise ValueError('Ground clearance and standoff length must be finite and positive')
    deck_half_height = 0.012
    chassis_height = clearance + deck_half_height
    chassis.set('pos', numbers([0, 0, chassis_height + 0.005]))
    for wheel in ('fl', 'fr', 'rl', 'rr'):
        knuckle = chassis.find(f"body[@name='knuckle_{wheel}']")
        pos = [float(v) for v in knuckle.get('pos').split()]
        pos[2] = p.tire_radius - chassis_height
        knuckle.set('pos', numbers(pos))
    # Retain the existing drive battery and motor silhouettes, with neutral colors.
    for geom in chassis.findall('geom'):
        geom.set('rgba', '0.06 0.06 0.06 1')
    chassis.remove(chassis.findall('geom')[-1])  # old yellow direction marker
    yaw = math.radians(float(config['cad_yaw_deg']))
    if not math.isfinite(yaw):
        raise ValueError('CAD yaw must be finite')
    assembly = ET.SubElement(chassis, 'body', name='cad_assembly',
                             pos=numbers([0, 0, deck_half_height + height]),
                             quat=numbers([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]))
    for part in manifest:
        name = part['name']
        ET.SubElement(xml.find('asset'), 'mesh', name=name,
                      file=str(ROOT / 'tt02_cad' / (name + '.obj')))
        body = ET.SubElement(assembly, 'body', name=name)
        color = '0.55 0.55 0.55 1' if name == 'D455Envelope' else '0.06 0.06 0.06 1'
        ET.SubElement(body, 'geom', name=name + '_visual', type='mesh', mesh=name,
                      rgba=color, mass='0', contype='0', conaffinity='0', group='2')
        bounds = np.asarray(part['bounds_mm']).reshape(2, 3) / 1000
        ET.SubElement(body, 'geom', name=name + '_collision', type='box',
                      pos=numbers(bounds.mean(axis=0)),
                      size=numbers((bounds[1] - bounds[0]) / 2),
                      mass=positive_mass(config['part_masses_kg'][name]),
                      rgba='0 0 0 0', group='3')
    jetson = config['jetson']
    board = ET.SubElement(chassis, 'body', name='jetson', pos=numbers(vector(jetson['position_m'])))
    ET.SubElement(board, 'geom', name='jetson_case', type='box',
                  size=numbers(vector(jetson['size_m'], positive=True) / 2),
                  mass=positive_mass(jetson['mass_kg']), rgba='0.055 0.055 0.055 1')
    # The CAD plate is mounted above the simplified drivetrain on four rigid posts.
    for x in (-0.085, 0.085):
        for y in (-0.043, 0.043):
            ET.SubElement(chassis, 'geom', type='cylinder',
                          pos=numbers([x, y, 0.012 + height / 2]),
                          size=numbers([0.003, height / 2]), mass='0',
                          contype='0', conaffinity='0', rgba='0.08 0.08 0.08 1')
    return xml
