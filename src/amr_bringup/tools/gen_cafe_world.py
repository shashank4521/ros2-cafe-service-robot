#!/usr/bin/env python3
"""Generate worlds/cafe.sdf (Gazebo Harmonic).

Edit the LAYOUT section and re-run:
    python3 ~/amr_ws/src/amr_bringup/tools/gen_cafe_world.py

Map frame = world frame: origin at the room centre, x towards the kitchen (east).
Room 12 x 8 m (inside faces x = -6..6, y = -4..4).
  Dining room x -6..3, kitchen x 3..6, doorway 1.2 m at y = -0.6..0.6.
Heights matter for the mast LiDAR (scans at 1.25 m): walls, pillar, partition,
fridge, shelf, counter display case and plants reach above it; tables (0.75 m)
and chairs (0.9 m) do NOT -- the depth camera and ToF sensors see those.
"""

import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'worlds', 'cafe.sdf')

# ============================== LAYOUT ==============================
ROOM_X, ROOM_Y = 6.0, 4.0          # half sizes (inside faces)
WALL_T, WALL_H = 0.2, 2.5
PARTITION_X, DOOR_HALF = 3.0, 0.6  # doorway y = -0.6..0.6

TABLE_R, TABLE_H = 0.40, 0.75
TABLES = {                          # name: (x, y)  -- 2 chairs each, east and west side
    'table_1': (-4.3, 2.3), 'table_2': (-1.5, 2.3), 'table_3': (1.3, 2.3),
    'table_4': (-4.3, -2.3), 'table_5': (-1.5, -2.3), 'table_6': (1.3, -2.3),
}
CHAIR_OFFSET = 0.65
PILLAR = (-1.5, 0.0, 0.4)           # x, y, side
COUNTER = dict(x0=5.4, x1=6.0, y0=-2.0, y1=2.0)
FRIDGE = (5.55, 3.4, 0.7, 0.7, 2.0)  # x, y, sx, sy, h
SHELF = (3.8, 3.75, 1.2, 0.4, 1.8)
DOCK = (4.5, -3.85)                 # charging dock against the south wall
DOCK_PAD = (4.5, -3.25, 0.8)        # floor pad (visual only)
PLANTS = [(-5.55, 3.55), (-5.55, -3.55)]
# ====================================================================

COL = {
    'wall': '0.93 0.89 0.80', 'floor': '0.62 0.47 0.32', 'wood': '0.55 0.36 0.20',
    'dark': '0.20 0.20 0.22', 'metal': '0.75 0.76 0.78', 'glass': '0.75 0.88 0.92',
    'green': '0.10 0.75 0.30', 'pillar': '0.80 0.78 0.74', 'chair': '0.35 0.22 0.12',
    'pot': '0.45 0.30 0.25', 'leaf': '0.15 0.50 0.15', 'counter': '0.30 0.30 0.32',
}


def mat(c):
    return (f'<material><ambient>{COL[c]} 1</ambient><diffuse>{COL[c]} 1</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular></material>')


def geom(shape):
    kind = shape[0]
    if kind == 'box':
        return f'<box><size>{shape[1]} {shape[2]} {shape[3]}</size></box>'
    if kind == 'cyl':
        return f'<cylinder><radius>{shape[1]}</radius><length>{shape[2]}</length></cylinder>'
    if kind == 'sphere':
        return f'<sphere><radius>{shape[1]}</radius></sphere>'
    raise ValueError(kind)


def part(name, pose, shape, color, collide=True):
    """One visual (+ collision) element, pose relative to the model."""
    p = ' '.join(str(round(v, 4)) for v in pose)
    s = (f'      <visual name="{name}_v"><pose>{p} 0 0 0</pose>'
         f'<geometry>{geom(shape)}</geometry>{mat(color)}</visual>\n')
    if collide:
        s += (f'      <collision name="{name}_c"><pose>{p} 0 0 0</pose>'
              f'<geometry>{geom(shape)}</geometry></collision>\n')
    return s


def model(name, x, y, yaw, parts):
    return (f'    <model name="{name}">\n      <static>true</static>\n'
            f'      <pose>{x} {y} 0 0 0 {yaw}</pose>\n      <link name="link">\n'
            f'{parts}      </link>\n    </model>\n')


def box_model(name, cx, cy, sx, sy, h, color):
    return model(name, cx, cy, 0, part('b', (0, 0, h / 2), ('box', sx, sy, h), color))


def chair(name, x, y, facing_yaw):
    # local frame: chair faces +x (towards the table); back on the -x side
    parts = part('seat', (0, 0, 0.45), ('box', 0.42, 0.42, 0.05), 'chair')
    parts += part('back', (-0.19, 0, 0.70), ('box', 0.04, 0.42, 0.45), 'chair')
    for i, (lx, ly) in enumerate([(0.18, 0.18), (0.18, -0.18), (-0.18, 0.18), (-0.18, -0.18)]):
        parts += part(f'leg{i}', (lx, ly, 0.21), ('box', 0.03, 0.03, 0.42), 'dark')
    return model(name, x, y, facing_yaw, parts)


def table(name, x, y):
    parts = part('top', (0, 0, TABLE_H - 0.02), ('cyl', TABLE_R, 0.04), 'wood')
    parts += part('stem', (0, 0, (TABLE_H - 0.04) / 2), ('cyl', 0.04, TABLE_H - 0.04), 'dark')
    parts += part('base', (0, 0, 0.015), ('cyl', 0.25, 0.03), 'dark')
    s = model(name, x, y, 0, parts)
    s += chair(f'{name}_chair_w', x - CHAIR_OFFSET, y, 0.0)
    s += chair(f'{name}_chair_e', x + CHAIR_OFFSET, y, 3.14159265)
    return s


def build():
    out = []
    X, Y, T, H = ROOM_X, ROOM_Y, WALL_T, WALL_H
    # outer walls
    out.append(box_model('wall_north', 0, Y + T / 2, 2 * X + 2 * T, T, H, 'wall'))
    out.append(box_model('wall_south', 0, -Y - T / 2, 2 * X + 2 * T, T, H, 'wall'))
    out.append(box_model('wall_east', X + T / 2, 0, T, 2 * Y, H, 'wall'))
    out.append(box_model('wall_west', -X - T / 2, 0, T, 2 * Y, H, 'wall'))
    # partition with doorway (+ lintel above 2.1 m)
    seg = Y - DOOR_HALF
    out.append(box_model('partition_north', PARTITION_X, DOOR_HALF + seg / 2, 0.15, seg, H, 'wall'))
    out.append(box_model('partition_south', PARTITION_X, -DOOR_HALF - seg / 2, 0.15, seg, H, 'wall'))
    out.append(model('door_lintel', PARTITION_X, 0, 0,
                     part('l', (0, 0, 2.1 + (H - 2.1) / 2), ('box', 0.15, 2 * DOOR_HALF, H - 2.1), 'wall')))
    # dining room
    px, py, ps = PILLAR
    out.append(box_model('pillar', px, py, ps, ps, H, 'pillar'))
    for name, (x, y) in TABLES.items():
        out.append(table(name, x, y))
    for i, (x, y) in enumerate(PLANTS):
        parts = part('pot', (0, 0, 0.25), ('cyl', 0.22, 0.5), 'pot')
        parts += part('leaves', (0, 0, 1.0), ('sphere', 0.40), 'leaf')
        out.append(model(f'plant_{i + 1}', x, y, 0, parts))
    # kitchen: counter with pastry display case (pickup point)
    c = COUNTER
    cx, cy, sx, sy = (c['x0'] + c['x1']) / 2, (c['y0'] + c['y1']) / 2, c['x1'] - c['x0'], c['y1'] - c['y0']
    parts = part('base', (0, 0, 0.45), ('box', sx, sy, 0.9), 'counter')
    parts += part('top', (0, 0, 0.92), ('box', sx + 0.04, sy, 0.04), 'wood')
    parts += part('display', (-0.05, 0, 1.165), ('box', sx - 0.15, sy - 0.2, 0.45), 'glass')
    out.append(model('kitchen_counter', cx, cy, 0, parts))
    fx, fy, fsx, fsy, fh = FRIDGE
    out.append(box_model('fridge', fx, fy, fsx, fsy, fh, 'metal'))
    sx_, sy_, ssx, ssy, sh = SHELF
    out.append(box_model('shelf_unit', sx_, sy_, ssx, ssy, sh, 'wood'))
    # charging dock + floor pad (pad has no collision)
    dx, dy = DOCK
    parts = part('body', (0, 0, 0.15), ('box', 0.4, 0.2, 0.3), 'dark')
    parts += part('led', (0, 0.101, 0.25), ('box', 0.3, 0.002, 0.03), 'green', collide=False)
    out.append(model('charging_dock', dx, dy, 0, parts))
    pdx, pdy, pds = DOCK_PAD
    out.append(model('charging_pad', pdx, pdy, 0,
                     part('pad', (0, 0, 0.001), ('box', pds, pds, 0.002), 'green', collide=False)))
    return ''.join(out)


HEADER = """<?xml version="1.0" ?>
<!-- GENERATED by src/amr_bringup/tools/gen_cafe_world.py -- edit that file, not this one. -->
<sdf version="1.10">
  <world name="cafe">

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim8-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.75 0.80 0.85 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 -0.4 -1</direction>
    </light>
    <light name="ceiling_dining" type="point">
      <pose>-1.5 0 2.4 0 0 0</pose>
      <diffuse>0.5 0.48 0.42 1</diffuse>
      <attenuation><range>12</range><constant>0.6</constant><linear>0.05</linear></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>{floor} 1</ambient><diffuse>{floor} 1</diffuse></material>
        </visual>
      </link>
    </model>

"""

if __name__ == '__main__':
    sdf = HEADER.replace('{floor}', COL['floor']) + build() + '  </world>\n</sdf>\n'
    with open(OUT, 'w') as f:
        f.write(sdf)
    print('wrote', os.path.normpath(OUT))
