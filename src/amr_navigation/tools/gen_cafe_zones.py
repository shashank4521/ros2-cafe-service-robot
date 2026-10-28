#!/usr/bin/env python3
"""Make maps/cafe_zones.{pgm,yaml}: no-go zones for things the mast LiDAR can't see.

The service robot's LiDAR scans at 1.25 m, so tables (0.75 m), chairs (0.9 m), the
counter base and the low charging dock are missing from cafe.pgm. Real service
robots solve this the same way: an operator draws no-go zones on the map.
Nav2 loads this image as an extra costmap layer (black = never drive here).

Zones come from the same LAYOUT used to build the world, so they always match:
    python3 ~/amr_ws/src/amr_navigation/tools/gen_cafe_zones.py
Re-run after changing the cafe layout or re-mapping, then rebuild amr_navigation.
"""

import os
import sys

import yaml
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
MAPS = os.path.join(HERE, '..', 'maps')
sys.path.insert(0, os.path.join(HERE, '..', '..', 'amr_bringup', 'tools'))
import gen_cafe_world as cafe  # noqa: E402  (the world layout)

PAD = 0.0   # no extra margin: the robot footprint has 10 cm padding in Nav2


def zones():
    out = []
    chair_reach = cafe.CHAIR_OFFSET + 0.25          # chair back edge from table centre
    for name, (x, y) in cafe.TABLES.items():         # table + its two chairs
        out.append((name, x - chair_reach, x + chair_reach, y - 0.42, y + 0.42))
    c = cafe.COUNTER                                 # counter base (display case is mapped)
    out.append(('counter', c['x0'] - 0.02, c['x1'], c['y0'] - 0.02, c['y1'] + 0.02))
    dx, dy = cafe.DOCK                               # charging dock body (0.3 m high)
    out.append(('charging_dock', dx - 0.2, dx + 0.2, dy - 0.1, dy + 0.1))
    return out


def main():
    meta = yaml.safe_load(open(os.path.join(MAPS, 'cafe.yaml')))
    base = Image.open(os.path.join(MAPS, meta['image']))
    w, h = base.size
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]

    img = Image.new('L', (w, h), 254)                # 254 = free = "no extra info"
    px = img.load()
    for name, x0, x1, y0, y1 in zones():
        c0, c1 = int((x0 - PAD - ox) / res), int((x1 + PAD - ox) / res)
        r0, r1 = int((y0 - PAD - oy) / res), int((y1 + PAD - oy) / res)
        for col in range(max(c0, 0), min(c1 + 1, w)):
            for row in range(max(r0, 0), min(r1 + 1, h)):
                px[col, h - 1 - row] = 0             # image row 0 is the top (max y)
        print(f'zone {name:14s} x {x0 - PAD:+.2f}..{x1 + PAD:+.2f}  y {y0 - PAD:+.2f}..{y1 + PAD:+.2f}')

    img.save(os.path.join(MAPS, 'cafe_zones.pgm'))
    zmeta = dict(meta, image='cafe_zones.pgm', mode='trinary')
    with open(os.path.join(MAPS, 'cafe_zones.yaml'), 'w') as f:
        yaml.safe_dump(zmeta, f, sort_keys=False)
    print('wrote', os.path.normpath(os.path.join(MAPS, 'cafe_zones.yaml')))


if __name__ == '__main__':
    main()
