"""Grab /map once, print stats, check wall positions, write a PNG."""
import sys, time, rclpy, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
rclpy.init(); n = Node('map_check'); got = []
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, '/map', lambda m: got.append(m), q)
t0 = time.time()
while not got and time.time() - t0 < 15: rclpy.spin_once(n, timeout_sec=0.5)
m = got[-1]; info = m.info
g = np.array(m.data, dtype=np.int16).reshape(info.height, info.width)
tot = g.size; occ = (g >= 65).sum(); free = (g == 0).sum(); unk = (g == -1).sum()
print(f'size {info.width}x{info.height} @ {info.resolution} m, origin ({info.origin.position.x:.2f},{info.origin.position.y:.2f})')
print(f'occupied {occ} ({100*occ/tot:.1f}%)  free {free} ({100*free/tot:.1f}%)  unknown {unk} ({100*unk/tot:.1f}%)')
ys, xs = np.nonzero(g >= 65)
wx = info.origin.position.x + (xs + 0.5) * info.resolution
wy = info.origin.position.y + (ys + 0.5) * info.resolution
# Walls expected at x=+-6, y=+-4 in map frame (robot started at world origin, yaw 0)
for name, vals, target in [('east x', wx, 6), ('west x', wx, -6), ('north y', wy, 4), ('south y', wy, -4)]:
    near = vals[np.abs(vals - target) < 0.5]
    print(f'{name}={target:+}: {len(near)} cells, mean {near.mean():+.3f}, std {near.std():.3f}' if len(near) else f'{name}: none')
inside = (np.abs(wx) < 5.5) & (np.abs(wy) < 3.5)
print('occupied cells away from walls (box + any ghosts):', inside.sum(),
      ' of which near box (3,1.5):', (inside & (np.abs(wx-3) < 0.7) & (np.abs(wy-1.5) < 0.7)).sum())
img = np.full(g.shape, 205, np.uint8); img[g == 0] = 254; img[g >= 65] = 0
img = np.flipud(img)
from PIL import Image
Image.fromarray(img).resize((info.width*3, info.height*3), Image.NEAREST).save(sys.argv[1])
print('wrote', sys.argv[1])
