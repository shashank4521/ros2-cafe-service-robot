import sys, time, rclpy, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
rclpy.init(); n = Node('cafe_map_check'); got = []
n.create_subscription(OccupancyGrid, '/map', got.append, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
t0 = time.time()
while not got and time.time() - t0 < 15: rclpy.spin_once(n, timeout_sec=0.5)
m = got[-1]; i = m.info; g = np.array(m.data, dtype=np.int16).reshape(i.height, i.width)
res, ox, oy = i.resolution, i.origin.position.x, i.origin.position.y
print(f'size {i.width}x{i.height} @ {res:.2f} m, x {ox:.2f}..{ox+i.width*res:.2f}, y {oy:.2f}..{oy+i.height*res:.2f}')
tot = g.size; print(f'occupied {(g>=65).sum()} ({100*(g>=65).mean():.1f}%)  free {(g==0).sum()} ({100*(g==0).mean():.1f}%)  unknown {(g==-1).sum()} ({100*(g==-1).mean():.1f}%)')
ys, xs = np.nonzero(g >= 65); wx = ox + (xs+.5)*res; wy = oy + (ys+.5)*res
def near(name, cond):
    print(f'  {name:42s} {int(cond.sum()):4d} occupied cells')
near('outer walls (|x|~6 or |y|~4)', (np.abs(np.abs(wx)-6) < .15) | (np.abs(np.abs(wy)-4) < .15))
near('partition x~3 (outside doorway)', (np.abs(wx-3) < .15) & (np.abs(wy) > .6))
near('doorway gap x~3, |y|<0.5 (should be 0)', (np.abs(wx-3) < .15) & (np.abs(wy) < .5))
near('pillar (-1.5, 0)', (np.abs(wx+1.5) < .35) & (np.abs(wy) < .35))
near('counter display front x~5.43, |y|<1.8', (np.abs(wx-5.43) < .15) & (np.abs(wy) < 1.8))
near('fridge (5.55, 3.4)', (np.abs(wx-5.55) < .45) & (np.abs(wy-3.4) < .45))
near('shelf unit (3.8, 3.75)', (np.abs(wx-3.8) < .7) & (np.abs(wy-3.75) < .3))
near('plants (-5.55, +-3.55)', (np.abs(wx+5.55) < .5) & (np.abs(np.abs(wy)-3.55) < .5))
tables = [(-4.3,2.3),(-1.5,2.3),(1.3,2.3),(-4.3,-2.3),(-1.5,-2.3),(1.3,-2.3)]
near('tables + chairs (expected 0: below LiDAR)', np.any([(np.abs(wx-tx)<1.0)&(np.abs(wy-ty)<.5) for tx,ty in tables], axis=0))
img = np.full(g.shape, 205, np.uint8); img[g==0] = 254; img[g>=65] = 0
from PIL import Image; Image.fromarray(np.flipud(img)).resize((i.width*3, i.height*3), Image.NEAREST).save(sys.argv[1]); print('wrote', sys.argv[1])
