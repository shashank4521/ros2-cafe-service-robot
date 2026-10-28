#!/usr/bin/env python3
"""Send the service robot to named cafe places with Nav2 and report how it went.

    python3 ~/amr_ws/src/amr_navigation/tools/cafe_goals.py                    # default tour
    python3 ~/amr_ws/src/amr_navigation/tools/cafe_goals.py counter table_3 home
    python3 ~/amr_ws/src/amr_navigation/tools/cafe_goals.py counter box=1.6,0,0.5   # + dropped box

Needs: ros2 launch amr_bringup sim.launch.py robot:=service world:=cafe nav:=true
For each goal it prints: result, time, final error vs the goal (Gazebo truth),
AMCL error vs truth, and the closest the robot's outline came to real furniture/walls.
"""

import math
import re
import subprocess
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

# Named places come from config/cafe_places.yaml (shared with delivery.py).
def load_places():
    import os
    import yaml
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('amr_navigation'), 'config', 'cafe_places.yaml')
    raw = yaml.safe_load(open(path))
    return {k: (float(v['x']), float(v['y']), math.radians(float(v['yaw']))) for k, v in raw.items()}


PLACES = load_places()
DEFAULT_TOUR = ['counter', 'table_6', 'table_1', 'counter', 'table_5', 'charger', 'home']

# Real obstacles (axis-aligned boxes x0, x1, y0, y1) for the clearance check
TABLE_XY = {'table_1': (-4.3, 2.3), 'table_2': (-1.5, 2.3), 'table_3': (1.3, 2.3),
            'table_4': (-4.3, -2.3), 'table_5': (-1.5, -2.3), 'table_6': (1.3, -2.3)}
OBSTACLES = [(f'{n}+chairs', x - 0.86, x + 0.86, y - 0.40, y + 0.40) for n, (x, y) in TABLE_XY.items()]
OBSTACLES += [('pillar', -1.7, -1.3, -0.2, 0.2), ('counter', 5.4, 6.0, -2.0, 2.0),
              ('charging_dock', 4.3, 4.7, -3.95, -3.75),
              ('partition_n', 2.925, 3.075, 0.6, 4.0), ('partition_s', 2.925, 3.075, -4.0, -0.6),
              ('wall_n', -6, 6, 4.0, 4.2), ('wall_s', -6, 6, -4.2, -4.0),
              ('wall_e', 6.0, 6.2, -4, 4), ('wall_w', -6.2, -6.0, -4, 4)]
FOOT = [(0.50, 0.26), (0.50, -0.26), (-0.10, -0.26), (-0.10, 0.26)]  # real body outline


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def gz_truth():
    out = subprocess.run(['gz', 'model', '-m', 'service_robot', '-p'],
                         capture_output=True, text=True, timeout=20).stdout
    v = [float(x) for x in re.findall(r'-?\d+\.\d+', out[out.index('Pose [ XYZ'):])][:6]
    return v[0], v[1], v[5]


def clearance(x, y, yaw):
    """Smallest gap (m) between the robot outline (sampled) and any obstacle box."""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for (ax, ay), (bx, by) in zip(FOOT, FOOT[1:] + FOOT[:1]):
        for k in range(6):
            px, py = ax + (bx - ax) * k / 6, ay + (by - ay) * k / 6
            pts.append((x + c * px - s * py, y + s * px + c * py))
    best = (99.0, '')
    for name, x0, x1, y0, y1 in OBSTACLES:
        for px, py in pts:
            dx = max(x0 - px, 0, px - x1)
            dy = max(y0 - py, 0, py - y1)
            d = math.hypot(dx, dy)
            if d < best[0]:
                best = (d, name)
    return best


def pose_stamped(nav, x, y, yaw):
    g = PoseStamped()
    g.header.frame_id = 'map'
    g.header.stamp = nav.get_clock().now().to_msg()
    g.pose.position.x, g.pose.position.y = x, y
    g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return g


def main():
    args = sys.argv[1:]
    # optional extra obstacle for the clearance check: box=x,y,size  (e.g. one you dropped in)
    for a in [a for a in args if a.startswith('box=')]:
        bx, by, bs = (float(v) for v in a[4:].split(','))
        OBSTACLES.append(('dropped_box', bx - bs / 2, bx + bs / 2, by - bs / 2, by + bs / 2))
    tour = [a for a in args if not a.startswith('box=')] or DEFAULT_TOUR
    unknown = [t for t in tour if t not in PLACES]
    if unknown:
        sys.exit(f'unknown place(s) {unknown}; choose from {list(PLACES)}')

    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, nav)
    nav.waitUntilNav2Active(localizer='amcl')

    ok = 0
    for name in tour:
        gx, gy, gyaw = PLACES[name]
        print(f'\n== {name}: ({gx:+.2f}, {gy:+.2f}, yaw {gyaw:+.2f})', flush=True)
        t0 = nav.get_clock().now().nanoseconds * 1e-9
        nav.goToPose(pose_stamped(nav, gx, gy, gyaw))
        closest, wall_t = (99.0, '', (0, 0, 0)), time.time()
        while not nav.isTaskComplete():
            rclpy.spin_once(nav, timeout_sec=0.1)
            try:
                tf = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
                px, py, pyaw = (tf.transform.translation.x, tf.transform.translation.y,
                                yaw_of(tf.transform.rotation))
                cl = clearance(px, py, pyaw)
                if cl[0] < closest[0]:
                    closest = (cl[0], cl[1], (px, py, pyaw))
            except tf2_ros.TransformException:
                pass
            if time.time() - wall_t > 300:
                nav.cancelTask()
                print('  timeout, cancelled', flush=True)
        res = nav.getResult()
        dt = nav.get_clock().now().nanoseconds * 1e-9 - t0
        time.sleep(0.5)
        tx, ty, tyaw = gz_truth()
        tf = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        ax, ay, ayaw = tf.transform.translation.x, tf.transform.translation.y, yaw_of(tf.transform.rotation)
        dyaw = abs(math.atan2(math.sin(tyaw - gyaw), math.cos(tyaw - gyaw)))
        ok += res.name == 'SUCCEEDED'
        print(f'  result {res.name} in {dt:.0f} s', flush=True)
        print(f'  arrived {math.hypot(tx - gx, ty - gy) * 100:.1f} cm / {math.degrees(dyaw):.1f} deg from goal (truth)', flush=True)
        print(f'  AMCL error {math.hypot(ax - tx, ay - ty) * 100:.1f} cm', flush=True)
        cx, cy, cyaw = closest[2]
        print(f'  closest approach {closest[0] * 100:.0f} cm to {closest[1]} '
              f'(robot at {cx:+.2f}, {cy:+.2f}, yaw {cyaw:+.2f})', flush=True)
    print(f'\n{ok}/{len(tour)} goals succeeded', flush=True)


if __name__ == '__main__':
    main()
