"""Send test goals through Nav2 and report result, time, and pose errors vs Gazebo truth."""
import math, subprocess, re, time, rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tf2_ros

GOALS = [('A open floor', 3.0, -1.5, 0.0),
         ('B behind box', 4.6, 2.8, math.pi/2),
         ('Home', 0.0, 0.0, 0.0)]

def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def angdiff(a, b): return abs(math.atan2(math.sin(a-b), math.cos(a-b)))
def gz_pose():
    out = subprocess.run(['gz','model','-m','amr','-p'],capture_output=True,text=True,timeout=20).stdout
    i = out.index('Pose [ XYZ'); n=[float(v) for v in re.findall(r'-?\d+\.\d+', out[i:])][:6]; return n[0], n[1], n[5]

rclpy.init()
nav = BasicNavigator()
nav.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, nav)
nav.waitUntilNav2Active(localizer='amcl')

for name, gx, gy, gyaw in GOALS:
    g = PoseStamped(); g.header.frame_id = 'map'; g.header.stamp = nav.get_clock().now().to_msg()
    g.pose.position.x, g.pose.position.y = gx, gy
    g.pose.orientation.z, g.pose.orientation.w = math.sin(gyaw/2), math.cos(gyaw/2)
    path = nav.getPath(PoseStamped(), g, use_start=False)
    plen = sum(math.hypot(b.pose.position.x-a.pose.position.x, b.pose.position.y-a.pose.position.y)
               for a, b in zip(path.poses, path.poses[1:])) if path else float('nan')
    min_box = min((math.hypot(p.pose.position.x-3.0, p.pose.position.y-1.5) for p in path.poses), default=float('nan')) if path else float('nan')
    print(f'\n== {name}: goal ({gx}, {gy}, yaw {gyaw:.2f}); planned path {plen:.2f} m, closest approach to box centre {min_box:.2f} m', flush=True)
    t0 = nav.get_clock().now().nanoseconds * 1e-9; w0 = time.time()
    nav.goToPose(g)
    while not nav.isTaskComplete():
        rclpy.spin_once(nav, timeout_sec=0.5)
        if time.time() - w0 > 900: nav.cancelTask(); print('  wall-clock timeout, cancelled', flush=True)
    res = nav.getResult()
    dt = nav.get_clock().now().nanoseconds * 1e-9 - t0
    time.sleep(1.0)
    for _ in range(10): rclpy.spin_once(nav, timeout_sec=0.1)
    tf = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
    ax, ay, ayaw = tf.transform.translation.x, tf.transform.translation.y, yaw_of(tf.transform.rotation)
    tx, ty, tyaw = gz_pose()
    print(f'  result: {res.name}  ({dt:.0f} s sim time)', flush=True)
    print(f'  truth pose  ({tx:+.3f}, {ty:+.3f}, {tyaw:+.3f})  -> goal error {math.hypot(tx-gx, ty-gy):.3f} m, {angdiff(tyaw, gyaw):.3f} rad', flush=True)
    print(f'  AMCL pose   ({ax:+.3f}, {ay:+.3f}, {ayaw:+.3f})  -> localization error {math.hypot(ax-tx, ay-ty):.3f} m, {angdiff(ayaw, tyaw):.3f} rad', flush=True)
print('\nALL DONE', flush=True)
