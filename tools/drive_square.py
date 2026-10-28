"""Drive a slow 2x2 m square (right turns) using EKF pose feedback."""
import math, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

V, W, SIDE = 0.10, 0.10, 2.0

def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

class Driver(Node):
    def __init__(s):
        super().__init__('square_driver', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        s.pub = s.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        s.pose = None
        s.create_subscription(Odometry, '/odometry/filtered', s.cb, 10)
    def cb(s, m):
        p = m.pose.pose
        s.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
    def send(s, v, w):
        m = TwistStamped(); m.header.stamp = s.get_clock().now().to_msg(); m.header.frame_id = 'base_link'
        m.twist.linear.x = v; m.twist.angular.z = w; s.pub.publish(m)
    def spin(s, dt=0.05): rclpy.spin_once(s, timeout_sec=dt)
    def stop(s, secs=3.0):
        t0 = time.time()
        while time.time() - t0 < secs: s.send(0.0, 0.0); s.spin()

    def forward(s, dist):
        x0, y0, _ = s.pose
        while math.hypot(s.pose[0]-x0, s.pose[1]-y0) < dist - 0.01:
            s.send(V, 0.0); s.spin()
        s.stop()

    def turn(s, ang):
        y0 = s.pose[2]; target = wrap(y0 + ang)
        while abs(wrap(target - s.pose[2])) > 0.01:
            s.send(0.0, math.copysign(W, wrap(target - s.pose[2]))); s.spin()
        s.stop()

rclpy.init(); d = Driver()
while d.pose is None: d.spin()
print('start pose', [round(v, 3) for v in d.pose], flush=True)
for i in range(4):
    d.forward(SIDE); print(f'side {i+1} done', [round(v, 3) for v in d.pose], flush=True)
    d.turn(-math.pi/2); print(f'turn {i+1} done', [round(v, 3) for v in d.pose], flush=True)
print('end pose (EKF)', [round(v, 3) for v in d.pose], flush=True)
