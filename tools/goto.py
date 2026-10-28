"""Drive to (x, y) then face yaw, using EKF pose (odom frame = world, robot started at origin)."""
import math, sys, rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def wrap(a): return math.atan2(math.sin(a), math.cos(a))
class G(Node):
    def __init__(s):
        super().__init__('goto', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        s.pub = s.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10); s.p = None
        s.create_subscription(Odometry, '/odometry/filtered', lambda m: setattr(s, 'p', (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))), 10)
    def send(s, v, w):
        m = TwistStamped(); m.header.stamp = s.get_clock().now().to_msg(); m.header.frame_id = 'base_link'
        m.twist.linear.x = float(v); m.twist.angular.z = float(w); s.pub.publish(m)
    def spin(s): rclpy.spin_once(s, timeout_sec=0.05)
    def face(s, yaw):
        while abs(wrap(yaw - s.p[2])) > 0.02:
            e = wrap(yaw - s.p[2]); s.send(0, max(-0.5, min(0.5, 1.5*e))); s.spin()
        for _ in range(30): s.send(0, 0); s.spin()
x, y, yaw = map(float, sys.argv[1:4])
rclpy.init(); g = G()
while g.p is None: g.spin()
g.face(math.atan2(y - g.p[1], x - g.p[0]))
while math.hypot(x - g.p[0], y - g.p[1]) > 0.03:
    d = math.hypot(x - g.p[0], y - g.p[1]); e = wrap(math.atan2(y - g.p[1], x - g.p[0]) - g.p[2])
    g.send(min(0.3, 0.8*d), 1.5*e); g.spin()
for _ in range(30): g.send(0, 0); g.spin()
g.face(yaw)
print('arrived (EKF):', [round(v, 3) for v in g.p])
