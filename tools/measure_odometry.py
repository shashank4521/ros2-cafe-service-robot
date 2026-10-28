"""Run one motion and compare yaw/distance from: Gazebo truth, wheel joints, diff_drive odom, IMU integration, EKF.
usage: measure.py spin <rad/s> <seconds> | line <m/s> <seconds>"""
import math, sys, time, subprocess, re, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState

def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def gz_model_name():
    out = subprocess.run(['gz','model','--list'],capture_output=True,text=True,timeout=20).stdout
    return 'service_robot' if 'service_robot' in out else 'amr'
MODEL = gz_model_name()
# wheel radius / separation of each robot (must match the controllers yaml)
R, SEP = (0.075, 0.47) if MODEL == 'service_robot' else (0.125, 0.8)
def gz_pose():
    out = subprocess.run(['gz','model','-m',MODEL,'-p'],capture_output=True,text=True,timeout=20).stdout
    i = out.index('Pose [ XYZ'); nums = [float(v) for v in re.findall(r'-?\d+\.\d+', out[i:])][:6]
    return nums  # x y z roll pitch yaw

class M(Node):
    def __init__(s):
        super().__init__('measure', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        s.pub = s.create_publisher(TwistStamped,'/diff_drive_controller/cmd_vel',10)
        s.odom = s.ekf = s.js = None; s.imu_yaw = 0.0; s.imu_t = None; s.imu_on = False
        s.odom_yaw_unwrapped = s.ekf_yaw_unwrapped = 0.0; s.last_o = s.last_e = None
        s.create_subscription(Odometry,'/diff_drive_controller/odom',s.o_cb,50)
        s.create_subscription(Odometry,'/odometry/filtered',s.e_cb,50)
        s.create_subscription(Imu,'/imu',s.i_cb,qos_profile_sensor_data)
        s.create_subscription(JointState,'/joint_states',s.j_cb,50)
    def unwrap(s, last, y, acc):
        if last is None: return y, acc
        return y, acc + math.atan2(math.sin(y-last), math.cos(y-last))
    def o_cb(s,m):
        s.odom = m; y = yaw_of(m.pose.pose.orientation); s.last_o, s.odom_yaw_unwrapped = s.unwrap(s.last_o, y, s.odom_yaw_unwrapped)
    def e_cb(s,m):
        s.ekf = m; y = yaw_of(m.pose.pose.orientation); s.last_e, s.ekf_yaw_unwrapped = s.unwrap(s.last_e, y, s.ekf_yaw_unwrapped)
    def i_cb(s,m):
        t = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        if s.imu_on and s.imu_t is not None: s.imu_yaw += m.angular_velocity.z*(t-s.imu_t)
        s.imu_t = t
    def j_cb(s,m):
        s.js = dict(zip(m.name, m.position))
    def send(s,v,w):
        msg = TwistStamped(); msg.header.stamp = s.get_clock().now().to_msg(); msg.header.frame_id='base_link'
        msg.twist.linear.x=float(v); msg.twist.angular.z=float(w); s.pub.publish(msg)
    def run_for(s, v, w, secs):
        t0 = s.get_clock().now().nanoseconds*1e-9
        while s.get_clock().now().nanoseconds*1e-9 - t0 < secs: s.send(v,w); rclpy.spin_once(s,timeout_sec=0.05)

mode, speed, secs = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
rclpy.init(); n = M()
while n.odom is None or n.ekf is None or n.js is None or n.imu_t is None: rclpy.spin_once(n,timeout_sec=0.1)
n.run_for(0,0,1.0)
g0 = gz_pose(); o0 = n.odom_yaw_unwrapped; e0 = n.ekf_yaw_unwrapped
op0 = n.odom.pose.pose.position; ep0 = n.ekf.pose.pose.position
jl0, jr0 = n.js['left_wheel_joint'], n.js['right_wheel_joint']
n.imu_yaw = 0.0; n.imu_on = True
if mode == 'spin': n.run_for(0, speed, secs)
else: n.run_for(speed, 0, secs)
n.run_for(0,0,3.0); n.imu_on = False
g1 = gz_pose()
dl = (n.js['left_wheel_joint']-jl0)*R; dr = (n.js['right_wheel_joint']-jr0)*R
op1 = n.odom.pose.pose.position; ep1 = n.ekf.pose.pose.position
gyaw = math.atan2(math.sin(g1[5]-g0[5]), math.cos(g1[5]-g0[5]))
if mode == 'spin':
    # unwrap truth using the odom estimate as a guide
    ref = n.odom_yaw_unwrapped - o0
    gyaw += 2*math.pi*round((ref-gyaw)/(2*math.pi))
    print(f'TRUTH (gazebo) yaw      {gyaw:+.4f} rad')
    print(f'wheel joints yaw       {(dr-dl)/SEP:+.4f}  -> effective separation for truth: {((dr-dl)/gyaw if abs(gyaw)>1e-6 else float("nan")):.4f} m')
    print(f'diff_drive odom yaw     {ref:+.4f}')
    print(f'IMU integrated yaw      {n.imu_yaw:+.4f}')
    print(f'EKF yaw                 {n.ekf_yaw_unwrapped-e0:+.4f}')
    print(f'gazebo pos drift        {math.hypot(g1[0]-g0[0], g1[1]-g0[1]):.4f} m, pitch {g1[4]:+.4f}')
else:
    d_true = math.hypot(g1[0]-g0[0], g1[1]-g0[1])
    print(f'TRUTH distance          {d_true:.4f} m, yaw change {gyaw:+.4f}')
    print(f'wheel joints distance   {(dl+dr)/2:.4f}  (L {dl:.4f}, R {dr:.4f})')
    print(f'diff_drive odom dist    {math.hypot(op1.x-op0.x, op1.y-op0.y):.4f}')
    print(f'EKF dist                {math.hypot(ep1.x-ep0.x, ep1.y-ep0.y):.4f}')
    print(f'IMU integrated yaw      {n.imu_yaw:+.4f}')
