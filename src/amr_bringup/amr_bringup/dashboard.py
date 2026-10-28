"""Live dashboard: what the robot measures (sensors) and what it decides (EKF, AMCL, Nav2).

    ros2 run amr_bringup dashboard                    # service robot (LiDAR on /scan)
    ros2 run amr_bringup dashboard --ros-args -p scan_topic:=/merged   # old AMR
    ros2 run amr_bringup dashboard --ros-args -p save_png:=/tmp/dash.png   # + save a snapshot every 5 s

Panels
  Turn rate      wheel odometry vs IMU gyro vs EKF output   (EKF follows the IMU when wheels slip)
  Speed          Nav2 command vs after the safety check vs EKF (collision monitor slowdowns show here)
  Heading        wheel-only vs EKF vs AMCL (map)             (drift and its correction)
  Wheels         left / right wheel speed from the encoders  (differential drive: turning = difference)
  LiDAR          360 deg scan, top view, robot outline
  Depth / Colour RealSense-like camera images
  ToF            4 short-range sensors, red when < 0.3 m
  Path           EKF-only track vs AMCL-corrected track
  Localization   how much AMCL corrects the EKF (cm) and how sure AMCL is (sigma)
  Decisions      Nav2 command, safety action, delivery status, nearest obstacle per sensor
"""

import collections
import math
import threading
import time

import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import tf2_ros  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist  # noqa: E402
from matplotlib.animation import FuncAnimation  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,  # noqa: E402
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image, Imu, JointState, LaserScan, Range  # noqa: E402
from std_msgs.msg import String  # noqa: E402

try:
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:  # dashboard still works without Nav2
    CollisionMonitorState = None

WINDOW_S = 30.0          # seconds shown in the time plots
TOFS = ['tof_front_left', 'tof_front_right', 'tof_rear_left', 'tof_rear_right']
SAFETY = {0: 'none', 1: 'STOP', 2: 'SLOWDOWN', 3: 'APPROACH (stop before contact)', 4: 'LIMIT'}
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Series:
    """Time series with yaw unwrapping option."""

    def __init__(self, unwrap=False):
        self.t, self.v = collections.deque(maxlen=3000), collections.deque(maxlen=3000)
        self.unwrap, self.last = unwrap, None

    def add(self, t, v):
        if self.unwrap and self.last is not None:
            v = self.last + math.atan2(math.sin(v - self.last), math.cos(v - self.last))
        self.last = v
        self.t.append(t)
        self.v.append(v)

    def arrays(self, now):
        t = np.array(self.t)
        v = np.array(self.v)
        keep = t > now - WINDOW_S
        return t[keep] - now, v[keep]


class Collector(Node):

    def __init__(self):
        super().__init__('dashboard')
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        # save_png:=/path/file.png -> also write the dashboard to that image every 5 s
        self.save_png = self.declare_parameter('save_png', '').value
        self.last_save = 0.0
        self.t0 = time.time()
        s = {k: Series() for k in ('w_wheel', 'w_imu', 'w_ekf', 'v_cmd', 'v_out', 'v_ekf',
                                   'wl', 'wr', 'corr_cm', 'amcl_sigma')}
        s.update({k: Series(unwrap=True) for k in ('yaw_wheel', 'yaw_ekf', 'yaw_amcl')})
        self.s = s
        self.path_ekf = collections.deque(maxlen=4000)
        self.path_amcl = collections.deque(maxlen=4000)
        self.scan = self.depth = self.color = None
        self.tof = {t: None for t in TOFS}
        self.cmd = Twist()
        self.out = Twist()
        self.safety = 'none'
        self.delivery = '(delivery not running)'
        self.amcl = None
        self.ekf_sigma_yaw = None
        self.wheel_r = None

        sd = qos_profile_sensor_data
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.on_wheel, 20)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_ekf, 20)
        self.create_subscription(Imu, '/imu', self.on_imu, sd)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 20)
        self.create_subscription(LaserScan, self.scan_topic, lambda m: setattr(self, 'scan', m), sd)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', lambda m: setattr(self, 'depth', m), sd)
        self.create_subscription(Image, '/camera/color/image_raw', lambda m: setattr(self, 'color', m), sd)
        for t in TOFS:
            self.create_subscription(Range, f'/{t}/range', lambda m, t=t: self.tof.__setitem__(t, m), 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_out, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, LATCHED)
        self.create_subscription(String, '/delivery/status', lambda m: setattr(self, 'delivery', m.data), LATCHED)
        if CollisionMonitorState is not None:
            self.create_subscription(CollisionMonitorState, '/collision_monitor_state', self.on_safety, 10)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.create_timer(0.1, self.on_tf)

    def now(self):
        return time.time() - self.t0

    def on_wheel(self, m):
        t = self.now()
        self.s['w_wheel'].add(t, m.twist.twist.angular.z)
        self.s['yaw_wheel'].add(t, yaw_of(m.pose.pose.orientation))

    def on_ekf(self, m):
        t = self.now()
        self.s['w_ekf'].add(t, m.twist.twist.angular.z)
        self.s['v_ekf'].add(t, m.twist.twist.linear.x)
        self.s['yaw_ekf'].add(t, yaw_of(m.pose.pose.orientation))
        self.path_ekf.append((m.pose.pose.position.x, m.pose.pose.position.y))
        self.ekf_sigma_yaw = math.sqrt(max(m.pose.covariance[35], 0.0))

    def on_imu(self, m):
        self.s['w_imu'].add(self.now(), m.angular_velocity.z)

    def on_joints(self, m):
        try:
            il, ir = m.name.index('left_wheel_joint'), m.name.index('right_wheel_joint')
        except ValueError:
            return
        if len(m.velocity) > max(il, ir):
            t = self.now()
            self.s['wl'].add(t, m.velocity[il])
            self.s['wr'].add(t, m.velocity[ir])

    def on_cmd(self, m):
        self.cmd = m
        self.s['v_cmd'].add(self.now(), m.linear.x)

    def on_out(self, m):
        self.out = m
        self.s['v_out'].add(self.now(), m.linear.x)

    def on_amcl(self, m):
        c = m.pose.covariance
        self.amcl = m
        self.s['amcl_sigma'].add(self.now(), 100 * math.sqrt(max(c[0] + c[7], 0.0)))

    def on_safety(self, m):
        self.safety = SAFETY.get(m.action_type, str(m.action_type)) + (
            f' ({m.polygon_name})' if m.polygon_name else '')

    def on_tf(self):
        t = self.now()
        try:
            mb = self.tf_buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.s['yaw_amcl'].add(t, yaw_of(mb.transform.rotation))
            self.path_amcl.append((mb.transform.translation.x, mb.transform.translation.y))
            mo = self.tf_buf.lookup_transform('map', 'odom', rclpy.time.Time())
            tr = mo.transform.translation
            self.s['corr_cm'].add(t, 100 * math.hypot(tr.x, tr.y))
        except tf2_ros.TransformException:
            pass


def nearest(values):
    v = [x for x in values if x is not None and math.isfinite(x)]
    return min(v) if v else None


def fmt(x, unit='m'):
    return '—' if x is None else f'{x:.2f} {unit}'


def main():
    rclpy.init()
    node = Collector()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    plt.style.use('dark_background')
    fig = plt.figure('Robot dashboard', figsize=(17, 10))
    gs = fig.add_gridspec(3, 4, hspace=0.45, wspace=0.28, left=0.04, right=0.99, top=0.95, bottom=0.05)
    ax_w, ax_v, ax_yaw, ax_txt = (fig.add_subplot(gs[0, i]) for i in range(4))
    ax_scan = fig.add_subplot(gs[1, 0])
    ax_depth, ax_color = fig.add_subplot(gs[1, 1]), fig.add_subplot(gs[1, 2])
    ax_tof = fig.add_subplot(gs[1, 3])
    ax_path = fig.add_subplot(gs[2, 0:2])
    ax_loc, ax_wheels = fig.add_subplot(gs[2, 2]), fig.add_subplot(gs[2, 3])

    def time_axes(ax, title, ylabel):
        ax.set_title(title, fontsize=10)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.set_xlim(-WINDOW_S, 0)
        ax.tick_params(labelsize=7)
        ax.grid(alpha=0.25)

    time_axes(ax_w, 'Turn rate: wheels vs IMU vs EKF', 'rad/s')
    l_ww, = ax_w.plot([], [], label='wheel odom', lw=1.2, color='tab:orange')
    l_wi, = ax_w.plot([], [], label='IMU gyro', lw=0.8, color='tab:cyan', alpha=0.7)
    l_we, = ax_w.plot([], [], label='EKF', lw=1.6, color='white')
    ax_w.legend(fontsize=7, loc='upper left')

    time_axes(ax_v, 'Speed: Nav2 command vs after safety vs actual', 'm/s')
    l_vc, = ax_v.plot([], [], label='Nav2 controller', lw=1.2, color='tab:purple')
    l_vo, = ax_v.plot([], [], label='after collision monitor', lw=1.2, color='tab:red')
    l_ve, = ax_v.plot([], [], label='actual (EKF)', lw=1.6, color='white')
    ax_v.legend(fontsize=7, loc='upper left')

    time_axes(ax_yaw, 'Heading: wheel-only vs EKF vs AMCL (map)', 'deg')
    l_yw, = ax_yaw.plot([], [], label='wheel odom', lw=1.2, color='tab:orange')
    l_ye, = ax_yaw.plot([], [], label='EKF', lw=1.6, color='white')
    l_ya, = ax_yaw.plot([], [], label='AMCL (map)', lw=1.2, color='lime', ls='--')
    ax_yaw.legend(fontsize=7, loc='upper left')

    ax_txt.axis('off')
    ax_txt.set_title('Decisions & state', fontsize=10)
    txt = ax_txt.text(0.0, 1.0, '', va='top', ha='left', family='monospace', fontsize=7.8,
                      transform=ax_txt.transAxes)

    ax_scan.set_title('LiDAR (top view, robot frame)', fontsize=10)
    ax_scan.set_aspect('equal')
    ax_scan.set_xlim(-6, 6)
    ax_scan.set_ylim(-6, 6)
    ax_scan.tick_params(labelsize=7)
    ax_scan.grid(alpha=0.2)
    pts_scan = ax_scan.scatter([], [], s=2, c='red')
    ax_scan.add_patch(Rectangle((-0.10, -0.26), 0.60, 0.52, fill=True, color='tab:blue', alpha=0.8))
    ax_scan.annotate('', xy=(0.9, 0), xytext=(0.5, 0), arrowprops=dict(color='white', width=1.5))

    ax_depth.set_title('Depth camera (m)', fontsize=10)
    ax_depth.axis('off')
    im_depth = ax_depth.imshow(np.zeros((240, 424)), cmap='turbo', vmin=0.2, vmax=5.0)
    fig.colorbar(im_depth, ax=ax_depth, fraction=0.03, pad=0.01).ax.tick_params(labelsize=7)
    ax_color.set_title('Colour camera', fontsize=10)
    ax_color.axis('off')
    im_color = ax_color.imshow(np.zeros((240, 424, 3), dtype=np.uint8))

    ax_tof.set_title('ToF sensors (red < 0.3 m)', fontsize=10)
    tof_names = ['front L', 'front R', 'rear L', 'rear R']
    bars = ax_tof.barh(tof_names, [0, 0, 0, 0], color='tab:green')
    ax_tof.set_xlim(0, 2.0)
    ax_tof.set_xlabel('m (2.0 = nothing seen)', fontsize=8)
    ax_tof.tick_params(labelsize=8)
    ax_tof.invert_yaxis()
    tof_txt = [ax_tof.text(0.02, i, '', va='center', fontsize=8, color='white') for i in range(4)]

    ax_path.set_title('Path: EKF only (drifts) vs AMCL-corrected (on the map)', fontsize=10)
    ax_path.set_aspect('equal')
    ax_path.tick_params(labelsize=7)
    ax_path.grid(alpha=0.2)
    l_pe, = ax_path.plot([], [], color='tab:orange', lw=1.2, label='EKF (odom frame)')
    l_pa, = ax_path.plot([], [], color='lime', lw=1.2, ls='--', label='AMCL (map frame)')
    ax_path.legend(fontsize=7, loc='upper left')

    time_axes(ax_loc, 'Localization: AMCL correction & certainty', 'cm')
    l_corr, = ax_loc.plot([], [], color='lime', lw=1.4, label='map→odom shift (EKF drift fixed)')
    l_sig, = ax_loc.plot([], [], color='magenta', lw=1.0, label='AMCL σ (uncertainty)')
    ax_loc.legend(fontsize=7, loc='upper left')

    time_axes(ax_wheels, 'Wheel encoders: wheel speeds', 'rad/s')
    l_wl, = ax_wheels.plot([], [], color='tab:blue', lw=1.2, label='left')
    l_wr, = ax_wheels.plot([], [], color='tab:pink', lw=1.2, label='right')
    ax_wheels.legend(fontsize=7, loc='upper left')

    def autoscale(ax, lines, lo=None):
        ys = np.concatenate([ln.get_ydata() for ln in lines if len(ln.get_ydata())] or [np.zeros(1)])
        a, b = float(ys.min()), float(ys.max())
        pad = max(0.1 * (b - a), 0.05)
        ax.set_ylim(a - pad if lo is None else lo, b + pad)

    def update(_):
        if not rclpy.ok():          # Ctrl+C / kill: rclpy shut down -> close the window
            plt.close(fig)
            return []
        now = node.now()
        S = node.s
        for line, key in [(l_ww, 'w_wheel'), (l_wi, 'w_imu'), (l_we, 'w_ekf'), (l_vc, 'v_cmd'),
                          (l_vo, 'v_out'), (l_ve, 'v_ekf'), (l_corr, 'corr_cm'),
                          (l_sig, 'amcl_sigma'), (l_wl, 'wl'), (l_wr, 'wr')]:
            line.set_data(*S[key].arrays(now))
        for line, key in [(l_yw, 'yaw_wheel'), (l_ye, 'yaw_ekf'), (l_ya, 'yaw_amcl')]:
            t, v = S[key].arrays(now)
            line.set_data(t, np.degrees(v))
        autoscale(ax_w, [l_ww, l_we])
        autoscale(ax_v, [l_vc, l_vo, l_ve])
        autoscale(ax_yaw, [l_yw, l_ye, l_ya])
        autoscale(ax_loc, [l_corr, l_sig], lo=0)
        autoscale(ax_wheels, [l_wl, l_wr])

        sc = node.scan
        lidar_min = None
        if sc is not None:
            r = np.array(sc.ranges)
            a = sc.angle_min + np.arange(len(r)) * sc.angle_increment
            ok = np.isfinite(r) & (r > sc.range_min)
            # x = forward (right in the plot), y = left (up in the plot)
            pts_scan.set_offsets(np.c_[r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])
            lidar_min = float(r[ok].min()) if ok.any() else None

        depth_min = None
        d = node.depth
        if d is not None and d.encoding == '32FC1':
            dep = np.frombuffer(bytes(d.data), dtype=np.float32).reshape(d.height, d.width)
            im_depth.set_data(dep)
            fin = dep[np.isfinite(dep)]
            depth_min = float(fin.min()) if fin.size else None
        c = node.color
        if c is not None and c.encoding in ('rgb8', 'bgr8'):
            img = np.frombuffer(bytes(c.data), dtype=np.uint8).reshape(c.height, c.width, 3)
            im_color.set_data(img if c.encoding == 'rgb8' else img[:, :, ::-1])

        tof_vals = []
        for i, t in enumerate(TOFS):
            m = node.tof[t]
            v = None if m is None else (m.range if math.isfinite(m.range) else m.max_range)
            tof_vals.append(v)
            bars[i].set_width(v or 0)
            bars[i].set_color('red' if v is not None and v < 0.3 else
                              ('orange' if v is not None and v < 1.0 else 'tab:green'))
            tof_txt[i].set_text('—' if v is None else f'{v:.2f} m')

        if node.path_ekf:
            pe = np.array(node.path_ekf)
            l_pe.set_data(pe[:, 0], pe[:, 1])
        if node.path_amcl:
            pa = np.array(node.path_amcl)
            l_pa.set_data(pa[:, 0], pa[:, 1])
        if node.path_ekf or node.path_amcl:
            allp = np.array(list(node.path_ekf) + list(node.path_amcl))
            lo, hi = allp.min(0) - 1.0, allp.max(0) + 1.0
            ax_path.set_xlim(lo[0], hi[0])
            ax_path.set_ylim(lo[1], hi[1])

        amcl = node.amcl
        if amcl is not None:
            p = amcl.pose.pose
            cv = amcl.pose.covariance
            amcl_txt = (f'({p.position.x:+.2f}, {p.position.y:+.2f}) '
                        f'{math.degrees(yaw_of(p.orientation)):+.0f}°  '
                        f'σ {100 * math.sqrt(max(cv[0] + cv[7], 0)):.1f} cm')
        else:
            amcl_txt = '— (no AMCL: run with nav:=true)'
        corr = S['corr_cm'].v[-1] if S['corr_cm'].v else None
        slip = None
        if S['w_wheel'].v and S['w_imu'].v:
            slip = S['w_wheel'].v[-1] - S['w_imu'].v[-1]
        ekf_sig = None if node.ekf_sigma_yaw is None else math.degrees(node.ekf_sigma_yaw)
        lines = [
            'NAV2 DECISION',
            f' controller   v {node.cmd.linear.x:+.2f} m/s  w {node.cmd.angular.z:+.2f} rad/s',
            f' after safety v {node.out.linear.x:+.2f} m/s  w {node.out.angular.z:+.2f} rad/s',
            f' safety: {node.safety}',
            'DELIVERY',
            f' {node.delivery[:52]}',
            'LOCALIZATION',
            f' AMCL {amcl_txt}',
            f' AMCL correction of EKF: {fmt(corr, "cm")}',
            f' EKF heading σ {fmt(ekf_sig, "°")} (no compass)',
            f' wheel-IMU turn gap: {fmt(slip, "rad/s")}',
            'NEAREST OBSTACLE',
            f' LiDAR 1.25 m: {fmt(lidar_min)}   depth: {fmt(depth_min)}',
            f' ToF (low): {fmt(nearest(tof_vals))}',
        ]
        txt.set_text('\n'.join(lines))
        if node.save_png and time.time() - node.last_save > 5.0:
            node.last_save = time.time()
            fig.savefig(node.save_png, dpi=80)
        return []

    _anim = FuncAnimation(fig, update, interval=200, cache_frame_data=False)  # noqa: F841
    plt.show()
    ex.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
