"""Check service-robot sensors against the known room (robot at origin, yaw 0)."""
import math, struct, time, rclpy, numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Image, PointCloud2, CameraInfo, Imu
from sensor_msgs_py import point_cloud2
class C(Node):
    def __init__(s):
        super().__init__('sensor_check', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        s.m = {}
        for t, ty in [('/scan', LaserScan), ('/camera/depth/image_rect_raw', Image), ('/camera/color/image_raw', Image),
                      ('/camera/depth/color/points', PointCloud2), ('/camera/color/camera_info', CameraInfo), ('/imu', Imu)]:
            s.create_subscription(ty, t, lambda msg, t=t: s.m.__setitem__(t, msg), qos_profile_sensor_data)
rclpy.init(); n = C(); t0 = time.time()
while len(n.m) < 6 and time.time() - t0 < 20: rclpy.spin_once(n, timeout_sec=0.2)
for _ in range(10): rclpy.spin_once(n, timeout_sec=0.1)
sc = n.m['/scan']
def at(a):
    i = round((a - sc.angle_min) / sc.angle_increment); v = [sc.ranges[j] for j in range(i-1, i+2) if 0 <= j < len(sc.ranges) and math.isfinite(sc.ranges[j])]
    return round(float(np.median(v)), 3) if v else None
print(f'LiDAR frame={sc.header.frame_id} beams={len(sc.ranges)} valid={sum(math.isfinite(r) for r in sc.ranges)}')
print(f'  ahead {at(0.0)} (exp 5.95)  behind {at(math.pi - 0.01)} (exp 6.05)  left {at(math.pi/2)} (exp 4.00)  right {at(-math.pi/2)} (exp 4.00)')
bx = math.atan2(1.5, 3.0 - 0.05); print(f'  toward box ({math.degrees(bx):.0f} deg): {at(bx)}  (box is 1.0 m tall, LiDAR at 1.25 m: should see the WALL behind, ~{4.0/math.sin(bx):.2f} m)')
d = n.m['/camera/depth/image_rect_raw']; img = n.m['/camera/color/image_raw']; ci = n.m['/camera/color/camera_info']
dep = np.frombuffer(bytes(d.data), dtype=np.float32).reshape(d.height, d.width)
print(f'Depth image frame={d.header.frame_id} {d.width}x{d.height} enc={d.encoding}; centre depth {dep[d.height//2, d.width//2]:.3f} m (exp floor at 2.57)')
print(f'Color image frame={img.header.frame_id} {img.width}x{img.height} enc={img.encoding}; camera_info frame={ci.header.frame_id} fx={ci.k[0]:.1f}')
pc = n.m['/camera/depth/color/points']
pts = np.array([[p[0], p[1], p[2]] for p in point_cloud2.read_points(pc, field_names=('x','y','z'), skip_nans=True)])
print(f'Point cloud frame={pc.header.frame_id} {pc.width}x{pc.height} fields={[f.name for f in pc.fields]} valid pts={len(pts)}')
if len(pts):
    c = pc.width//2 + (pc.height//2)*pc.width
    allp = list(point_cloud2.read_points(pc, field_names=('x','y','z'), skip_nans=False))
    print(f'  centre point (x,y,z) = {tuple(round(float(v),3) for v in allp[c])}   -> link frame expects (~2.42, 0, ~-0.83) for floor; optical expects (0, ~0.8x, 2.57)')
im = n.m['/imu']; print(f'IMU frame={im.header.frame_id} acc_z={im.linear_acceleration.z:.2f} gyro_z={im.angular_velocity.z:+.3f}')
