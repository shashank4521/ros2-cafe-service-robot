"""Build amr_navigation/config/nav2_params.yaml from the Jazzy nav2_bringup defaults.
Basic settings for the current 1.5 x 1.0 m AMR only: full tuning waits for the service-robot geometry."""
import sys, yaml
src, dst = sys.argv[1], sys.argv[2]
p = yaml.safe_load(open(src))
FOOT = '[[0.75, 0.50], [0.75, -0.50], [-0.75, -0.50], [-0.75, 0.50]]'
ODOM = '/odometry/filtered'
SCAN = '/merged'
V, W = 0.10, 0.10      # diff_drive_controller limits (m/s, rad/s)
A = 0.10               # diff_drive_controller accel limits

r = lambda k: p[k]['ros__parameters']

a = r('amcl')
a.update(base_frame_id='base_link', scan_topic=SCAN, laser_max_range=12.0, laser_min_range=0.1,
         max_beams=120, set_initial_pose=True,
         initial_pose={'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
         update_min_d=0.1, update_min_a=0.1)

r('bt_navigator')['odom_topic'] = ODOM

c = r('controller_server')
c['odom_topic'] = ODOM
c['progress_checker']['movement_time_allowance'] = 40.0   # slow robot: a 180 deg turn takes ~32 s
c['progress_checker']['required_movement_radius'] = 0.3
c['FollowPath'] = {
    'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    'desired_linear_vel': V,
    'lookahead_dist': 0.8, 'min_lookahead_dist': 0.6, 'max_lookahead_dist': 1.2,
    'lookahead_time': 1.5, 'use_velocity_scaled_lookahead_dist': False,
    'rotate_to_heading_angular_vel': W, 'max_angular_accel': A,
    'use_rotate_to_heading': True, 'rotate_to_heading_min_angle': 0.5,
    'transform_tolerance': 0.2,
    'min_approach_linear_velocity': 0.03, 'approach_velocity_scaling_dist': 0.6,
    'use_collision_detection': True, 'max_allowed_time_to_collision_up_to_carrot': 2.0,
    'use_regulated_linear_velocity_scaling': True, 'use_cost_regulated_linear_velocity_scaling': False,
    'regulated_linear_scaling_min_radius': 0.9, 'regulated_linear_scaling_min_speed': 0.03,
    'allow_reversing': False, 'max_robot_pose_search_dist': 10.0,
}

for name in ('local_costmap', 'global_costmap'):
    cm = p[name][name]['ros__parameters']
    cm.pop('robot_radius', None)
    cm['footprint'] = FOOT
    cm['footprint_padding'] = 0.05
    cm['inflation_layer'].update(inflation_radius=1.0, cost_scaling_factor=2.5)
    layer = 'voxel_layer' if name == 'local_costmap' else 'obstacle_layer'
    cm[layer]['scan'].update(topic=SCAN, raytrace_max_range=6.0, obstacle_max_range=5.5,
                             min_obstacle_height=-0.1, max_obstacle_height=2.0)
    cm[layer]['min_obstacle_height'] = -0.1
lc = p['local_costmap']['local_costmap']['ros__parameters']
lc.update(width=6, height=6)
lc['voxel_layer'].update(origin_z=-0.1)

b = r('behavior_server')
b.update(robot_base_frame='base_link', max_rotational_vel=W, min_rotational_vel=0.05,
         rotational_acc_lim=A, local_frame='odom')

vs = r('velocity_smoother')
vs.update(max_velocity=[V, 0.0, W], min_velocity=[-V, 0.0, -W],
          max_accel=[A, 0.0, A], max_decel=[-A, 0.0, -A], odom_topic=ODOM)

cmon = r('collision_monitor')
cmon['base_frame_id'] = 'base_link'
cmon['scan'].update(topic=SCAN, min_height=-0.1)    # /merged is a flat scan in base_link (z = 0)

r('docking_server')['base_frame'] = 'base_link'
p.pop('loopback_simulator', None)

hdr = ('# Nav2 params for the current AMR (1.5 x 1.0 m, diff drive) on ROS 2 Jazzy.\n'
       '# Generated from nav2_bringup/params/nav2_params.yaml. Basic settings to prove\n'
       '# autonomy only; full tuning happens after the service-robot geometry swap.\n'
       '# Velocity output: /cmd_vel (Twist) -> twist_to_stamped -> /diff_drive_controller/cmd_vel\n')
open(dst, 'w').write(hdr + yaml.safe_dump(p, sort_keys=False, default_flow_style=False, width=100))
print('wrote', dst)
