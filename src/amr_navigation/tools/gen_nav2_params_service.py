#!/usr/bin/env python3
"""Build config/nav2_params_service.yaml (service robot) from the Jazzy nav2_bringup defaults.

    python3 ~/amr_ws/src/amr_navigation/tools/gen_nav2_params_service.py

Edit the ROBOT section below, re-run, restart the sim (the file is symlinked into install/).

Sensor plan (mast LiDAR is blind below 1.25 m):
  /scan                         360 deg LiDAR at 1.25 m  -> localization (AMCL) + tall obstacles
  /camera/depth/color/points    depth camera, front     -> tables, chairs, boxes, people
  /tof_*                        4 ToF, 0.12 m, corners  -> very close low obstacles
  /keepout_zones                drawn no-go zones        -> tables/counter/dock in the global plan
"""

import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = '/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml'
DST = os.path.join(HERE, '..', 'config', 'nav2_params_service.yaml')

# ============================== ROBOT ==============================
# base_link = middle of the rear (drive) axle; body x -0.10..0.50, y -0.26..0.26
FOOTPRINT = '[[0.52, 0.27], [0.52, -0.27], [-0.12, -0.27], [-0.12, 0.27]]'
V_MAX, V_REV, W_MAX = 0.35, 0.15, 0.8      # cruise speed (m/s), reverse, turn (rad/s)
A_LIN, D_LIN, A_ANG = 0.4, 0.6, 1.5        # accel / decel limits (controller: 0.4 / 0.6 / 1.5)
ODOM = '/odometry/filtered'
SCAN = '/scan'
CLOUD = '/camera/depth/color/points'
TOFS = ['tof_front_left', 'tof_front_right', 'tof_rear_left', 'tof_rear_right']
INFLATION = 0.70                           # >= circumscribed radius incl. padding (0.64 m), needed by Smac
PADDING = 0.05                             # safety margin around the body (blind sides!)
XY_TOL, YAW_TOL = 0.10, 0.10               # how close to the goal counts as "arrived"
# ===================================================================


def scan_source(topic, z_ok_max, obstacle_range, raytrace_range, inf_valid=False):
    return {'topic': topic, 'data_type': 'LaserScan', 'marking': True, 'clearing': True,
            'min_obstacle_height': 0.0, 'max_obstacle_height': z_ok_max,
            'obstacle_max_range': obstacle_range, 'obstacle_min_range': 0.0,
            'raytrace_max_range': raytrace_range, 'raytrace_min_range': 0.0,
            'inf_is_valid': inf_valid}


def cloud_source():
    # min height 0.08 m removes the floor; max 1.6 m = robot height + margin
    return {'topic': CLOUD, 'data_type': 'PointCloud2', 'marking': True, 'clearing': True,
            'min_obstacle_height': 0.08, 'max_obstacle_height': 1.6,
            'obstacle_max_range': 3.5, 'obstacle_min_range': 0.0,
            'raytrace_max_range': 4.0, 'raytrace_min_range': 0.0}


def zones_layer():
    return {'plugin': 'nav2_costmap_2d::StaticLayer', 'map_topic': '/keepout_zones',
            'map_subscribe_transient_local': True, 'use_maximum': True,
            'footprint_clearing_enabled': False}


def main():
    p = yaml.safe_load(open(SRC))
    r = lambda k: p[k]['ros__parameters']  # noqa: E731

    a = r('amcl')
    a.update(base_frame_id='base_link', scan_topic=SCAN, laser_max_range=12.0,
             laser_min_range=0.15, max_beams=120, set_initial_pose=True,
             initial_pose={'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
             update_min_d=0.1, update_min_a=0.1, min_particles=500, max_particles=2000)

    r('bt_navigator')['odom_topic'] = ODOM

    c = r('controller_server')
    c['odom_topic'] = ODOM
    c['progress_checker'].update(required_movement_radius=0.3, movement_time_allowance=15.0)
    c['general_goal_checker'].update(xy_goal_tolerance=XY_TOL, yaw_goal_tolerance=YAW_TOL)
    # Regulated Pure Pursuit: follows the planner's path. Smac (below) has already
    # checked the robot's full rectangle along that path, so following it keeps the
    # body clear. (MPPI was tried: timid turning and dithering at the doorway.)
    c['FollowPath'] = {
        'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        'desired_linear_vel': V_MAX,
        'lookahead_dist': 0.5, 'min_lookahead_dist': 0.3, 'max_lookahead_dist': 0.8,
        'lookahead_time': 1.5, 'use_velocity_scaled_lookahead_dist': True,
        'rotate_to_heading_angular_vel': 0.6, 'max_angular_accel': A_ANG,
        'use_rotate_to_heading': True, 'rotate_to_heading_min_angle': 0.6,
        'transform_tolerance': 0.2,
        'min_approach_linear_velocity': 0.05, 'approach_velocity_scaling_dist': 0.6,
        'use_collision_detection': True, 'max_allowed_time_to_collision_up_to_carrot': 1.5,
        'use_regulated_linear_velocity_scaling': True,
        'use_cost_regulated_linear_velocity_scaling': True,
        'cost_scaling_dist': 0.4, 'cost_scaling_gain': 1.0,
        'inflation_cost_scaling_factor': 3.0,
        'regulated_linear_scaling_min_radius': 0.9, 'regulated_linear_scaling_min_speed': 0.1,
        'allow_reversing': False, 'max_robot_pose_search_dist': 10.0,
    }

    # ---------- local costmap: everything the sensors see right now ----------
    lc = p['local_costmap']['local_costmap']['ros__parameters']
    lc.pop('robot_radius', None)
    lc.update(footprint=FOOTPRINT, footprint_padding=PADDING, width=5, height=5, resolution=0.05,
              update_frequency=8.0, publish_frequency=4.0,
              plugins=['zones_layer', 'voxel_layer', 'inflation_layer'])
    lc.pop('static_layer', None)
    lc['zones_layer'] = zones_layer()
    sources = ['scan', 'camera'] + TOFS
    lc['voxel_layer'] = {
        'plugin': 'nav2_costmap_2d::VoxelLayer', 'enabled': True, 'publish_voxel_map': False,
        'origin_z': 0.0, 'z_resolution': 0.1, 'z_voxels': 16,          # 0..1.6 m
        'max_obstacle_height': 1.6, 'mark_threshold': 0, 'unknown_threshold': 15,
        'observation_sources': ' '.join(sources),
        'scan': scan_source(SCAN, 1.6, 5.5, 6.0),
        'camera': cloud_source(),
    }
    for t in TOFS:
        lc['voxel_layer'][t] = scan_source('/' + t, 1.6, 1.8, 2.0, inf_valid=True)
    lc['inflation_layer'].update(inflation_radius=INFLATION, cost_scaling_factor=3.0)

    # ---------- global costmap: map + zones + what the sensors saw ----------
    gc = p['global_costmap']['global_costmap']['ros__parameters']
    gc.pop('robot_radius', None)
    gc.update(footprint=FOOTPRINT, footprint_padding=PADDING, resolution=0.05,
              update_frequency=2.0, publish_frequency=1.0,
              plugins=['static_layer', 'zones_layer', 'obstacle_layer', 'inflation_layer'])
    gc['zones_layer'] = zones_layer()
    # 3D voxels here too: in a 2D layer the mast LiDAR's beams (1.25 m) pass OVER low
    # obstacles and erase what the camera marked -- the planner then drove at a box.
    gc['obstacle_layer'] = {
        'plugin': 'nav2_costmap_2d::VoxelLayer', 'enabled': True, 'publish_voxel_map': False,
        'origin_z': 0.0, 'z_resolution': 0.1, 'z_voxels': 16,
        'max_obstacle_height': 1.6, 'mark_threshold': 0, 'unknown_threshold': 15,
        'observation_sources': 'scan camera',
        'scan': scan_source(SCAN, 1.6, 5.5, 6.0),
        'camera': cloud_source(),
    }
    gc['inflation_layer'].update(inflation_radius=INFLATION, cost_scaling_factor=3.0)

    # Planner: NavFn (plans for the rear-axle point, cost-aware, keeps to lane centres).
    # Smac Hybrid-A* (full-rectangle checks) was tried: it can't turn on the spot, so
    # it made looping approaches to the table stops and brushed chairs. Body safety
    # comes from: footprint padding + the controller's footprint collision check +
    # 3D voxel costmaps + the collision monitor.
    pl = r('planner_server')
    pl['GridBased'].update(tolerance=0.3, use_astar=False, allow_unknown=False)

    b = r('behavior_server')
    b.update(robot_base_frame='base_link', local_frame='odom',
             max_rotational_vel=W_MAX, min_rotational_vel=0.2, rotational_acc_lim=A_ANG)

    vs = r('velocity_smoother')
    vs.update(max_velocity=[V_MAX, 0.0, W_MAX], min_velocity=[-V_REV, 0.0, -W_MAX],
              max_accel=[A_LIN, 0.0, A_ANG], max_decel=[-D_LIN, 0.0, -A_ANG], odom_topic=ODOM)

    # Collision monitor: last safety check on every velocity command
    cm = r('collision_monitor')
    cm['base_frame_id'] = 'base_link'
    # SlowdownZone: anything within ~0.4 m of the front / 0.15 m of the sides -> 40% speed.
    # FootprintApproach: predicts the body along the current motion and stops just
    # before contact -- but still allows moving AWAY (a hard 'stop' zone froze the
    # robot against an obstacle in testing).
    cm['polygons'] = ['SlowdownZone', 'FootprintApproach']
    cm['SlowdownZone'] = {'type': 'polygon', 'action_type': 'slowdown', 'slowdown_ratio': 0.4,
                          'points': '[[0.9, 0.42], [0.9, -0.42], [-0.25, -0.42], [-0.25, 0.42]]',
                          'min_points': 4, 'visualize': True, 'polygon_pub_topic': 'slowdown_zone',
                          'enabled': True}
    cm['FootprintApproach'].update(time_before_collision=1.5, min_points=4)
    cm['observation_sources'] = ['scan', 'camera'] + TOFS
    for t in TOFS:
        cm[t] = {'type': 'range', 'topic': f'/{t}/range', 'obstacles_angle': 0.436,
                 'enabled': True}
    cm['scan'] = {'type': 'scan', 'topic': SCAN, 'min_height': 0.0, 'max_height': 1.6,
                  'enabled': True}
    cm['camera'] = {'type': 'pointcloud', 'topic': CLOUD, 'min_height': 0.08,
                    'max_height': 1.6, 'enabled': True}

    r('docking_server')['base_frame'] = 'base_link'
    p.pop('loopback_simulator', None)

    hdr = ('# Nav2 params for the SERVICE ROBOT (0.6 x 0.52 m cart, rear-axle base_link).\n'
           '# GENERATED by tools/gen_nav2_params_service.py -- edit that file, not this one.\n'
           '# Velocity output: /cmd_vel (Twist) -> twist_to_stamped -> /diff_drive_controller/cmd_vel\n')
    with open(DST, 'w') as f:
        f.write(hdr + yaml.safe_dump(p, sort_keys=False, default_flow_style=False, width=100))
    print('wrote', os.path.normpath(DST))


if __name__ == '__main__':
    main()
