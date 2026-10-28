"""One-command simulation for the AMR or the service robot.

  ros2 launch amr_bringup sim.launch.py robot:=service world:=cafe  # service robot in the cafe
  ros2 launch amr_bringup sim.launch.py robot:=service              # service robot, warehouse
  ros2 launch amr_bringup sim.launch.py robot:=service rviz:=true   # ... + RViz2
  ros2 launch amr_bringup sim.launch.py robot:=service slam:=true   # ... + SLAM Toolbox (build a map)
  ros2 launch amr_bringup sim.launch.py robot:=service world:=cafe nav:=true rviz:=true  # autonomous cafe robot
  ros2 launch amr_bringup sim.launch.py nav:=true                   # AMR + AMCL + Nav2 on the saved map
  add gui:=false for no Gazebo window (headless), tof:=false to skip ToF conversion.

robot:=amr     1.5 x 1.0 m AMR, front + rear LiDAR merged into /merged (default, Nav2 ready)
robot:=service 0.6 x 0.52 m service cart, 360 deg LiDAR on /scan, RGB-D camera, 4 ToF
               (Nav2 with depth camera + ToF obstacles and cafe no-go zones)

Start order: Gazebo -> robot_state_publisher -> spawn -> bridge -> controllers
(after spawn) -> [merger], EKF, encoders, ToF -> SLAM or Nav2 (after a delay).
"""

import os
import re

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def world_path_and_name(world, worlds_dir):
    """world:=cafe or world:=/full/path.sdf -> (path, <world name> inside the file)."""
    path = world if os.path.isfile(world) else os.path.join(worlds_dir, world + '.sdf')
    if not os.path.isfile(path):
        raise RuntimeError(f'world not found: {world}')
    match = re.search(r'<world\s+name="([^"]+)"', open(path).read())
    return path, (match.group(1) if match else 'default')

ROBOTS = {
    'amr': {
        'urdf': 'amr.urdf.xacro',
        'entity': 'amr',
        'scan_topic': '/merged',
        'slam_params': 'mapper_params_online_async.yaml',
        'rviz': 'amr.rviz',
        'nav_params': 'nav2_params.yaml',
        'maps': {'amr_warehouse': ('warehouse.yaml', '')},   # world: (map, no-go zones)
    },
    'service': {
        'urdf': 'service_robot.urdf.xacro',
        'entity': 'service_robot',
        'scan_topic': '/scan',
        'slam_params': 'mapper_params_service.yaml',
        'rviz': 'service.rviz',
        'nav_params': 'nav2_params_service.yaml',
        'maps': {'cafe': ('cafe.yaml', 'cafe_zones.yaml')},
    },
}


def is_true(context, name):
    return context.launch_configurations[name].lower() in ('true', '1', 'yes')


def launch_setup(context):
    robot = context.launch_configurations['robot']
    if robot not in ROBOTS:
        raise RuntimeError(f"robot:={robot} unknown, use one of {list(ROBOTS)}")
    cfg = ROBOTS[robot]
    bringup_dir = get_package_share_directory('amr_bringup')
    world, world_name = world_path_and_name(context.launch_configurations['world'],
                                            os.path.join(bringup_dir, 'worlds'))
    spawn_pose = [context.launch_configurations[k] for k in ('x', 'y', 'yaw')]
    navigation_dir = get_package_share_directory('amr_navigation')
    world_key = os.path.splitext(os.path.basename(world))[0]
    default_map, default_zones = cfg['maps'].get(world_key, ('', ''))
    map_yaml = context.launch_configurations['map'] or (
        os.path.join(navigation_dir, 'maps', default_map) if default_map else '')
    zones_arg = context.launch_configurations['zones']
    zones_yaml = (os.path.join(navigation_dir, 'maps', default_zones) if default_zones else '') \
        if zones_arg == 'auto' else zones_arg

    bringup = get_package_share_directory('amr_bringup')
    description = get_package_share_directory('amr_description')
    navigation = get_package_share_directory('amr_navigation')

    actions = []

    # -r = start running, -s = server only (no window)
    gz_cmd = ['gz', 'sim', '-r', world] if is_true(context, 'gui') else ['gz', 'sim', '-r', '-s', world]
    actions.append(ExecuteProcess(cmd=gz_cmd, output='screen'))

    xacro_args = ''
    fault = float(context.launch_configurations['wheel_fault'])
    if fault != 1.0:
        # Teaching demo: wheel odometry mis-calibrated (like slipping wheels). The robot
        # really turns `fault` times more than the wheel odometry reports.
        if robot != 'service':
            raise RuntimeError('wheel_fault is only available for robot:=service')
        ctrl = yaml.safe_load(open(os.path.join(description, 'config', 'service_controllers.yaml')))
        ctrl['diff_drive_controller']['ros__parameters']['wheel_separation_multiplier'] = fault
        fault_file = '/tmp/amr_service_controllers_fault.yaml'
        yaml.safe_dump(ctrl, open(fault_file, 'w'))
        xacro_args = f' controllers_file:={fault_file}'
        actions.append(LogInfo(msg=f'WHEEL FAULT ON: wheel odometry under-reports turning by '
                                   f'{100 * (1 - 1 / fault):.0f}% (wheel_fault:={fault})'))
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(description, 'urdf', cfg['urdf']), xacro_args]),
        value_type=str)
    actions.append(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'))

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', world_name, '-topic', 'robot_description',
                   '-name', cfg['entity'], '-z', '0.02',
                   '-x', spawn_pose[0], '-y', spawn_pose[1], '-Y', spawn_pose[2]],
        output='screen')
    actions.append(TimerAction(period=3.0, actions=[spawn]))

    # Gazebo -> ROS bridges
    if robot == 'service':
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            parameters=[{'config_file': os.path.join(bringup, 'config', 'service_bridge.yaml')}],
            output='screen'))
    else:
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                       '/front_lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                       '/rear_lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                       '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
            output='screen'))
        if is_true(context, 'tof'):
            tof_names = ['tof_front_left', 'tof_front_right', 'tof_rear_left', 'tof_rear_right']
            actions.append(Node(
                package='ros_gz_bridge', executable='parameter_bridge', name='tof_bridge',
                arguments=[f'/{n}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' for n in tof_names],
                output='screen'))

    # Everything below starts once the robot exists in Gazebo.
    after = [
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'diff_drive_controller',
                        '--controller-manager-timeout', '60'],
             output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[os.path.join(bringup, 'config', 'ekf.yaml')], output='screen'),
        Node(package='amr_encoders', executable='wheel_encoder_node',
             parameters=[{'use_sim_time': True}], output='screen'),
    ]
    if robot == 'amr':
        after.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'merger.launch.py'))))
    if is_true(context, 'tof'):
        after.append(Node(package='amr_bringup', executable='tof_scan_to_range',
                          parameters=[{'use_sim_time': True}], output='screen'))

    later = []
    if is_true(context, 'slam'):
        later.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': os.path.join(navigation, 'config', cfg['slam_params']),
                'use_sim_time': 'true'}.items()))
    elif is_true(context, 'nav'):
        if not map_yaml:
            raise RuntimeError(f'nav:=true needs a map for robot:={robot} in world {world_key}: '
                               'pass map:=/path/to/map.yaml')
        later.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation, 'launch',
                                                       'navigation.launch.py')),
            launch_arguments={'map': map_yaml, 'zones': zones_yaml, 'use_sim_time': 'true',
                              'params_file': os.path.join(navigation, 'config',
                                                          cfg['nav_params'])}.items()))
        # Extra RViz layers: no-go zones (orange) and live obstacles (red)
        if zones_yaml:
            actions.append(Node(package='amr_navigation', executable='map_to_markers',
                                name='zones_to_markers', output='screen',
                                parameters=[{'grid_topic': '/keepout_zones', 'style': 'zones',
                                             'marker_topic': '/zone_markers', 'use_sim_time': True}]))
        actions.append(Node(package='amr_navigation', executable='map_to_markers',
                            name='local_costmap_to_markers', output='screen',
                            parameters=[{'grid_topic': '/local_costmap/costmap', 'style': 'costmap',
                                         'marker_topic': '/obstacle_markers', 'use_sim_time': True}]))
    if later:
        # SLAM / Nav2 wait a few seconds for TF and the scan to settle.
        after.append(TimerAction(period=8.0, actions=later))

    actions.append(RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after)))

    # RViz's Map display renders white on this laptop (GLSL error); this draws /map as markers.
    actions.append(Node(package='amr_navigation', executable='map_to_markers',
                        parameters=[{'use_sim_time': True}], output='screen'))

    if is_true(context, 'dashboard'):
        actions.append(Node(package='amr_bringup', executable='dashboard', output='screen',
                            parameters=[{'scan_topic': cfg['scan_topic']}]))

    if is_true(context, 'rviz'):
        rviz_cfg = os.path.join(bringup, 'rviz', cfg['rviz'])
        if not (is_true(context, 'slam') or is_true(context, 'nav')):
            # no map frame without SLAM/Nav2: view in odom instead
            text = open(rviz_cfg).read().replace('Fixed Frame: map', 'Fixed Frame: odom')
            rviz_cfg = '/tmp/amr_sim_rviz_odom.rviz'
            open(rviz_cfg, 'w').write(text)
        actions.append(Node(
            package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
            parameters=[{'use_sim_time': True}], output='screen'))

    return actions


def generate_launch_description():
    bringup = get_package_share_directory('amr_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='amr', description='amr or service'),
        DeclareLaunchArgument('gui', default_value='true', description='Show the Gazebo window'),
        DeclareLaunchArgument('world', default_value='amr_warehouse',
                              description='world name in amr_bringup/worlds (cafe, amr_warehouse) or a .sdf path'),
        DeclareLaunchArgument('x', default_value='0.0', description='spawn x'),
        DeclareLaunchArgument('y', default_value='0.0', description='spawn y'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='spawn yaw (rad)'),
        DeclareLaunchArgument('slam', default_value='false', description='Run SLAM Toolbox'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='Run AMCL + Nav2 on the saved map'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('wheel_fault', default_value='1.0',
                              description='service robot: e.g. 1.15 = wheel odometry under-reports turns (EKF demo)'),
        DeclareLaunchArgument('dashboard', default_value='false',
                              description='Live dashboard: sensors, EKF, AMCL, Nav2 decisions'),
        DeclareLaunchArgument('tof', default_value='true',
                              description='Convert the 4 ToF scans to Range messages'),
        DeclareLaunchArgument('map', default_value='',
                              description='map yaml for nav (empty = the saved map for this world)'),
        DeclareLaunchArgument('zones', default_value='auto',
                              description="no-go zones yaml ('auto' = this world's, '' = none)"),
        # gz_ros2_control and other system plugins live here.
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/opt/ros/jazzy/lib'),
        OpaqueFunction(function=launch_setup),
    ])
