"""AMCL + Nav2 on a saved map, plus the Twist -> TwistStamped adapter.

Expects the simulation, EKF (and for the AMR the LiDAR merger) to be running already.
Normally started by sim.launch.py with nav:=true. Stand-alone:
  ros2 launch amr_navigation navigation.launch.py map:=<map.yaml> params_file:=<nav2.yaml> \
      [zones:=<zones.yaml>]

zones: optional second map with no-go areas (black = never drive), published on
/keepout_zones and used by the service robot's costmaps.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('amr_navigation')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    params = LaunchConfiguration('params_file')
    zones = LaunchConfiguration('zones')
    use_sim_time = LaunchConfiguration('use_sim_time')
    have_zones = IfCondition(PythonExpression(["'", zones, "' != ''"]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg, 'maps', 'warehouse.yaml')),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('zones', default_value='',
                              description='no-go zones map yaml (empty = none)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'params_file': params,
                'use_sim_time': use_sim_time,
                'slam': 'False',
                'autostart': 'True',
                'use_composition': 'False',
            }.items(),
        ),

        # No-go zones: a second map server on /keepout_zones
        Node(
            package='nav2_map_server', executable='map_server', name='zones_map_server',
            parameters=[{'yaml_filename': zones, 'topic_name': 'keepout_zones',
                         'frame_id': 'map', 'use_sim_time': use_sim_time}],
            condition=have_zones, output='screen'),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_zones',
            parameters=[{'autostart': True, 'node_names': ['zones_map_server'],
                         'use_sim_time': use_sim_time}],
            condition=have_zones, output='screen'),

        Node(
            package='amr_navigation', executable='twist_to_stamped', name='twist_to_stamped',
            parameters=[{'use_sim_time': use_sim_time}], output='screen'),
    ])
