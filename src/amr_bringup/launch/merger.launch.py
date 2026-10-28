"""Front + rear LiDAR merger -> /merged, using config/dual_laser_merger.yaml."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('amr_bringup'),
        'config', 'dual_laser_merger.yaml')

    container = ComposableNodeContainer(
        name='merger_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        parameters=[{'use_sim_time': True}],
        composable_node_descriptions=[
            ComposableNode(
                package='dual_laser_merger',
                plugin='merger_node::MergerNode',
                name='amr_dual_laser_merger',
                parameters=[params],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
