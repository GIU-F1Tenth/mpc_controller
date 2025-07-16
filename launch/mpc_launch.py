#!/usr/bin/env python3

"""
MPC Launch File

This launch file starts the MPC controller node with its configuration parameters.

Author: Mohammed Azab <mohammed@azab.io>
License: MIT
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for the MPC controller node."""

    # Get package directory
    pkg_dir = get_package_share_directory('mpc_controller')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'mpc_params.yaml'),
        description='Path to the MPC configuration file'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Logging level for the MPC controller node'
    )

    # Define the MPC node
    mpc_node = Node(
        package='mpc_controller',
        executable='mpc_node',
        name='mpc_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        config_file_arg,
        log_level_arg,
        mpc_node,
    ])
