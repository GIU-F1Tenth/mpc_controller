"""
Launch file for MPC Controller node.

This launch file starts the MPC controller node with configurable parameters
for the F1Tenth autonomous vehicle.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for MPC controller."""

    # Declare launch arguments
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.33',
        description='Vehicle wheelbase in meters'
    )

    mpc_horizon_arg = DeclareLaunchArgument(
        'mpc_horizon',
        default_value='10',
        description='MPC prediction horizon'
    )

    mpc_time_horizon_arg = DeclareLaunchArgument(
        'mpc_time_horizon',
        default_value='1.0',
        description='MPC time horizon in seconds'
    )

    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='5.0',
        description='Maximum vehicle velocity in m/s'
    )

    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='10.0',
        description='Control loop frequency in Hz'
    )

    # MPC Controller node
    mpc_node = Node(
        package='mpc_controller',
        executable='mpc_node',
        name='mpc_controller',
        output='screen',
        parameters=[{
            'wheelbase': LaunchConfiguration('wheelbase'),
            'mpc_horizon': LaunchConfiguration('mpc_horizon'),
            'mpc_time_horizon': LaunchConfiguration('mpc_time_horizon'),
            'max_velocity': LaunchConfiguration('max_velocity'),
            'control_frequency': LaunchConfiguration('control_frequency'),
        }],
        # remappings=[
        #     ('/odom', '/odometry/filtered'),  # Remap if needed
        #     ('/ackermann_cmd', '/ackermann_cmd_mux/input/navigation'),
        # ]
    )

    return LaunchDescription([
        wheelbase_arg,
        mpc_horizon_arg,
        mpc_time_horizon_arg,
        max_velocity_arg,
        control_frequency_arg,
        mpc_node,
    ])
