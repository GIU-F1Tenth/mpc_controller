# launch/mpc_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpc_controller',
            executable='MPCtrlNode',
            name='mpc_controller',
            output='screen',
            parameters=[{
                'wheelbase': 0.33,
                'mpc_horizon': 10,
                'mpc_timestep': 0.1
            }]
        )
    ])
