from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='params',
        description='Configuration file name (params, params_aggressive, params_conservative, params_precision)'
    )

    # Get config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('mpc_controller'),
        'config',
        [LaunchConfiguration('config'), '.yaml']
    ])

    # Optimized MPC controller node
    mpc_node = Node(
        package='mpc_controller',
        executable='mpc_node',
        name='optimized_mpc_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
        remappings=[
            # Add any topic remappings if needed
        ]
    )

    return LaunchDescription([
        config_arg,
        mpc_node
    ])


# Usage examples:

# 1. Launch with default parameters:
# ros2 launch mpc_controller mpc_controller.launch.py

# 2. Launch with aggressive parameters:
# ros2 launch mpc_controller mpc_controller.launch.py config:=params_aggressive

# 3. Launch with conservative parameters:
# ros2 launch mpc_controller mpc_controller.launch.py config:=params_conservative

# 4. Launch with precision parameters:
# ros2 launch mpc_controller mpc_controller.launch.py config:=params_precision
