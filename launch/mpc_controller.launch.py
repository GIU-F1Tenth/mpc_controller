from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='params',
        description='Configuration file name (params, params_aggressive, params_conservative, params_precision)'
    )

    trajectory_type_arg = DeclareLaunchArgument(
        'trajectory_type',
        default_value='straight',
        description='Trajectory type: straight, circle, or infinity (for simple publisher)'
    )

    reference_speed_arg = DeclareLaunchArgument(
        'reference_speed',
        default_value='1.0',
        description='Reference speed for trajectory (for simple publisher)'
    )

    use_simple_publisher_arg = DeclareLaunchArgument(
        'use_simple_publisher',
        default_value='false',
        description='Use simple trajectory publisher (true) or CSV-based publisher (false)'
    )

    # Get config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('mpc_controller'),
        'config',
        [LaunchConfiguration('config'), '.yaml']
    ])

    # Simple trajectory publisher - optional, used when explicitly requested
    simple_trajectory_publisher_node = Node(
        package='mpc_controller',
        executable='simple_trajectory_publisher',
        name='simple_trajectory_publisher',
        parameters=[{
            'trajectory_type': LaunchConfiguration('trajectory_type'),
            'reference_speed': LaunchConfiguration('reference_speed'),
            'horizon_N': 10,
            'publish_rate': 10.0
        }],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_simple_publisher'))
    )

    # CSV-based trajectory publisher (default) - uses trajectory files
    csv_trajectory_publisher_node = Node(
        package='mpc_controller',
        executable='trajectory_publisher_node',
        name='trajectory_publisher_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/trajectory_publisher/reference_trajectory', '/mpc/reference_trajectory'),
            ('/trajectory_publisher/path_ready', '/mpc/path_ready')
        ],
        condition=UnlessCondition(LaunchConfiguration('use_simple_publisher'))
    )

    # Optimized MPC controller node - launched after trajectory publisher
    mpc_node = TimerAction(
        period=2.0,  # Wait 2 seconds for trajectory publisher to initialize
        actions=[
            Node(
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
        ]
    )

    return LaunchDescription([
        config_arg,
        trajectory_type_arg,
        reference_speed_arg,
        use_simple_publisher_arg,
        simple_trajectory_publisher_node,
        csv_trajectory_publisher_node,
        mpc_node
    ])


# Usage examples:
# Default - uses CSV-based trajectory publisher:
# ros2 launch mpc_controller mpc_controller.launch.py

# CSV-based trajectory publisher with aggressive config:
# ros2 launch mpc_controller mpc_controller.launch.py config:=params_aggressive

# Use simple trajectory publisher with circle pattern:
# ros2 launch mpc_controller mpc_controller.launch.py
# use_simple_publisher:=true trajectory_type:=circle reference_speed:=2.0

# Simple publisher with conservative MPC settings and infinity pattern:
# ros2 launch mpc_controller mpc_controller.launch.py
# use_simple_publisher:=true config:=params_conservative
# trajectory_type:=infinity
