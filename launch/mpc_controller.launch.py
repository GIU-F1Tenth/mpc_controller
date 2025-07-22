from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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

    trajectory_arg = DeclareLaunchArgument(
        'trajectory_path',
        default_value='',
        description='Optional trajectory file path override'
    )

    # Get config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('mpc_controller'),
        'config',
        [LaunchConfiguration('config'), '.yaml']
    ])

    # Trajectory publisher node - launched first
    trajectory_publisher_node = Node(
        package='mpc_controller',
        executable='trajectory_publisher_node',
        name='trajectory_publisher_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/trajectory_publisher/reference_trajectory', '/mpc/reference_trajectory'),
            ('/trajectory_publisher/path_ready', '/mpc/path_ready')
        ]
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
        trajectory_arg,
        trajectory_publisher_node,
        mpc_node
    ])


# ros2 launch mpc_controller mpc_controller.launch.py
# ros2 launch mpc_controller mpc_controller.launch.py config:=params_aggressive
# ros2 launch mpc_controller mpc_controller.launch.py config:=params_conservative
# ros2 launch mpc_controller mpc_controller.launch.py config:=params_precision