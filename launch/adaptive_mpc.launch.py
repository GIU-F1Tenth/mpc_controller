from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
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
    
    enable_adaptation_arg = DeclareLaunchArgument(
        'enable_adaptation',
        default_value='true',
        description='Enable adaptive parameter tuning'
    )
    
    adaptation_rate_arg = DeclareLaunchArgument(
        'adaptation_rate',
        default_value='0.1',
        description='Rate of parameter adaptation (0.0 to 1.0)'
    )
    
    control_hz_arg = DeclareLaunchArgument(
        'control_hz',
        default_value='15.0',
        description='Control loop frequency in Hz'
    )
    
    mpc_type_arg = DeclareLaunchArgument(
        'mpc_type',
        default_value='kinematic',
        description='MPC model type: kinematic or dynamic'
    )
    
    debug_logging_arg = DeclareLaunchArgument(
        'debug_logging',
        default_value='false',
        description='Enable debug logging'
    )

    # Get config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('mpc_controller'),
        'config',
        [LaunchConfiguration('config'), '.yaml']
    ])

    # Adaptive MPC controller node
    adaptive_mpc_node = Node(
        package='mpc_controller',
        executable='adaptive_mpc_node',
        name='adaptive_mpc_node',
        namespace='',
        parameters=[
            config_file,
            {
                'enable_adaptation': LaunchConfiguration('enable_adaptation'),
                'adaptation_rate': LaunchConfiguration('adaptation_rate'),
                'control_hz': LaunchConfiguration('control_hz'),
                'mpc_type': LaunchConfiguration('mpc_type'),
                'debug_logging_enabled': LaunchConfiguration('debug_logging'),
            }
        ],
        output='screen',
        emulate_tty=True,
        remappings=[
            # F1TENTH topic remappings to actual available topics
            # No remapping needed for /car_state/odom - using directly
            # No remapping needed for /scan - using directly  
            # No remapping needed for /global_waypoints - using directly
            # No remapping needed for /horizon_mapper/reference_path - using directly
            # No remapping needed for /drive - using directly
        ]
    )

    # Log info about adaptive features
    log_adaptive_info = LogInfo(
        msg=[
            'Starting F1TENTH Adaptive MPC Controller with:\n',
            '  🔧 Configuration: ', LaunchConfiguration('config'), '\n',
            '  🤖 Adaptation: ', LaunchConfiguration('enable_adaptation'), '\n',
            '  ⚡ Rate: ', LaunchConfiguration('adaptation_rate'), '\n',
            '  🎯 Control Hz: ', LaunchConfiguration('control_hz'), '\n',
            '  🚗 Model: ', LaunchConfiguration('mpc_type'), '\n',
            '  🐛 Debug: ', LaunchConfiguration('debug_logging')
        ]
    )

    return LaunchDescription([
        config_arg,
        enable_adaptation_arg,
        adaptation_rate_arg,
        control_hz_arg,
        mpc_type_arg,
        debug_logging_arg,
        log_adaptive_info,
        adaptive_mpc_node
    ])


# Usage examples:

# 1. Launch with default adaptive parameters:
# ros2 launch mpc_controller adaptive_mpc.launch.py

# 2. Launch with aggressive parameters and fast adaptation:
# ros2 launch mpc_controller adaptive_mpc.launch.py config:=params_aggressive adaptation_rate:=0.3

# 3. Launch with conservative parameters and slow adaptation:
# ros2 launch mpc_controller adaptive_mpc.launch.py config:=params_conservative adaptation_rate:=0.05

# 4. Launch with adaptation disabled (fixed parameters):
# ros2 launch mpc_controller adaptive_mpc.launch.py enable_adaptation:=false

# 5. Launch with dynamic model and high frequency control:
# ros2 launch mpc_controller adaptive_mpc.launch.py mpc_type:=dynamic control_hz:=30.0

# 6. Launch with debug logging enabled:
# ros2 launch mpc_controller adaptive_mpc.launch.py debug_logging:=true

# 7. Custom topic remappings example:
# Add custom remappings in the Node configuration above for your specific setup
