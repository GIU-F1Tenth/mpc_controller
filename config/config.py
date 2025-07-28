# Trajectory settings
enable_trajectory_generation= True
optimal_trajectory_path= "/home/mohammedazab/ws/src/race_stack/mpc_controller/trajectory/optimal_trajectory.csv"
reference_trajectory_path= "/home/mohammedazab/ws/src/race_stack/mpc_controller/trajectory/ref_trajectory.csv"

# Vehicle parameters
wheelbase= 0.33

# MPC Horizon
horizon_N= 13
horizon_T= 0.8
lookahead_distance= 0.5

# Vehicle limits
max_steering_angle= 0.9
max_acceleration= 2.8
max_deceleration= 2.8
min_speed= 0.1
max_speed= 8.0

# Cost function weights
enable_cost_function_weights= True
steering_weight= 1.0
acceleration_weight= 0.5
jerk_weight= 0.1
heading_weight= 2.0
position_weight= 5.0
velocity_weight= 1.0

# Hard constraints
enable_hard_constraints= False
hard_max_steering_angle= 0.4
hard_max_acceleration= 0.8
hard_max_deceleration= 0.8

# Obstacle avoidance
enable_obstacle_avoidance= False
obstacle_avoidance_weight= 0.5

# Speed control
enable_speed_control= True
speed_control_weight= 0.2

# Trajectory tracking
enable_trajectory_tracking= True
trajectory_tracking_weight= 0.1

# Safety checks
enable_safety_checks= True
safety_check_distance= 0.5

# Logging
enable_logging= True

# Optimized MPC parameters
mpc_type= "kinematic" # "kinematic" or "dynamic"
solver_type= "ipopt" # "ipopt" or "sqpmethod"
control_hz= 20.0

# Safety Parameters
safety_timeout= 1.0
emergency_brake_threshold= 2.0

# Topics
odom_topic= "car_state/odom"
reference_topic= "/mpc/reference_trajectory"
status_topic= "/mpc/path_ready"
control_topic= "/drive"

# QoS
qos_depth= 10

# Debug and Logging settings
debug_logging_enabled= False
performance_logging_enabled= True
state_logging_enabled= False
control_logging_enabled= True
trajectory_logging_enabled= False
solver_logging_enabled= False
log_frequency_divider= 10  # Log every N iterations

yaml_config_enabled= False