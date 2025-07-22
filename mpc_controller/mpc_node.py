#!/usr/bin/env python3

"""
F1TENTH Optimized MPC Controller Node with All Parameters

This ROS2 node implements an advanced Model Predictive Controller for trajectory tracking
in F1TENTH autonomous racing cars using all parameters from params.yaml.

Author: Mohammed Azab <mohammed@azab.io>
License: MIT
Version: 2.0.0
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from giu_f1t_interfaces.msg import VehicleState, VehicleStateArray

import numpy as np
from tf_transformations import euler_from_quaternion
import time

from .optimized_mpc_controller import OptimizedMPCController, MPCType


class MPCNode(Node):
    def __init__(self):
        super().__init__('optimized_mpc_node')

        self._declare_all_parameters()
        self._load_all_parameters()
        self._initialize_optimized_mpc()
        self._initialize_state()
        self._setup_subscriptions()
        self._setup_publishers()
        self._setup_timers()

        self.get_logger().info("🏎️ F1TENTH Optimized MPC Node with All Parameters started successfully")

    def _declare_all_parameters(self):
        """Declare all ROS2 parameters from params.yaml"""

        # Trajectory settings
        self.declare_parameter('enable_trajectory_generation', True)
        self.declare_parameter('optimal_trajectory_path', '')
        self.declare_parameter('reference_trajectory_path', '')

        # Vehicle parameters
        self.declare_parameter('wheelbase', 0.33)

        # MPC Horizon
        self.declare_parameter('horizon_N', 10)
        self.declare_parameter('horizon_T', 1.0)
        self.declare_parameter('lookahead_distance', 0.7)

        # Vehicle limits
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('max_acceleration', 1.0)
        self.declare_parameter('max_deceleration', 1.0)
        self.declare_parameter('min_speed', 0.1)
        self.declare_parameter('max_speed', 2.0)

        # Cost function weights
        self.declare_parameter('enable_cost_function_weights', True)
        self.declare_parameter('cost_function_weights.steering_weight', 0.1)
        self.declare_parameter('cost_function_weights.acceleration_weight', 0.1)
        self.declare_parameter('cost_function_weights.jerk_weight', 0.1)
        self.declare_parameter('cost_function_weights.heading_weight', 0.1)
        self.declare_parameter('cost_function_weights.position_weight', 0.1)
        self.declare_parameter('cost_function_weights.velocity_weight', 0.1)

        # Hard constraints
        self.declare_parameter('enable_hard_constraints', True)
        self.declare_parameter('hard_constraints.max_steering_angle', 0.5)
        self.declare_parameter('hard_constraints.max_acceleration', 1.0)
        self.declare_parameter('hard_constraints.max_deceleration', 1.0)

        # Obstacle avoidance
        self.declare_parameter('enable_obstacle_avoidance', False)
        self.declare_parameter('obstacle_avoidance_weight', 0.5)

        # Speed control
        self.declare_parameter('enable_speed_control', True)
        self.declare_parameter('speed_control_weight', 0.3)

        # Trajectory tracking
        self.declare_parameter('enable_trajectory_tracking', True)
        self.declare_parameter('trajectory_tracking_weight', 0.2)

        # Safety checks
        self.declare_parameter('enable_safety_checks', True)
        self.declare_parameter('safety_check_distance', 0.5)

        # Logging
        self.declare_parameter('enable_logging', True)

        # Additional MPC parameters
        self.declare_parameter('mpc_type', 'kinematic')  # 'kinematic' or 'dynamic'
        self.declare_parameter('solver_type', 'ipopt')   # 'ipopt' or 'sqpmethod'
        self.declare_parameter('control_hz', 20.0)

        # Safety Parameters
        self.declare_parameter('safety_timeout', 1.0)
        self.declare_parameter('emergency_brake_threshold', 2.0)

        # Topics
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('reference_topic', '/mpc/reference_trajectory')
        self.declare_parameter('status_topic', '/mpc/path_ready')
        self.declare_parameter('control_topic', '/drive')

        # QoS
        self.declare_parameter('qos_depth', 10)

    def _load_all_parameters(self):
        """Load all parameters from ROS2 parameter server"""

        # Trajectory settings
        self.enable_trajectory_generation = self.get_parameter('enable_trajectory_generation').value
        self.optimal_trajectory_path = self.get_parameter('optimal_trajectory_path').value
        self.reference_trajectory_path = self.get_parameter('reference_trajectory_path').value

        # Vehicle parameters
        self.wheelbase = self.get_parameter('wheelbase').value

        # MPC Horizon
        self.horizon_N = self.get_parameter('horizon_N').value
        self.horizon_T = self.get_parameter('horizon_T').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value

        # Vehicle limits
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.max_deceleration = self.get_parameter('max_deceleration').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_speed = self.get_parameter('max_speed').value

        # Cost function weights
        self.enable_cost_function_weights = self.get_parameter('enable_cost_function_weights').value
        self.cost_function_weights = {
            'steering_weight': self.get_parameter('cost_function_weights.steering_weight').value,
            'acceleration_weight': self.get_parameter('cost_function_weights.acceleration_weight').value,
            'jerk_weight': self.get_parameter('cost_function_weights.jerk_weight').value,
            'heading_weight': self.get_parameter('cost_function_weights.heading_weight').value,
            'position_weight': self.get_parameter('cost_function_weights.position_weight').value,
            'velocity_weight': self.get_parameter('cost_function_weights.velocity_weight').value,
        }

        # Hard constraints
        self.enable_hard_constraints = self.get_parameter('enable_hard_constraints').value
        self.hard_constraints = {
            'max_steering_angle': self.get_parameter('hard_constraints.max_steering_angle').value,
            'max_acceleration': self.get_parameter('hard_constraints.max_acceleration').value,
            'max_deceleration': self.get_parameter('hard_constraints.max_deceleration').value,
        }

        # Obstacle avoidance
        self.enable_obstacle_avoidance = self.get_parameter('enable_obstacle_avoidance').value
        self.obstacle_avoidance_weight = self.get_parameter('obstacle_avoidance_weight').value

        # Speed control
        self.enable_speed_control = self.get_parameter('enable_speed_control').value
        self.speed_control_weight = self.get_parameter('speed_control_weight').value

        # Trajectory tracking
        self.enable_trajectory_tracking = self.get_parameter('enable_trajectory_tracking').value
        self.trajectory_tracking_weight = self.get_parameter('trajectory_tracking_weight').value

        # Safety checks
        self.enable_safety_checks = self.get_parameter('enable_safety_checks').value
        self.safety_check_distance = self.get_parameter('safety_check_distance').value

        # Logging
        self.enable_logging = self.get_parameter('enable_logging').value

        # Optimized MPC Parameters
        mpc_type_str = self.get_parameter('mpc_type').value
        self.mpc_type = MPCType.KINEMATIC if mpc_type_str == 'kinematic' else MPCType.DYNAMIC
        self.solver_type = self.get_parameter('solver_type').value
        self.control_hz = self.get_parameter('control_hz').value

        # Safety Parameters
        self.safety_timeout = self.get_parameter('safety_timeout').value
        self.emergency_brake_threshold = self.get_parameter('emergency_brake_threshold').value

        # Topics
        self.odom_topic = self.get_parameter('odom_topic').value
        self.reference_topic = self.get_parameter('reference_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.control_topic = self.get_parameter('control_topic').value

        self.qos_depth = self.get_parameter('qos_depth').value

    def _initialize_optimized_mpc(self):
        """Initialize the optimized MPC controller with all parameters"""

        try:
            self.mpc_controller = OptimizedMPCController(
                N=self.horizon_N,
                T=self.horizon_T,
                wheelbase=self.wheelbase,
                mpc_type=self.mpc_type,
                solver_type=self.solver_type,
                enable_logging=self.enable_logging,
                # All parameters from params.yaml
                max_steering_angle=self.max_steering_angle,
                max_acceleration=self.max_acceleration,
                max_deceleration=self.max_deceleration,
                min_speed=self.min_speed,
                max_speed=self.max_speed,
                enable_cost_function_weights=self.enable_cost_function_weights,
                cost_function_weights=self.cost_function_weights,
                enable_hard_constraints=self.enable_hard_constraints,
                hard_constraints=self.hard_constraints,
                enable_obstacle_avoidance=self.enable_obstacle_avoidance,
                obstacle_avoidance_weight=self.obstacle_avoidance_weight,
                enable_speed_control=self.enable_speed_control,
                speed_control_weight=self.speed_control_weight,
                enable_trajectory_tracking=self.enable_trajectory_tracking,
                trajectory_tracking_weight=self.trajectory_tracking_weight,
                enable_safety_checks=self.enable_safety_checks,
                safety_check_distance=self.safety_check_distance,
                lookahead_distance=self.lookahead_distance
            )

            if self.enable_logging:
                self.get_logger().info(
                    f"✅ Initialized {self.mpc_type.value} Optimized MPC controller with all parameters")

        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize Optimized MPC controller: {e}")
            raise e

    def _initialize_state(self):
        """Initialize node state variables"""

        # Vehicle state
        self.current_pose = None
        self.current_velocity = 0.0
        self.current_yaw = 0.0
        self.current_steering_angle = 0.0

        # Additional states for dynamic model
        self.current_beta = 0.0  # Sideslip angle
        self.current_yaw_rate = 0.0

        # Reference trajectory
        self.reference_trajectory = []
        self.path_ready = False

        # Safety and performance tracking
        self.last_trajectory_time = None
        self.last_odom_time = None
        self.control_active = False
        self.emergency_stop = False

        # Performance metrics
        self.control_loop_times = []

    def _setup_subscriptions(self):
        """Setup ROS2 subscriptions"""

        # Odometry subscription
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            self.qos_depth
        )

        # Reference trajectory subscription
        self.create_subscription(
            VehicleStateArray,
            self.reference_topic,
            self._reference_callback,
            self.qos_depth
        )

        # Path ready status subscription
        self.create_subscription(
            Bool,
            self.status_topic,
            self._status_callback,
            self.qos_depth
        )

    def _setup_publishers(self):
        """Setup ROS2 publishers"""

        # Control command publisher
        self.control_publisher = self.create_publisher(
            AckermannDriveStamped,
            self.control_topic,
            self.qos_depth
        )

        # Performance metrics publishers
        self.solve_time_publisher = self.create_publisher(
            Float32,
            '/mpc/solve_time',
            self.qos_depth
        )

        # Diagnostics publisher
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            '/mpc/diagnostics',
            self.qos_depth
        )

    def _setup_timers(self):
        """Setup ROS2 timers"""

        # Main control loop
        self.create_timer(1.0 / self.control_hz, self._control_loop)

        # Diagnostics timer (lower frequency)
        self.create_timer(1.0, self._publish_diagnostics)

    def _odom_callback(self, msg: Odometry):
        """Process odometry data"""

        self.current_pose = msg.pose.pose

        # Extract velocity
        linear_vel = msg.twist.twist.linear
        self.current_velocity = np.sqrt(linear_vel.x**2 + linear_vel.y**2)

        # Extract yaw angle
        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])

        # Extract additional states for dynamic model
        if self.mpc_type == MPCType.DYNAMIC:
            self.current_yaw_rate = msg.twist.twist.angular.z
            # Estimate sideslip angle (simplified)
            if self.current_velocity > 0.1:
                lateral_vel = linear_vel.y  # Assuming body frame
                self.current_beta = np.arctan(lateral_vel / self.current_velocity)

        self.last_odom_time = self.get_clock().now()

    def _reference_callback(self, msg: VehicleStateArray):
        """Process reference trajectory"""

        if len(msg.states) < self.horizon_N:
            self.get_logger().warn(f"Reference trajectory too short: {len(msg.states)} < {self.horizon_N}")
            return

        # Convert to numpy array for MPC
        self.reference_trajectory = []
        for i, state in enumerate(msg.states):
            if self.mpc_type == MPCType.KINEMATIC:
                # Calculate heading from trajectory points
                if i < len(msg.states) - 1:
                    next_state = msg.states[i + 1]
                    theta = np.arctan2(next_state.y - state.y, next_state.x - state.x)
                else:
                    theta = self.current_yaw  # Use current heading for last point

                self.reference_trajectory.append([
                    state.x, state.y, state.v, theta
                ])
            else:  # DYNAMIC
                # Calculate heading from trajectory points
                if i < len(msg.states) - 1:
                    next_state = msg.states[i + 1]
                    theta = np.arctan2(next_state.y - state.y, next_state.x - state.x)
                else:
                    theta = self.current_yaw

                self.reference_trajectory.append([
                    state.x, state.y, state.v, theta, 0.0, 0.0  # beta=0, r=0 for reference
                ])

        self.reference_trajectory = np.array(self.reference_trajectory)
        self.last_trajectory_time = self.get_clock().now()

    def _status_callback(self, msg: Bool):
        """Process path ready status"""
        self.path_ready = msg.data

    def _control_loop(self):
        """Main MPC control loop"""

        loop_start_time = time.time()

        # Safety checks
        if not self._safety_checks():
            self._publish_emergency_stop()
            return

        # Check if we have all required data
        if not self._data_ready():
            self._publish_zero_control()
            return

        # Solve MPC optimization
        try:
            result = self._solve_mpc_optimization()

            if result['success']:
                self._publish_control_command(
                    result['acceleration'],
                    result['steering']
                )

                # Publish solve time
                solve_time_msg = Float32()
                solve_time_msg.data = result['solve_time']
                self.solve_time_publisher.publish(solve_time_msg)

                self.control_active = True

            else:
                if self.enable_logging:
                    self.get_logger().warn("Optimized MPC optimization failed, publishing zero control")
                self._publish_zero_control()
                self.control_active = False

        except Exception as e:
            self.get_logger().error(f"Control loop error: {e}")
            self._publish_emergency_stop()
            self.control_active = False

        # Track performance
        loop_time = time.time() - loop_start_time
        self.control_loop_times.append(loop_time)

        if len(self.control_loop_times) > 100:
            self.control_loop_times.pop(0)

    def _safety_checks(self):
        """Perform safety checks using parameters"""

        if not self.enable_safety_checks:
            return True

        current_time = self.get_clock().now()

        # Check for data timeouts
        if self.last_odom_time:
            odom_age = (current_time - self.last_odom_time).nanoseconds / 1e9
            if odom_age > self.safety_timeout:
                return False

        if self.last_trajectory_time:
            traj_age = (current_time - self.last_trajectory_time).nanoseconds / 1e9
            if traj_age > self.safety_timeout:
                return False

        # Check for excessive velocity
        if self.current_velocity > self.emergency_brake_threshold * self.max_speed:
            return False

        return True

    def _data_ready(self):
        """Check if all required data is available"""

        checks = [
            self.current_pose is not None,
            self.path_ready,
            len(self.reference_trajectory) >= self.horizon_N
        ]

        return all(checks)

    def _solve_mpc_optimization(self):
        """Solve MPC optimization problem"""

        # Prepare current state
        if self.mpc_type == MPCType.KINEMATIC:
            current_state = {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y,
                'v': self.current_velocity,
                'theta': self.current_yaw
            }
        else:  # DYNAMIC
            current_state = {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y,
                'v': self.current_velocity,
                'theta': self.current_yaw,
                'beta': self.current_beta,
                'r': self.current_yaw_rate
            }

        # Ensure reference trajectory is the right size
        if len(self.reference_trajectory) > self.horizon_N + 1:
            reference_traj = self.reference_trajectory[:self.horizon_N + 1]
        else:
            reference_traj = self.reference_trajectory

        # Solve MPC
        result = self.mpc_controller.solve_mpc(current_state, reference_traj)

        return result

    def _publish_control_command(self, acceleration, steering_angle):
        """Publish control command with safety limits from parameters"""

        # Apply safety limits from parameters
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        # Convert acceleration to velocity command
        target_velocity = self.current_velocity + acceleration * (1.0 / self.control_hz)
        target_velocity = np.clip(target_velocity, self.min_speed, self.max_speed)

        # Create and publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = float(target_velocity)
        drive_msg.drive.steering_angle = float(steering_angle)

        self.control_publisher.publish(drive_msg)
        self.current_steering_angle = steering_angle

    def _publish_zero_control(self):
        """Publish zero control command"""
        self._publish_control_command(0.0, 0.0)

    def _publish_emergency_stop(self):
        """Publish emergency stop command"""

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0

        self.control_publisher.publish(drive_msg)
        self.emergency_stop = True
        self.get_logger().warn("🚨 EMERGENCY STOP ACTIVATED")

    def _publish_diagnostics(self):
        """Publish diagnostics information with all parameters"""

        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # MPC Controller Status
        mpc_status = DiagnosticStatus()
        mpc_status.name = "Optimized MPC Controller"
        mpc_status.hardware_id = "optimized_mpc_node"

        if self.control_active and not self.emergency_stop:
            mpc_status.level = DiagnosticStatus.OK
            mpc_status.message = f"Optimized MPC ({self.mpc_type.value}) operating normally with all parameters"
        elif self.emergency_stop:
            mpc_status.level = DiagnosticStatus.ERROR
            mpc_status.message = "Emergency stop active"
        else:
            mpc_status.level = DiagnosticStatus.WARN
            mpc_status.message = "Optimized MPC controller inactive"

        # Add performance metrics
        perf_stats = self.mpc_controller.get_performance_stats()
        if perf_stats:
            mpc_status.values.extend([
                KeyValue(key="avg_solve_time", value=f"{perf_stats['avg_solve_time']:.4f}"),
                KeyValue(key="success_rate", value=f"{perf_stats['success_rate']:.3f}"),
                KeyValue(key="real_time_factor", value=f"{perf_stats['real_time_factor']:.1f}"),
                KeyValue(key="total_iterations", value=str(perf_stats['total_iterations']))
            ])

        # Add parameter status
        mpc_status.values.extend([
            KeyValue(key="mpc_type", value=self.mpc_type.value),
            KeyValue(key="horizon_N", value=str(self.horizon_N)),
            KeyValue(key="max_speed", value=f"{self.max_speed:.2f}"),
            KeyValue(key="max_steering_angle", value=f"{self.max_steering_angle:.3f}"),
            KeyValue(key="safety_checks", value=str(self.enable_safety_checks)),
            KeyValue(key="obstacle_avoidance", value=str(self.enable_obstacle_avoidance)),
            KeyValue(key="cost_weights_enabled", value=str(self.enable_cost_function_weights)),
            KeyValue(key="trajectory_tracking", value=str(self.enable_trajectory_tracking))
        ])

        diag_array.status = [mpc_status]
        self.diagnostics_publisher.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = MPCNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🏁 Optimized MPC Node shutting down...")
    except Exception as e:
        print(f"❌ Fatal error in Optimized MPC Node: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
