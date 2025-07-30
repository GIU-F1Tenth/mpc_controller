#!/usr/bin/env python3
"""
F1TENTH MPC Real-Time Tuning GUI

A comprehensive PyQt5-based GUI for real-time tuning of MPC controller parameters
during F1TENTH autonomous racing. Supports live parameter updates, configuration
save/load, and real-time performance monitoring.

Features:
- Real-time parameter adjustment with sliders and spin boxes
- Live parameter updates via ROS2 parameter server
- Configuration save/load (YAML format)
- Real-time performance monitoring and plotting
- Emergency stop functionality
- Parameter presets (conservative, balanced, aggressive, precision)
- Cost function visualization
- Solver performance metrics

Author: Mohammed Azab <mohammed@azab.io>
Version: 1.0.0
License: MIT
"""

import sys
import os
import yaml
import time
import threading
from collections import deque
from dataclasses import dataclass
from typing import Dict, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import pyqtgraph as pg
from pyqtgraph import PlotWidget


@dataclass
class MPCParameters:
    """Data class to hold all MPC parameters for easy management"""
    # Vehicle parameters
    wheelbase: float = 0.33
    
    # MPC Horizon
    horizon_N: int = 10
    horizon_T: float = 1.0
    lookahead_distance: float = 0.5
    
    # Vehicle limits
    max_steering_angle: float = 0.5
    max_acceleration: float = 2.0
    max_deceleration: float = 2.0
    min_speed: float = 0.1
    max_speed: float = 8.0
    
    # Cost function weights
    enable_cost_function_weights: bool = True
    steering_weight: float = 0.1
    acceleration_weight: float = 0.1
    jerk_weight: float = 0.1
    heading_weight: float = 0.2
    position_weight: float = 0.3
    velocity_weight: float = 0.1
    
    # Hard constraints
    enable_hard_constraints: bool = True
    hard_max_steering_angle: float = 0.4
    hard_max_acceleration: float = 1.5
    hard_max_deceleration: float = 1.5
    
    # Advanced features
    enable_obstacle_avoidance: bool = False
    obstacle_avoidance_weight: float = 0.5
    enable_speed_control: bool = True
    speed_control_weight: float = 0.2
    enable_trajectory_tracking: bool = True
    trajectory_tracking_weight: float = 0.3
    enable_safety_checks: bool = True
    safety_check_distance: float = 0.5
    
    # MPC settings
    mpc_type: str = "kinematic"  # "kinematic" or "dynamic"
    solver_type: str = "ipopt"   # "ipopt" or "sqpmethod"
    control_hz: float = 20.0
    
    # Safety
    safety_timeout: float = 1.0
    emergency_brake_threshold: float = 2.0


class MPCParameterService(Node):
    """ROS2 node for handling parameter updates to the MPC controller"""
    
    def __init__(self):
        super().__init__('mpc_tuning_service')
        self.get_logger().info("MPC Parameter Service started")
        
        # Performance monitoring
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10
        )
        
        # Control monitoring
        self.control_sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.control_callback,
            10
        )
        
        # Odometry monitoring
        self.odom_sub = self.create_subscription(
            Odometry,
            '/car_state/odom',
            self.odom_callback,
            10
        )
        
        # Performance data storage
        self.max_data_points = 200
        self.solve_times = deque(maxlen=self.max_data_points)
        self.success_rates = deque(maxlen=self.max_data_points)
        self.control_commands = deque(maxlen=self.max_data_points)
        self.velocities = deque(maxlen=self.max_data_points)
        self.timestamps = deque(maxlen=self.max_data_points)
        
        # Latest performance data
        self.latest_performance = {}
        self.latest_control = {}
        self.latest_state = {}
        
    def update_parameters(self, params: MPCParameters) -> bool:
        """Update MPC controller parameters via ROS2 parameter server"""
        try:
            # Create parameter client for the MPC node
            param_client = self.create_client(
                rclpy.parameter.SetParameters,
                '/optimized_mpc_node/set_parameters'
            )
            
            if not param_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("MPC node parameter service not available")
                return False
            
            # Prepare parameter list
            parameters = []
            
            # Vehicle parameters
            parameters.extend([
                Parameter('wheelbase', Parameter.Type.DOUBLE, params.wheelbase),
                Parameter('horizon_N', Parameter.Type.INTEGER, params.horizon_N),
                Parameter('horizon_T', Parameter.Type.DOUBLE, params.horizon_T),
                Parameter('lookahead_distance', Parameter.Type.DOUBLE, params.lookahead_distance),
            ])
            
            # Vehicle limits
            parameters.extend([
                Parameter('max_steering_angle', Parameter.Type.DOUBLE, params.max_steering_angle),
                Parameter('max_acceleration', Parameter.Type.DOUBLE, params.max_acceleration),
                Parameter('max_deceleration', Parameter.Type.DOUBLE, params.max_deceleration),
                Parameter('min_speed', Parameter.Type.DOUBLE, params.min_speed),
                Parameter('max_speed', Parameter.Type.DOUBLE, params.max_speed),
            ])
            
            # Cost function weights
            parameters.extend([
                Parameter('enable_cost_function_weights', Parameter.Type.BOOL, params.enable_cost_function_weights),
                Parameter('cost_function_weights.steering_weight', Parameter.Type.DOUBLE, params.steering_weight),
                Parameter('cost_function_weights.acceleration_weight', Parameter.Type.DOUBLE, params.acceleration_weight),
                Parameter('cost_function_weights.jerk_weight', Parameter.Type.DOUBLE, params.jerk_weight),
                Parameter('cost_function_weights.heading_weight', Parameter.Type.DOUBLE, params.heading_weight),
                Parameter('cost_function_weights.position_weight', Parameter.Type.DOUBLE, params.position_weight),
                Parameter('cost_function_weights.velocity_weight', Parameter.Type.DOUBLE, params.velocity_weight),
            ])
            
            # Hard constraints
            parameters.extend([
                Parameter('enable_hard_constraints', Parameter.Type.BOOL, params.enable_hard_constraints),
                Parameter('hard_constraints.max_steering_angle', Parameter.Type.DOUBLE, params.hard_max_steering_angle),
                Parameter('hard_constraints.max_acceleration', Parameter.Type.DOUBLE, params.hard_max_acceleration),
                Parameter('hard_constraints.max_deceleration', Parameter.Type.DOUBLE, params.hard_max_deceleration),
            ])
            
            # Advanced features
            parameters.extend([
                Parameter('enable_obstacle_avoidance', Parameter.Type.BOOL, params.enable_obstacle_avoidance),
                Parameter('obstacle_avoidance_weight', Parameter.Type.DOUBLE, params.obstacle_avoidance_weight),
                Parameter('enable_speed_control', Parameter.Type.BOOL, params.enable_speed_control),
                Parameter('speed_control_weight', Parameter.Type.DOUBLE, params.speed_control_weight),
                Parameter('enable_trajectory_tracking', Parameter.Type.BOOL, params.enable_trajectory_tracking),
                Parameter('trajectory_tracking_weight', Parameter.Type.DOUBLE, params.trajectory_tracking_weight),
                Parameter('enable_safety_checks', Parameter.Type.BOOL, params.enable_safety_checks),
                Parameter('safety_check_distance', Parameter.Type.DOUBLE, params.safety_check_distance),
            ])
            
            # MPC settings
            parameters.extend([
                Parameter('mpc_type', Parameter.Type.STRING, params.mpc_type),
                Parameter('solver_type', Parameter.Type.STRING, params.solver_type),
                Parameter('control_hz', Parameter.Type.DOUBLE, params.control_hz),
                Parameter('safety_timeout', Parameter.Type.DOUBLE, params.safety_timeout),
                Parameter('emergency_brake_threshold', Parameter.Type.DOUBLE, params.emergency_brake_threshold),
            ])
            
            # Send parameters
            request = rclpy.parameter.SetParameters.Request()
            request.parameters = parameters
            
            future = param_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                self.get_logger().info("Parameters updated successfully")
                return True
            else:
                self.get_logger().error("Failed to update parameters")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error updating parameters: {e}")
            return False
    
    def diagnostics_callback(self, msg):
        """Process diagnostic information"""
        for status in msg.status:
            if "MPC" in status.name:
                perf_data = {}
                for value in status.values:
                    try:
                        if value.key in ['avg_solve_time', 'success_rate', 'real_time_factor']:
                            perf_data[value.key] = float(value.value)
                        elif value.key in ['total_iterations']:
                            perf_data[value.key] = int(value.value)
                    except ValueError:
                        pass
                
                if perf_data:
                    self.latest_performance = perf_data
                    current_time = time.time()
                    
                    if 'avg_solve_time' in perf_data:
                        self.solve_times.append(perf_data['avg_solve_time'] * 1000)  # Convert to ms
                    if 'success_rate' in perf_data:
                        self.success_rates.append(perf_data['success_rate'] * 100)  # Convert to percentage
                    
                    self.timestamps.append(current_time)
    
    def control_callback(self, msg):
        """Process control commands"""
        self.latest_control = {
            'steering': msg.drive.steering_angle,
            'acceleration': msg.drive.acceleration,
            'speed': msg.drive.speed
        }
        
        # Store control data
        if len(self.timestamps) > 0:
            self.control_commands.append(abs(msg.drive.steering_angle))
    
    def odom_callback(self, msg):
        """Process odometry data"""
        velocity = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        self.latest_state = {
            'velocity': velocity,
            'position_x': msg.pose.pose.position.x,
            'position_y': msg.pose.pose.position.y
        }
        
        # Store velocity data
        if len(self.timestamps) > 0:
            self.velocities.append(velocity)


class ParameterWidget(QWidget):
    """Widget for adjusting a single parameter with slider and spinbox"""
    
    valueChanged = pyqtSignal(float)
    
    def __init__(self, name: str, value: float, min_val: float, max_val: float, 
                 step: float = 0.01, decimals: int = 3, description: str = ""):
        super().__init__()
        self.name = name
        self.description = description
        
        layout = QHBoxLayout()
        
        # Label
        label = QLabel(f"{name}:")
        label.setMinimumWidth(150)
        if description:
            label.setToolTip(description)
        layout.addWidget(label)
        
        # Slider
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(int(min_val / step))
        self.slider.setMaximum(int(max_val / step))
        self.slider.setValue(int(value / step))
        self.slider.valueChanged.connect(self._slider_changed)
        layout.addWidget(self.slider)
        
        # SpinBox
        self.spinbox = QDoubleSpinBox()
        self.spinbox.setMinimum(min_val)
        self.spinbox.setMaximum(max_val)
        self.spinbox.setSingleStep(step)
        self.spinbox.setDecimals(decimals)
        self.spinbox.setValue(value)
        self.spinbox.valueChanged.connect(self._spinbox_changed)
        self.spinbox.setMinimumWidth(80)
        layout.addWidget(self.spinbox)
        
        self.step = step
        self.setLayout(layout)
    
    def _slider_changed(self):
        value = self.slider.value() * self.step
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(value)
        self.spinbox.blockSignals(False)
        self.valueChanged.emit(value)
    
    def _spinbox_changed(self):
        value = self.spinbox.value()
        self.slider.blockSignals(True)
        self.slider.setValue(int(value / self.step))
        self.slider.blockSignals(False)
        self.valueChanged.emit(value)
    
    def getValue(self) -> float:
        return self.spinbox.value()
    
    def setValue(self, value: float):
        self.slider.blockSignals(True)
        self.spinbox.blockSignals(True)
        self.slider.setValue(int(value / self.step))
        self.spinbox.setValue(value)
        self.slider.blockSignals(False)
        self.spinbox.blockSignals(False)


class CheckBoxWidget(QWidget):
    """Widget for boolean parameters"""
    
    valueChanged = pyqtSignal(bool)
    
    def __init__(self, name: str, value: bool, description: str = ""):
        super().__init__()
        layout = QHBoxLayout()
        
        self.checkbox = QCheckBox(name)
        self.checkbox.setChecked(value)
        self.checkbox.stateChanged.connect(lambda state: self.valueChanged.emit(state == Qt.Checked))
        if description:
            self.checkbox.setToolTip(description)
        
        layout.addWidget(self.checkbox)
        layout.addStretch()
        self.setLayout(layout)
    
    def getValue(self) -> bool:
        return self.checkbox.isChecked()
    
    def setValue(self, value: bool):
        self.checkbox.setChecked(value)


class ComboBoxWidget(QWidget):
    """Widget for string parameters with predefined options"""
    
    valueChanged = pyqtSignal(str)
    
    def __init__(self, name: str, value: str, options: list, description: str = ""):
        super().__init__()
        layout = QHBoxLayout()
        
        label = QLabel(f"{name}:")
        label.setMinimumWidth(150)
        if description:
            label.setToolTip(description)
        layout.addWidget(label)
        
        self.combobox = QComboBox()
        self.combobox.addItems(options)
        self.combobox.setCurrentText(value)
        self.combobox.currentTextChanged.connect(self.valueChanged.emit)
        layout.addWidget(self.combobox)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def getValue(self) -> str:
        return self.combobox.currentText()
    
    def setValue(self, value: str):
        self.combobox.setCurrentText(value)


class MPCTuningGUI(QMainWindow):
    """Main GUI application for MPC tuning"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("F1TENTH MPC Real-Time Tuning GUI v1.0")
        self.setGeometry(100, 100, 1400, 900)
        
        # Initialize ROS2
        rclpy.init()
        self.ros_node = MPCParameterService()
        
        # Start ROS2 in separate thread
        self.ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self.ros_thread.start()
        
        # Current parameters
        self.current_params = MPCParameters()
        
        # Parameter widgets
        self.param_widgets = {}
        
        # Setup timers first
        self.setup_timers()
        
        # Setup UI
        self.setup_ui()
        
        # Load initial parameters from precision config after UI is set up
        self.load_config_file('/home/mohammedazab/ws/src/race_stack/mpc_controller/config/params_precision.yaml')
        
    def setup_timers(self):
        """Setup update and apply timers"""
        # Setup update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(100)  # Update every 100ms
        
        # Auto-apply timer
        self.apply_timer = QTimer()
        self.apply_timer.timeout.connect(self.apply_parameters)
        self.apply_timer.setSingleShot(True)
        
    def _ros_spin(self):
        """Run ROS2 in separate thread"""
        try:
            rclpy.spin(self.ros_node)
        except Exception as e:
            print(f"ROS2 error: {e}")
    
    def setup_ui(self):
        """Setup the main user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QHBoxLayout()
        
        # Left panel - Parameters
        left_panel = self.create_parameter_panel()
        main_layout.addWidget(left_panel, 1)
        
        # Right panel - Monitoring
        right_panel = self.create_monitoring_panel()
        main_layout.addWidget(right_panel, 1)
        
        central_widget.setLayout(main_layout)
        
        # Menu bar
        self.create_menu_bar()
        
        # Status bar
        self.statusBar().showMessage("Ready - Load a config or adjust parameters")
    
    def create_menu_bar(self):
        """Create menu bar with file operations and presets"""
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu('&File')
        
        save_action = QAction('&Save Config...', self)
        save_action.setShortcut('Ctrl+S')
        save_action.triggered.connect(self.save_config)
        file_menu.addAction(save_action)
        
        load_action = QAction('&Load Config...', self)
        load_action.setShortcut('Ctrl+O')
        load_action.triggered.connect(self.load_config)
        file_menu.addAction(load_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction('E&xit', self)
        exit_action.setShortcut('Ctrl+Q')
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # Presets menu
        presets_menu = menubar.addMenu('&Presets')
        
        conservative_action = QAction('&Conservative', self)
        conservative_action.triggered.connect(lambda: self.load_preset('conservative'))
        presets_menu.addAction(conservative_action)
        
        balanced_action = QAction('&Balanced', self)
        balanced_action.triggered.connect(lambda: self.load_preset('balanced'))
        presets_menu.addAction(balanced_action)
        
        aggressive_action = QAction('&Aggressive', self)
        aggressive_action.triggered.connect(lambda: self.load_preset('aggressive'))
        presets_menu.addAction(aggressive_action)
        
        precision_action = QAction('&Precision', self)
        precision_action.triggered.connect(lambda: self.load_preset('precision'))
        presets_menu.addAction(precision_action)
        
        # Control menu
        control_menu = menubar.addMenu('&Control')
        
        apply_action = QAction('&Apply Parameters', self)
        apply_action.setShortcut('Ctrl+A')
        apply_action.triggered.connect(self.apply_parameters)
        control_menu.addAction(apply_action)
        
        emergency_action = QAction('&Emergency Stop', self)
        emergency_action.setShortcut('Ctrl+E')
        emergency_action.triggered.connect(self.emergency_stop)
        control_menu.addAction(emergency_action)
    
    def create_parameter_panel(self):
        """Create the parameter adjustment panel"""
        panel = QWidget()
        layout = QVBoxLayout()
        
        # Create tabs for different parameter groups
        tab_widget = QTabWidget()
        
        # Vehicle & Horizon tab
        vehicle_tab = QScrollArea()
        vehicle_widget = QWidget()
        vehicle_layout = QVBoxLayout()
        
        # Vehicle parameters
        vehicle_group = QGroupBox("Vehicle Parameters")
        vehicle_group_layout = QVBoxLayout()
        
        self.param_widgets['wheelbase'] = ParameterWidget(
            "Wheelbase", self.current_params.wheelbase, 0.2, 0.5, 0.01, 3,
            "Distance between front and rear axles"
        )
        vehicle_group_layout.addWidget(self.param_widgets['wheelbase'])
        
        vehicle_group.setLayout(vehicle_group_layout)
        vehicle_layout.addWidget(vehicle_group)
        
        # MPC Horizon parameters
        horizon_group = QGroupBox("MPC Horizon")
        horizon_group_layout = QVBoxLayout()
        
        # Horizon N (integer parameter)
        horizon_n_widget = QWidget()
        horizon_n_layout = QHBoxLayout()
        horizon_n_layout.addWidget(QLabel("Horizon N:"))
        self.horizon_n_spin = QSpinBox()
        self.horizon_n_spin.setMinimum(3)
        self.horizon_n_spin.setMaximum(50)
        self.horizon_n_spin.setValue(self.current_params.horizon_N)
        self.horizon_n_spin.valueChanged.connect(self._param_changed)
        horizon_n_layout.addWidget(self.horizon_n_spin)
        horizon_n_layout.addStretch()
        horizon_n_widget.setLayout(horizon_n_layout)
        horizon_group_layout.addWidget(horizon_n_widget)
        
        self.param_widgets['horizon_T'] = ParameterWidget(
            "Horizon T", self.current_params.horizon_T, 0.1, 3.0, 0.1, 2,
            "Total prediction horizon time"
        )
        horizon_group_layout.addWidget(self.param_widgets['horizon_T'])
        
        self.param_widgets['lookahead_distance'] = ParameterWidget(
            "Lookahead Dist", self.current_params.lookahead_distance, 0.1, 2.0, 0.1, 2,
            "Lookahead distance for trajectory tracking"
        )
        horizon_group_layout.addWidget(self.param_widgets['lookahead_distance'])
        
        horizon_group.setLayout(horizon_group_layout)
        vehicle_layout.addWidget(horizon_group)
        
        vehicle_widget.setLayout(vehicle_layout)
        vehicle_tab.setWidget(vehicle_widget)
        tab_widget.addTab(vehicle_tab, "Vehicle & Horizon")
        
        # Vehicle Limits tab
        limits_tab = QScrollArea()
        limits_widget = QWidget()
        limits_layout = QVBoxLayout()
        
        limits_group = QGroupBox("Vehicle Limits")
        limits_group_layout = QVBoxLayout()
        
        self.param_widgets['max_steering_angle'] = ParameterWidget(
            "Max Steering", self.current_params.max_steering_angle, 0.1, 1.0, 0.01, 3,
            "Maximum steering angle (radians)"
        )
        limits_group_layout.addWidget(self.param_widgets['max_steering_angle'])
        
        self.param_widgets['max_acceleration'] = ParameterWidget(
            "Max Acceleration", self.current_params.max_acceleration, 0.1, 5.0, 0.1, 2,
            "Maximum acceleration (m/s²)"
        )
        limits_group_layout.addWidget(self.param_widgets['max_acceleration'])
        
        self.param_widgets['max_deceleration'] = ParameterWidget(
            "Max Deceleration", self.current_params.max_deceleration, 0.1, 5.0, 0.1, 2,
            "Maximum deceleration (m/s²)"
        )
        limits_group_layout.addWidget(self.param_widgets['max_deceleration'])
        
        self.param_widgets['min_speed'] = ParameterWidget(
            "Min Speed", self.current_params.min_speed, 0.01, 1.0, 0.01, 3,
            "Minimum allowed speed (m/s)"
        )
        limits_group_layout.addWidget(self.param_widgets['min_speed'])
        
        self.param_widgets['max_speed'] = ParameterWidget(
            "Max Speed", self.current_params.max_speed, 1.0, 15.0, 0.1, 2,
            "Maximum allowed speed (m/s)"
        )
        limits_group_layout.addWidget(self.param_widgets['max_speed'])
        
        limits_group.setLayout(limits_group_layout)
        limits_layout.addWidget(limits_group)
        
        limits_widget.setLayout(limits_layout)
        limits_tab.setWidget(limits_widget)
        tab_widget.addTab(limits_tab, "Vehicle Limits")
        
        # Cost Function tab
        cost_tab = QScrollArea()
        cost_widget = QWidget()
        cost_layout = QVBoxLayout()
        
        cost_group = QGroupBox("Cost Function Weights")
        cost_group_layout = QVBoxLayout()
        
        self.param_widgets['enable_cost_function_weights'] = CheckBoxWidget(
            "Enable Cost Function Weights", self.current_params.enable_cost_function_weights,
            "Enable custom cost function weights"
        )
        cost_group_layout.addWidget(self.param_widgets['enable_cost_function_weights'])
        
        self.param_widgets['steering_weight'] = ParameterWidget(
            "Steering Weight", self.current_params.steering_weight, 0.0, 2.0, 0.01, 3,
            "Penalty for steering effort"
        )
        cost_group_layout.addWidget(self.param_widgets['steering_weight'])
        
        self.param_widgets['acceleration_weight'] = ParameterWidget(
            "Acceleration Weight", self.current_params.acceleration_weight, 0.0, 2.0, 0.01, 3,
            "Penalty for acceleration effort"
        )
        cost_group_layout.addWidget(self.param_widgets['acceleration_weight'])
        
        self.param_widgets['jerk_weight'] = ParameterWidget(
            "Jerk Weight", self.current_params.jerk_weight, 0.0, 2.0, 0.01, 3,
            "Penalty for acceleration changes (smoothness)"
        )
        cost_group_layout.addWidget(self.param_widgets['jerk_weight'])
        
        self.param_widgets['heading_weight'] = ParameterWidget(
            "Heading Weight", self.current_params.heading_weight, 0.0, 2.0, 0.01, 3,
            "Penalty for heading error"
        )
        cost_group_layout.addWidget(self.param_widgets['heading_weight'])
        
        self.param_widgets['position_weight'] = ParameterWidget(
            "Position Weight", self.current_params.position_weight, 0.0, 2.0, 0.01, 3,
            "Penalty for position error"
        )
        cost_group_layout.addWidget(self.param_widgets['position_weight'])
        
        self.param_widgets['velocity_weight'] = ParameterWidget(
            "Velocity Weight", self.current_params.velocity_weight, 0.0, 2.0, 0.01, 3,
            "Penalty for velocity error"
        )
        cost_group_layout.addWidget(self.param_widgets['velocity_weight'])
        
        cost_group.setLayout(cost_group_layout)
        cost_layout.addWidget(cost_group)
        
        cost_widget.setLayout(cost_layout)
        cost_tab.setWidget(cost_widget)
        tab_widget.addTab(cost_tab, "Cost Function")
        
        # Constraints & Safety tab
        constraints_tab = QScrollArea()
        constraints_widget = QWidget()
        constraints_layout = QVBoxLayout()
        
        # Hard constraints
        hard_constraints_group = QGroupBox("Hard Constraints")
        hard_constraints_layout = QVBoxLayout()
        
        self.param_widgets['enable_hard_constraints'] = CheckBoxWidget(
            "Enable Hard Constraints", self.current_params.enable_hard_constraints,
            "Enable hard constraints (stricter limits)"
        )
        hard_constraints_layout.addWidget(self.param_widgets['enable_hard_constraints'])
        
        self.param_widgets['hard_max_steering_angle'] = ParameterWidget(
            "Hard Max Steering", self.current_params.hard_max_steering_angle, 0.1, 1.0, 0.01, 3,
            "Hard constraint on steering angle"
        )
        hard_constraints_layout.addWidget(self.param_widgets['hard_max_steering_angle'])
        
        self.param_widgets['hard_max_acceleration'] = ParameterWidget(
            "Hard Max Accel", self.current_params.hard_max_acceleration, 0.1, 3.0, 0.1, 2,
            "Hard constraint on acceleration"
        )
        hard_constraints_layout.addWidget(self.param_widgets['hard_max_acceleration'])
        
        self.param_widgets['hard_max_deceleration'] = ParameterWidget(
            "Hard Max Decel", self.current_params.hard_max_deceleration, 0.1, 3.0, 0.1, 2,
            "Hard constraint on deceleration"
        )
        hard_constraints_layout.addWidget(self.param_widgets['hard_max_deceleration'])
        
        hard_constraints_group.setLayout(hard_constraints_layout)
        constraints_layout.addWidget(hard_constraints_group)
        
        # Safety settings
        safety_group = QGroupBox("Safety Settings")
        safety_layout = QVBoxLayout()
        
        self.param_widgets['enable_safety_checks'] = CheckBoxWidget(
            "Enable Safety Checks", self.current_params.enable_safety_checks,
            "Enable safety checking systems"
        )
        safety_layout.addWidget(self.param_widgets['enable_safety_checks'])
        
        self.param_widgets['safety_check_distance'] = ParameterWidget(
            "Safety Distance", self.current_params.safety_check_distance, 0.1, 2.0, 0.1, 2,
            "Minimum safety distance"
        )
        safety_layout.addWidget(self.param_widgets['safety_check_distance'])
        
        self.param_widgets['safety_timeout'] = ParameterWidget(
            "Safety Timeout", self.current_params.safety_timeout, 0.1, 5.0, 0.1, 2,
            "Timeout for safety checks"
        )
        safety_layout.addWidget(self.param_widgets['safety_timeout'])
        
        self.param_widgets['emergency_brake_threshold'] = ParameterWidget(
            "Emergency Brake", self.current_params.emergency_brake_threshold, 1.0, 5.0, 0.1, 2,
            "Emergency brake threshold multiplier"
        )
        safety_layout.addWidget(self.param_widgets['emergency_brake_threshold'])
        
        safety_group.setLayout(safety_layout)
        constraints_layout.addWidget(safety_group)
        
        constraints_widget.setLayout(constraints_layout)
        constraints_tab.setWidget(constraints_widget)
        tab_widget.addTab(constraints_tab, "Constraints & Safety")
        
        # Advanced Features tab
        advanced_tab = QScrollArea()
        advanced_widget = QWidget()
        advanced_layout = QVBoxLayout()
        
        # Advanced features
        features_group = QGroupBox("Advanced Features")
        features_layout = QVBoxLayout()
        
        self.param_widgets['enable_obstacle_avoidance'] = CheckBoxWidget(
            "Enable Obstacle Avoidance", self.current_params.enable_obstacle_avoidance,
            "Enable obstacle avoidance system"
        )
        features_layout.addWidget(self.param_widgets['enable_obstacle_avoidance'])
        
        self.param_widgets['obstacle_avoidance_weight'] = ParameterWidget(
            "Obstacle Weight", self.current_params.obstacle_avoidance_weight, 0.0, 3.0, 0.1, 2,
            "Weight for obstacle avoidance cost"
        )
        features_layout.addWidget(self.param_widgets['obstacle_avoidance_weight'])
        
        self.param_widgets['enable_speed_control'] = CheckBoxWidget(
            "Enable Speed Control", self.current_params.enable_speed_control,
            "Enable advanced speed control"
        )
        features_layout.addWidget(self.param_widgets['enable_speed_control'])
        
        self.param_widgets['speed_control_weight'] = ParameterWidget(
            "Speed Control Weight", self.current_params.speed_control_weight, 0.0, 2.0, 0.01, 3,
            "Weight for speed control cost"
        )
        features_layout.addWidget(self.param_widgets['speed_control_weight'])
        
        self.param_widgets['enable_trajectory_tracking'] = CheckBoxWidget(
            "Enable Traj Tracking", self.current_params.enable_trajectory_tracking,
            "Enable enhanced trajectory tracking"
        )
        features_layout.addWidget(self.param_widgets['enable_trajectory_tracking'])
        
        self.param_widgets['trajectory_tracking_weight'] = ParameterWidget(
            "Traj Tracking Weight", self.current_params.trajectory_tracking_weight, 0.0, 2.0, 0.01, 3,
            "Weight for trajectory tracking cost"
        )
        features_layout.addWidget(self.param_widgets['trajectory_tracking_weight'])
        
        features_group.setLayout(features_layout)
        advanced_layout.addWidget(features_group)
        
        # MPC Settings
        mpc_settings_group = QGroupBox("MPC Settings")
        mpc_settings_layout = QVBoxLayout()
        
        self.param_widgets['mpc_type'] = ComboBoxWidget(
            "MPC Type", self.current_params.mpc_type, ["kinematic", "dynamic"],
            "Choose between kinematic or dynamic bicycle model"
        )
        mpc_settings_layout.addWidget(self.param_widgets['mpc_type'])
        
        self.param_widgets['solver_type'] = ComboBoxWidget(
            "Solver Type", self.current_params.solver_type, ["ipopt", "sqpmethod"],
            "Choose optimization solver"
        )
        mpc_settings_layout.addWidget(self.param_widgets['solver_type'])
        
        self.param_widgets['control_hz'] = ParameterWidget(
            "Control Hz", self.current_params.control_hz, 5.0, 50.0, 1.0, 1,
            "Control loop frequency"
        )
        mpc_settings_layout.addWidget(self.param_widgets['control_hz'])
        
        mpc_settings_group.setLayout(mpc_settings_layout)
        advanced_layout.addWidget(mpc_settings_group)
        
        advanced_widget.setLayout(advanced_layout)
        advanced_tab.setWidget(advanced_widget)
        tab_widget.addTab(advanced_tab, "Advanced & MPC")
        
        layout.addWidget(tab_widget)
        
        # Control buttons
        button_layout = QHBoxLayout()
        
        apply_btn = QPushButton("Apply Parameters")
        apply_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }")
        apply_btn.clicked.connect(self.apply_parameters)
        button_layout.addWidget(apply_btn)
        
        emergency_btn = QPushButton("EMERGENCY STOP")
        emergency_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; }")
        emergency_btn.clicked.connect(self.emergency_stop)
        button_layout.addWidget(emergency_btn)
        
        # Auto-apply checkbox
        self.auto_apply_cb = QCheckBox("Auto-apply (500ms delay)")
        self.auto_apply_cb.setChecked(True)
        button_layout.addWidget(self.auto_apply_cb)
        
        layout.addLayout(button_layout)
        
        # Connect all parameter change signals
        for widget in self.param_widgets.values():
            if hasattr(widget, 'valueChanged'):
                widget.valueChanged.connect(self._param_changed)
        
        self.horizon_n_spin.valueChanged.connect(self._param_changed)
        
        panel.setLayout(layout)
        return panel
    
    def create_monitoring_panel(self):
        """Create the monitoring and plotting panel"""
        panel = QWidget()
        layout = QVBoxLayout()
        
        # Status group
        status_group = QGroupBox("MPC Status")
        status_layout = QGridLayout()
        
        # Status labels
        self.status_labels = {}
        status_items = [
            ("Solve Time", "solve_time", "ms"),
            ("Success Rate", "success_rate", "%"),
            ("Real-time Factor", "real_time_factor", "x"),
            ("Current Speed", "speed", "m/s"),
            ("Steering Angle", "steering", "rad"),
            ("Acceleration", "acceleration", "m/s²")
        ]
        
        for i, (name, key, unit) in enumerate(status_items):
            row, col = i // 3, (i % 3) * 2
            
            label = QLabel(f"{name}:")
            status_layout.addWidget(label, row, col)
            
            value_label = QLabel("--")
            value_label.setStyleSheet("QLabel { font-weight: bold; color: blue; }")
            self.status_labels[key] = (value_label, unit)
            status_layout.addWidget(value_label, row, col + 1)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # Plots
        plots_tab = QTabWidget()
        
        # Performance plots
        perf_widget = QWidget()
        perf_layout = QVBoxLayout()
        
        # Solve time plot
        self.solve_time_plot = PlotWidget(title="Solver Performance")
        self.solve_time_plot.setLabel('left', 'Solve Time', units='ms')
        self.solve_time_plot.setLabel('bottom', 'Time', units='s')
        self.solve_time_plot.showGrid(x=True, y=True)
        perf_layout.addWidget(self.solve_time_plot)
        
        # Success rate plot
        self.success_rate_plot = PlotWidget(title="Success Rate")
        self.success_rate_plot.setLabel('left', 'Success Rate', units='%')
        self.success_rate_plot.setLabel('bottom', 'Time', units='s')
        self.success_rate_plot.showGrid(x=True, y=True)
        perf_layout.addWidget(self.success_rate_plot)
        
        perf_widget.setLayout(perf_layout)
        plots_tab.addTab(perf_widget, "Performance")
        
        # Control plots
        control_widget = QWidget()
        control_layout = QVBoxLayout()
        
        # Steering plot
        self.steering_plot = PlotWidget(title="Steering Commands")
        self.steering_plot.setLabel('left', 'Steering Angle', units='rad')
        self.steering_plot.setLabel('bottom', 'Time', units='s')
        self.steering_plot.showGrid(x=True, y=True)
        control_layout.addWidget(self.steering_plot)
        
        # Velocity plot
        self.velocity_plot = PlotWidget(title="Vehicle Velocity")
        self.velocity_plot.setLabel('left', 'Velocity', units='m/s')
        self.velocity_plot.setLabel('bottom', 'Time', units='s')
        self.velocity_plot.showGrid(x=True, y=True)
        control_layout.addWidget(self.velocity_plot)
        
        control_widget.setLayout(control_layout)
        plots_tab.addTab(control_widget, "Control")
        
        layout.addWidget(plots_tab)
        
        panel.setLayout(layout)
        return panel
    
    def _param_changed(self):
        """Handle parameter changes"""
        # Update current parameters from widgets
        self.update_current_params()
        
        # Auto-apply if enabled and checkbox exists
        if hasattr(self, 'auto_apply_cb') and self.auto_apply_cb.isChecked():
            self.apply_timer.start(500)  # 500ms delay
    
    def update_current_params(self):
        """Update current parameters from widgets"""
        # Update from regular parameter widgets
        for param_name, widget in self.param_widgets.items():
            if hasattr(self.current_params, param_name):
                setattr(self.current_params, param_name, widget.getValue())
        
        # Update horizon_N from spinbox
        self.current_params.horizon_N = self.horizon_n_spin.value()
    
    def apply_parameters(self):
        """Apply current parameters to MPC controller"""
        self.update_current_params()
        
        success = self.ros_node.update_parameters(self.current_params)
        
        if success:
            self.statusBar().showMessage("Parameters applied successfully", 2000)
        else:
            self.statusBar().showMessage("Failed to apply parameters", 3000)
            QMessageBox.warning(self, "Error", "Failed to apply parameters to MPC controller")
    
    def emergency_stop(self):
        """Trigger emergency stop"""
        # You can implement emergency stop logic here
        # For example, setting max_speed to 0 or sending a stop command
        
        reply = QMessageBox.question(self, "Emergency Stop", 
                                   "This will set max speed to 0 and apply safety parameters. Continue?",
                                   QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            # Set safety parameters
            self.param_widgets['max_speed'].setValue(0.1)
            self.param_widgets['max_acceleration'].setValue(0.1)
            self.param_widgets['enable_safety_checks'].setValue(True)
            self.param_widgets['safety_timeout'].setValue(0.5)
            
            # Apply immediately
            self.apply_parameters()
            
            self.statusBar().showMessage("EMERGENCY STOP ACTIVATED", 5000)
    
    def save_config(self):
        """Save current configuration to YAML file"""
        filename, _ = QFileDialog.getSaveFileName(self, "Save Configuration", "", "YAML Files (*.yaml)")
        
        if filename:
            try:
                self.update_current_params()
                config_dict = {
                    'optimized_mpc_controller': {
                        'ros__parameters': {
                            # Vehicle parameters
                            'wheelbase': self.current_params.wheelbase,
                            'horizon_N': self.current_params.horizon_N,
                            'horizon_T': self.current_params.horizon_T,
                            'lookahead_distance': self.current_params.lookahead_distance,
                            
                            # Vehicle limits
                            'max_steering_angle': self.current_params.max_steering_angle,
                            'max_acceleration': self.current_params.max_acceleration,
                            'max_deceleration': self.current_params.max_deceleration,
                            'min_speed': self.current_params.min_speed,
                            'max_speed': self.current_params.max_speed,
                            
                            # Cost function weights
                            'enable_cost_function_weights': self.current_params.enable_cost_function_weights,
                            'cost_function_weights': {
                                'steering_weight': self.current_params.steering_weight,
                                'acceleration_weight': self.current_params.acceleration_weight,
                                'jerk_weight': self.current_params.jerk_weight,
                                'heading_weight': self.current_params.heading_weight,
                                'position_weight': self.current_params.position_weight,
                                'velocity_weight': self.current_params.velocity_weight,
                            },
                            
                            # Hard constraints
                            'enable_hard_constraints': self.current_params.enable_hard_constraints,
                            'hard_constraints': {
                                'max_steering_angle': self.current_params.hard_max_steering_angle,
                                'max_acceleration': self.current_params.hard_max_acceleration,
                                'max_deceleration': self.current_params.hard_max_deceleration,
                            },
                            
                            # Advanced features
                            'enable_obstacle_avoidance': self.current_params.enable_obstacle_avoidance,
                            'obstacle_avoidance_weight': self.current_params.obstacle_avoidance_weight,
                            'enable_speed_control': self.current_params.enable_speed_control,
                            'speed_control_weight': self.current_params.speed_control_weight,
                            'enable_trajectory_tracking': self.current_params.enable_trajectory_tracking,
                            'trajectory_tracking_weight': self.current_params.trajectory_tracking_weight,
                            'enable_safety_checks': self.current_params.enable_safety_checks,
                            'safety_check_distance': self.current_params.safety_check_distance,
                            
                            # MPC settings
                            'mpc_type': self.current_params.mpc_type,
                            'solver_type': self.current_params.solver_type,
                            'control_hz': self.current_params.control_hz,
                            'safety_timeout': self.current_params.safety_timeout,
                            'emergency_brake_threshold': self.current_params.emergency_brake_threshold,
                            
                            # Additional required parameters
                            'enable_logging': True,
                            'odom_topic': "/car_state/odom",
                            'reference_topic': "/mpc/reference_trajectory",
                            'status_topic': "/mpc/path_ready",
                            'control_topic': "/drive",
                            'qos_depth': 10
                        }
                    }
                }
                
                with open(filename, 'w') as f:
                    yaml.dump(config_dict, f, default_flow_style=False, indent=2)
                
                self.statusBar().showMessage(f"Configuration saved to {filename}", 3000)
                
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save configuration: {e}")
    
    def load_config(self):
        """Load configuration from YAML file"""
        filename, _ = QFileDialog.getOpenFileName(self, "Load Configuration", "", "YAML Files (*.yaml)")
        
        if filename:
            self.load_config_file(filename)
    
    def load_config_file(self, filename: str):
        """Load configuration from specific file"""
        try:
            if not os.path.exists(filename):
                print(f"Warning: Config file not found: {filename}")
                return
                
            with open(filename, 'r') as f:
                config = yaml.safe_load(f)
            
            # Extract parameters
            params = config['optimized_mpc_controller']['ros__parameters']
            
            # Update current parameters
            self.current_params.wheelbase = params.get('wheelbase', 0.33)
            self.current_params.horizon_N = params.get('horizon_N', 10)
            self.current_params.horizon_T = params.get('horizon_T', 1.0)
            self.current_params.lookahead_distance = params.get('lookahead_distance', 0.5)
            
            self.current_params.max_steering_angle = params.get('max_steering_angle', 0.5)
            self.current_params.max_acceleration = params.get('max_acceleration', 2.0)
            self.current_params.max_deceleration = params.get('max_deceleration', 2.0)
            self.current_params.min_speed = params.get('min_speed', 0.1)
            self.current_params.max_speed = params.get('max_speed', 8.0)
            
            self.current_params.enable_cost_function_weights = params.get('enable_cost_function_weights', True)
            
            # Cost function weights
            cost_weights = params.get('cost_function_weights', {})
            self.current_params.steering_weight = cost_weights.get('steering_weight', 0.1)
            self.current_params.acceleration_weight = cost_weights.get('acceleration_weight', 0.1)
            self.current_params.jerk_weight = cost_weights.get('jerk_weight', 0.1)
            self.current_params.heading_weight = cost_weights.get('heading_weight', 0.2)
            self.current_params.position_weight = cost_weights.get('position_weight', 0.3)
            self.current_params.velocity_weight = cost_weights.get('velocity_weight', 0.1)
            
            # Hard constraints
            self.current_params.enable_hard_constraints = params.get('enable_hard_constraints', True)
            hard_constraints = params.get('hard_constraints', {})
            self.current_params.hard_max_steering_angle = hard_constraints.get('max_steering_angle', 0.4)
            self.current_params.hard_max_acceleration = hard_constraints.get('max_acceleration', 1.5)
            self.current_params.hard_max_deceleration = hard_constraints.get('max_deceleration', 1.5)
            
            # Advanced features
            self.current_params.enable_obstacle_avoidance = params.get('enable_obstacle_avoidance', False)
            self.current_params.obstacle_avoidance_weight = params.get('obstacle_avoidance_weight', 0.5)
            self.current_params.enable_speed_control = params.get('enable_speed_control', True)
            self.current_params.speed_control_weight = params.get('speed_control_weight', 0.2)
            self.current_params.enable_trajectory_tracking = params.get('enable_trajectory_tracking', True)
            self.current_params.trajectory_tracking_weight = params.get('trajectory_tracking_weight', 0.3)
            self.current_params.enable_safety_checks = params.get('enable_safety_checks', True)
            self.current_params.safety_check_distance = params.get('safety_check_distance', 0.5)
            
            # MPC settings
            self.current_params.mpc_type = params.get('mpc_type', 'kinematic')
            self.current_params.solver_type = params.get('solver_type', 'ipopt')
            self.current_params.control_hz = params.get('control_hz', 20.0)
            self.current_params.safety_timeout = params.get('safety_timeout', 1.0)
            self.current_params.emergency_brake_threshold = params.get('emergency_brake_threshold', 2.0)
            
            # Update GUI widgets
            self.update_widgets_from_params()
            
            # Update status bar if it exists
            if hasattr(self, 'statusBar'):
                self.statusBar().showMessage(f"Configuration loaded from {filename}", 3000)
            else:
                print(f"Configuration loaded from {filename}")
            
        except FileNotFoundError:
            error_msg = f"Configuration file not found: {filename}"
            print(f"Error: {error_msg}")
            if hasattr(self, 'statusBar'):
                self.statusBar().showMessage(error_msg, 3000)
        except Exception as e:
            error_msg = f"Failed to load configuration: {e}"
            print(f"Error: {error_msg}")
            if hasattr(self, 'statusBar'):
                self.statusBar().showMessage(error_msg, 3000)
            # Only show message box if GUI is fully initialized
            if hasattr(self, 'param_widgets') and self.param_widgets:
                QMessageBox.critical(self, "Error", error_msg)
    
    def update_widgets_from_params(self):
        """Update GUI widgets from current parameters"""
        # Check if widgets are initialized
        if not hasattr(self, 'param_widgets') or not self.param_widgets:
            return
            
        # Update parameter widgets
        for param_name, widget in self.param_widgets.items():
            if hasattr(self.current_params, param_name):
                value = getattr(self.current_params, param_name)
                widget.setValue(value)
        
        # Update horizon_N spinbox if it exists
        if hasattr(self, 'horizon_n_spin'):
            self.horizon_n_spin.setValue(self.current_params.horizon_N)
    
    def load_preset(self, preset_name: str):
        """Load predefined parameter presets"""
        config_files = {
            'conservative': '/home/mohammedazab/ws/src/race_stack/mpc_controller/config/params_conservative.yaml',
            'balanced': '/home/mohammedazab/ws/src/race_stack/mpc_controller/config/params.yaml',
            'aggressive': '/home/mohammedazab/ws/src/race_stack/mpc_controller/config/params_aggressive.yaml',
            'precision': '/home/mohammedazab/ws/src/race_stack/mpc_controller/config/params_precision.yaml'
        }
        
        if preset_name in config_files:
            filename = config_files[preset_name]
            if os.path.exists(filename):
                self.load_config_file(filename)
                self.statusBar().showMessage(f"Loaded {preset_name} preset", 2000)
            else:
                QMessageBox.warning(self, "Error", f"Preset file not found: {filename}")
        else:
            QMessageBox.warning(self, "Error", f"Unknown preset: {preset_name}")
    
    def update_plots(self):
        """Update real-time plots"""
        try:
            # Update status labels
            perf_data = self.ros_node.latest_performance
            control_data = self.ros_node.latest_control
            state_data = self.ros_node.latest_state
            
            # Update performance status
            if 'avg_solve_time' in perf_data:
                self.status_labels['solve_time'][0].setText(f"{perf_data['avg_solve_time']*1000:.1f}")
            if 'success_rate' in perf_data:
                self.status_labels['success_rate'][0].setText(f"{perf_data['success_rate']*100:.1f}")
            if 'real_time_factor' in perf_data:
                self.status_labels['real_time_factor'][0].setText(f"{perf_data['real_time_factor']:.1f}")
            
            # Update control status
            if 'speed' in control_data:
                self.status_labels['speed'][0].setText(f"{control_data['speed']:.2f}")
            if 'steering' in control_data:
                self.status_labels['steering'][0].setText(f"{control_data['steering']:.3f}")
            if 'acceleration' in control_data:
                self.status_labels['acceleration'][0].setText(f"{control_data['acceleration']:.2f}")
            
            # Update state status
            if 'velocity' in state_data:
                self.status_labels['speed'][0].setText(f"{state_data['velocity']:.2f}")
            
            # Update plots if we have data
            if len(self.ros_node.timestamps) > 1:
                # Create time array relative to start
                times = np.array(self.ros_node.timestamps)
                if len(times) > 0:
                    times = times - times[0]  # Make relative to start
                    
                    # Solve time plot
                    if len(self.ros_node.solve_times) > 0:
                        self.solve_time_plot.clear()
                        self.solve_time_plot.plot(times[:len(self.ros_node.solve_times)], 
                                                self.ros_node.solve_times, pen='b')
                    
                    # Success rate plot
                    if len(self.ros_node.success_rates) > 0:
                        self.success_rate_plot.clear()
                        self.success_rate_plot.plot(times[:len(self.ros_node.success_rates)], 
                                                  self.ros_node.success_rates, pen='g')
                    
                    # Control plots
                    if len(self.ros_node.control_commands) > 0:
                        self.steering_plot.clear()
                        self.steering_plot.plot(times[:len(self.ros_node.control_commands)], 
                                              self.ros_node.control_commands, pen='r')
                    
                    if len(self.ros_node.velocities) > 0:
                        self.velocity_plot.clear()
                        self.velocity_plot.plot(times[:len(self.ros_node.velocities)], 
                                              self.ros_node.velocities, pen='m')
                        
        except Exception as e:
            # Silently handle plotting errors to avoid GUI freezing
            pass
    
    def closeEvent(self, event):
        """Handle application close"""
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main():
    """Main application entry point"""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Modern look
    
    # Set application properties
    app.setApplicationName("MPC Tuning GUI")
    app.setApplicationVersion("1.0.0")
    app.setOrganizationName("F1TENTH")
    
    # Create and show main window
    window = MPCTuningGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
