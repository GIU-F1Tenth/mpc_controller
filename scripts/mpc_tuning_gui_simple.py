#!/usr/bin/env python3
"""
F1TENTH MPC Simple Tuning GUI (Tkinter)

A lightweight tkinter-based GUI for real-time tuning of MPC controller parameters
during F1TENTH autonomous racing. This is a simpler alternative to the PyQt5 version.

Features:
- Real-time parameter adjustment with sliders and entry boxes
- Live parameter updates via ROS2 parameter server
- Configuration save/load (YAML format)
- Parameter presets (conservative, balanced, aggressive, precision)
- Emergency stop functionality
- Basic performance monitoring

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
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import json

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np


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
    
    def control_callback(self, msg):
        """Process control commands"""
        self.latest_control = {
            'steering': msg.drive.steering_angle,
            'acceleration': msg.drive.acceleration,
            'speed': msg.drive.speed
        }
    
    def odom_callback(self, msg):
        """Process odometry data"""
        velocity = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        self.latest_state = {
            'velocity': velocity,
            'position_x': msg.pose.pose.position.x,
            'position_y': msg.pose.pose.position.y
        }


class ParameterFrame(ttk.Frame):
    """Frame for adjusting a single parameter with scale and entry"""
    
    def __init__(self, parent, name, value, min_val, max_val, resolution=0.01, callback=None):
        super().__init__(parent)
        self.name = name
        self.callback = callback
        self.min_val = min_val
        self.max_val = max_val
        self.resolution = resolution
        
        # Create layout
        self.grid_columnconfigure(1, weight=1)
        
        # Label
        self.label = ttk.Label(self, text=f"{name}:", width=20)
        self.label.grid(row=0, column=0, padx=5, pady=2, sticky='w')
        
        # Scale
        self.scale = ttk.Scale(self, from_=min_val, to=max_val, orient=tk.HORIZONTAL)
        self.scale.set(value)
        self.scale.grid(row=0, column=1, padx=5, pady=2, sticky='ew')
        self.scale.configure(command=self._scale_changed)
        
        # Entry
        self.var = tk.StringVar(value=str(value))
        self.entry = ttk.Entry(self, textvariable=self.var, width=10)
        self.entry.grid(row=0, column=2, padx=5, pady=2)
        self.var.trace('w', self._entry_changed)
        
    def _scale_changed(self, value):
        """Handle scale changes"""
        val = float(value)
        self.var.set(f"{val:.3f}")
        if self.callback:
            self.callback(self.name, val)
    
    def _entry_changed(self, *args):
        """Handle entry changes"""
        try:
            val = float(self.var.get())
            if self.min_val <= val <= self.max_val:
                self.scale.set(val)
                if self.callback:
                    self.callback(self.name, val)
        except ValueError:
            pass
    
    def get_value(self):
        """Get current value"""
        return self.scale.get()
    
    def set_value(self, value):
        """Set value"""
        self.scale.set(value)
        self.var.set(f"{value:.3f}")


class CheckFrame(ttk.Frame):
    """Frame for boolean parameters"""
    
    def __init__(self, parent, name, value, callback=None):
        super().__init__(parent)
        self.name = name
        self.callback = callback
        
        self.var = tk.BooleanVar(value=value)
        self.check = ttk.Checkbutton(self, text=name, variable=self.var, command=self._changed)
        self.check.pack(anchor='w')
    
    def _changed(self):
        """Handle checkbox changes"""
        if self.callback:
            self.callback(self.name, self.var.get())
    
    def get_value(self):
        """Get current value"""
        return self.var.get()
    
    def set_value(self, value):
        """Set value"""
        self.var.set(value)


class ComboFrame(ttk.Frame):
    """Frame for string parameters with options"""
    
    def __init__(self, parent, name, value, options, callback=None):
        super().__init__(parent)
        self.name = name
        self.callback = callback
        
        # Label
        self.label = ttk.Label(self, text=f"{name}:", width=20)
        self.label.pack(side=tk.LEFT, padx=5)
        
        # Combobox
        self.var = tk.StringVar(value=value)
        self.combo = ttk.Combobox(self, textvariable=self.var, values=options, state='readonly')
        self.combo.pack(side=tk.LEFT, padx=5)
        self.combo.bind('<<ComboboxSelected>>', self._changed)
    
    def _changed(self, event=None):
        """Handle combobox changes"""
        if self.callback:
            self.callback(self.name, self.var.get())
    
    def get_value(self):
        """Get current value"""
        return self.var.get()
    
    def set_value(self, value):
        """Set value"""
        self.var.set(value)


class MPCTuningGUI:
    """Main GUI application for MPC tuning using tkinter"""
    
    def __init__(self):
        # Initialize ROS2
        rclpy.init()
        self.ros_node = MPCParameterService()
        
        # Start ROS2 in separate thread
        self.ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self.ros_thread.start()
        
        # Current parameters
        self.current_params = MPCParameters()
        
        # Load initial parameters from precision config
        self.load_config_file('/home/mohammedazab/ws/src/race_stack/mpc_controller/config/params_precision.yaml')
        
        # Parameter widgets
        self.param_widgets = {}
        
        # Create GUI
        self.setup_gui()
        
        # Auto-apply tracking
        self.auto_apply_var = tk.BooleanVar(value=True)
        self.apply_timer = None
        
        # Start status update timer
        self.update_status()
        
    def _ros_spin(self):
        """Run ROS2 in separate thread"""
        try:
            rclpy.spin(self.ros_node)
        except Exception as e:
            print(f"ROS2 error: {e}")
    
    def setup_gui(self):
        """Setup the main user interface"""
        self.root = tk.Tk()
        self.root.title("F1TENTH MPC Tuning GUI (Simple)")
        self.root.geometry("1000x800")
        
        # Create main notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create tabs
        self.create_basic_tab()
        self.create_cost_function_tab()
        self.create_constraints_tab()
        self.create_advanced_tab()
        self.create_status_tab()
        
        # Control frame
        self.create_control_frame()
        
        # Menu
        self.create_menu()
    
    def create_menu(self):
        """Create menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        
        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Save Config...", command=self.save_config, accelerator="Ctrl+S")
        file_menu.add_command(label="Load Config...", command=self.load_config, accelerator="Ctrl+O")
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit, accelerator="Ctrl+Q")
        
        # Presets menu
        presets_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Presets", menu=presets_menu)
        presets_menu.add_command(label="Conservative", command=lambda: self.load_preset('conservative'))
        presets_menu.add_command(label="Balanced", command=lambda: self.load_preset('balanced'))
        presets_menu.add_command(label="Aggressive", command=lambda: self.load_preset('aggressive'))
        presets_menu.add_command(label="Precision", command=lambda: self.load_preset('precision'))
        
        # Control menu
        control_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Control", menu=control_menu)
        control_menu.add_command(label="Apply Parameters", command=self.apply_parameters, accelerator="Ctrl+A")
        control_menu.add_command(label="Emergency Stop", command=self.emergency_stop, accelerator="Ctrl+E")
        
        # Bind keyboard shortcuts
        self.root.bind('<Control-s>', lambda e: self.save_config())
        self.root.bind('<Control-o>', lambda e: self.load_config())
        self.root.bind('<Control-a>', lambda e: self.apply_parameters())
        self.root.bind('<Control-e>', lambda e: self.emergency_stop())
        self.root.bind('<Control-q>', lambda e: self.root.quit())
    
    def create_basic_tab(self):
        """Create basic parameters tab"""
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Basic Parameters")
        
        # Create scrollable frame
        canvas = tk.Canvas(frame)
        scrollbar = ttk.Scrollbar(frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Vehicle parameters
        vehicle_frame = ttk.LabelFrame(scrollable_frame, text="Vehicle Parameters")
        vehicle_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.param_widgets['wheelbase'] = ParameterFrame(
            vehicle_frame, "Wheelbase", self.current_params.wheelbase, 0.2, 0.5, 0.01, self._param_changed
        )
        self.param_widgets['wheelbase'].pack(fill=tk.X, padx=5, pady=2)
        
        # MPC Horizon
        horizon_frame = ttk.LabelFrame(scrollable_frame, text="MPC Horizon")
        horizon_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Horizon N (integer)
        horizon_n_frame = ttk.Frame(horizon_frame)
        horizon_n_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(horizon_n_frame, text="Horizon N:", width=20).pack(side=tk.LEFT, padx=5)
        
        self.horizon_n_var = tk.IntVar(value=self.current_params.horizon_N)
        horizon_n_spin = tk.Spinbox(horizon_n_frame, from_=3, to=50, textvariable=self.horizon_n_var, 
                                   command=lambda: self._param_changed('horizon_N', self.horizon_n_var.get()))
        horizon_n_spin.pack(side=tk.LEFT, padx=5)
        
        self.param_widgets['horizon_T'] = ParameterFrame(
            horizon_frame, "Horizon T", self.current_params.horizon_T, 0.1, 3.0, 0.1, self._param_changed
        )
        self.param_widgets['horizon_T'].pack(fill=tk.X, padx=5, pady=2)
        
        self.param_widgets['lookahead_distance'] = ParameterFrame(
            horizon_frame, "Lookahead Dist", self.current_params.lookahead_distance, 0.1, 2.0, 0.1, self._param_changed
        )
        self.param_widgets['lookahead_distance'].pack(fill=tk.X, padx=5, pady=2)
        
        # Vehicle limits
        limits_frame = ttk.LabelFrame(scrollable_frame, text="Vehicle Limits")
        limits_frame.pack(fill=tk.X, padx=5, pady=5)
        
        limits_params = [
            ('max_steering_angle', "Max Steering", 0.1, 1.0),
            ('max_acceleration', "Max Acceleration", 0.1, 5.0),
            ('max_deceleration', "Max Deceleration", 0.1, 5.0),
            ('min_speed', "Min Speed", 0.01, 1.0),
            ('max_speed', "Max Speed", 1.0, 15.0)
        ]
        
        for param_name, display_name, min_val, max_val in limits_params:
            value = getattr(self.current_params, param_name)
            self.param_widgets[param_name] = ParameterFrame(
                limits_frame, display_name, value, min_val, max_val, 0.01, self._param_changed
            )
            self.param_widgets[param_name].pack(fill=tk.X, padx=5, pady=2)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
    
    def create_cost_function_tab(self):
        """Create cost function parameters tab"""
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Cost Function")
        
        # Create scrollable frame
        canvas = tk.Canvas(frame)
        scrollbar = ttk.Scrollbar(frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Cost function weights
        cost_frame = ttk.LabelFrame(scrollable_frame, text="Cost Function Weights")
        cost_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.param_widgets['enable_cost_function_weights'] = CheckFrame(
            cost_frame, "Enable Cost Function Weights", self.current_params.enable_cost_function_weights, self._param_changed
        )
        self.param_widgets['enable_cost_function_weights'].pack(fill=tk.X, padx=5, pady=2)
        
        cost_params = [
            ('steering_weight', "Steering Weight", 0.0, 2.0),
            ('acceleration_weight', "Acceleration Weight", 0.0, 2.0),
            ('jerk_weight', "Jerk Weight", 0.0, 2.0),
            ('heading_weight', "Heading Weight", 0.0, 2.0),
            ('position_weight', "Position Weight", 0.0, 2.0),
            ('velocity_weight', "Velocity Weight", 0.0, 2.0)
        ]
        
        for param_name, display_name, min_val, max_val in cost_params:
            value = getattr(self.current_params, param_name)
            self.param_widgets[param_name] = ParameterFrame(
                cost_frame, display_name, value, min_val, max_val, 0.01, self._param_changed
            )
            self.param_widgets[param_name].pack(fill=tk.X, padx=5, pady=2)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
    
    def create_constraints_tab(self):
        """Create constraints and safety tab"""
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Constraints & Safety")
        
        # Create scrollable frame
        canvas = tk.Canvas(frame)
        scrollbar = ttk.Scrollbar(frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Hard constraints
        hard_frame = ttk.LabelFrame(scrollable_frame, text="Hard Constraints")
        hard_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.param_widgets['enable_hard_constraints'] = CheckFrame(
            hard_frame, "Enable Hard Constraints", self.current_params.enable_hard_constraints, self._param_changed
        )
        self.param_widgets['enable_hard_constraints'].pack(fill=tk.X, padx=5, pady=2)
        
        hard_params = [
            ('hard_max_steering_angle', "Hard Max Steering", 0.1, 1.0),
            ('hard_max_acceleration', "Hard Max Acceleration", 0.1, 3.0),
            ('hard_max_deceleration', "Hard Max Deceleration", 0.1, 3.0)
        ]
        
        for param_name, display_name, min_val, max_val in hard_params:
            value = getattr(self.current_params, param_name)
            self.param_widgets[param_name] = ParameterFrame(
                hard_frame, display_name, value, min_val, max_val, 0.01, self._param_changed
            )
            self.param_widgets[param_name].pack(fill=tk.X, padx=5, pady=2)
        
        # Safety settings
        safety_frame = ttk.LabelFrame(scrollable_frame, text="Safety Settings")
        safety_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.param_widgets['enable_safety_checks'] = CheckFrame(
            safety_frame, "Enable Safety Checks", self.current_params.enable_safety_checks, self._param_changed
        )
        self.param_widgets['enable_safety_checks'].pack(fill=tk.X, padx=5, pady=2)
        
        safety_params = [
            ('safety_check_distance', "Safety Distance", 0.1, 2.0),
            ('safety_timeout', "Safety Timeout", 0.1, 5.0),
            ('emergency_brake_threshold', "Emergency Brake Threshold", 1.0, 5.0)
        ]
        
        for param_name, display_name, min_val, max_val in safety_params:
            value = getattr(self.current_params, param_name)
            self.param_widgets[param_name] = ParameterFrame(
                safety_frame, display_name, value, min_val, max_val, 0.01, self._param_changed
            )
            self.param_widgets[param_name].pack(fill=tk.X, padx=5, pady=2)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
    
    def create_advanced_tab(self):
        """Create advanced features tab"""
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Advanced Features")
        
        # Create scrollable frame
        canvas = tk.Canvas(frame)
        scrollbar = ttk.Scrollbar(frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Advanced features
        features_frame = ttk.LabelFrame(scrollable_frame, text="Advanced Features")
        features_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Boolean features
        bool_features = [
            ('enable_obstacle_avoidance', "Enable Obstacle Avoidance"),
            ('enable_speed_control', "Enable Speed Control"),
            ('enable_trajectory_tracking', "Enable Trajectory Tracking")
        ]
        
        for param_name, display_name in bool_features:
            value = getattr(self.current_params, param_name)
            self.param_widgets[param_name] = CheckFrame(
                features_frame, display_name, value, self._param_changed
            )
            self.param_widgets[param_name].pack(fill=tk.X, padx=5, pady=2)
        
        # Weight parameters
        weight_params = [
            ('obstacle_avoidance_weight', "Obstacle Avoidance Weight", 0.0, 3.0),
            ('speed_control_weight', "Speed Control Weight", 0.0, 2.0),
            ('trajectory_tracking_weight', "Trajectory Tracking Weight", 0.0, 2.0)
        ]
        
        for param_name, display_name, min_val, max_val in weight_params:
            value = getattr(self.current_params, param_name)
            self.param_widgets[param_name] = ParameterFrame(
                features_frame, display_name, value, min_val, max_val, 0.01, self._param_changed
            )
            self.param_widgets[param_name].pack(fill=tk.X, padx=5, pady=2)
        
        # MPC Settings
        mpc_frame = ttk.LabelFrame(scrollable_frame, text="MPC Settings")
        mpc_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.param_widgets['mpc_type'] = ComboFrame(
            mpc_frame, "MPC Type", self.current_params.mpc_type, ["kinematic", "dynamic"], self._param_changed
        )
        self.param_widgets['mpc_type'].pack(fill=tk.X, padx=5, pady=2)
        
        self.param_widgets['solver_type'] = ComboFrame(
            mpc_frame, "Solver Type", self.current_params.solver_type, ["ipopt", "sqpmethod"], self._param_changed
        )
        self.param_widgets['solver_type'].pack(fill=tk.X, padx=5, pady=2)
        
        self.param_widgets['control_hz'] = ParameterFrame(
            mpc_frame, "Control Hz", self.current_params.control_hz, 5.0, 50.0, 1.0, self._param_changed
        )
        self.param_widgets['control_hz'].pack(fill=tk.X, padx=5, pady=2)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
    
    def create_status_tab(self):
        """Create status monitoring tab"""
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Status")
        
        # Performance status
        perf_frame = ttk.LabelFrame(frame, text="Performance Metrics")
        perf_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.status_labels = {}
        status_items = [
            ("Solve Time", "solve_time", "ms"),
            ("Success Rate", "success_rate", "%"),
            ("Real-time Factor", "real_time_factor", "x"),
            ("Total Iterations", "total_iterations", "")
        ]
        
        for i, (name, key, unit) in enumerate(status_items):
            frame_item = ttk.Frame(perf_frame)
            frame_item.pack(fill=tk.X, padx=5, pady=2)
            
            ttk.Label(frame_item, text=f"{name}:", width=20).pack(side=tk.LEFT)
            label = ttk.Label(frame_item, text="--", foreground="blue")
            label.pack(side=tk.LEFT, padx=10)
            self.status_labels[key] = (label, unit)
        
        # Control status
        control_frame = ttk.LabelFrame(frame, text="Current Control")
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        control_items = [
            ("Current Speed", "speed", "m/s"),
            ("Steering Angle", "steering", "rad"),
            ("Acceleration", "acceleration", "m/s²")
        ]
        
        for i, (name, key, unit) in enumerate(control_items):
            frame_item = ttk.Frame(control_frame)
            frame_item.pack(fill=tk.X, padx=5, pady=2)
            
            ttk.Label(frame_item, text=f"{name}:", width=20).pack(side=tk.LEFT)
            label = ttk.Label(frame_item, text="--", foreground="green")
            label.pack(side=tk.LEFT, padx=10)
            self.status_labels[key] = (label, unit)
        
        # ROS2 connection status
        ros_frame = ttk.LabelFrame(frame, text="ROS2 Connection")
        ros_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.ros_status_label = ttk.Label(ros_frame, text="Checking...", foreground="orange")
        self.ros_status_label.pack(padx=5, pady=5)
    
    def create_control_frame(self):
        """Create control buttons frame"""
        control_frame = ttk.Frame(self.root)
        control_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Apply button
        apply_btn = ttk.Button(control_frame, text="Apply Parameters", command=self.apply_parameters)
        apply_btn.pack(side=tk.LEFT, padx=5)
        
        # Emergency stop button
        emergency_btn = ttk.Button(control_frame, text="EMERGENCY STOP", command=self.emergency_stop)
        emergency_btn.pack(side=tk.LEFT, padx=5)
        
        # Auto-apply checkbox
        auto_apply_check = ttk.Checkbutton(control_frame, text="Auto-apply (500ms delay)", variable=self.auto_apply_var)
        auto_apply_check.pack(side=tk.LEFT, padx=20)
        
        # Status
        self.status_var = tk.StringVar(value="Ready")
        status_label = ttk.Label(control_frame, textvariable=self.status_var)
        status_label.pack(side=tk.RIGHT, padx=5)
    
    def _param_changed(self, param_name, value):
        """Handle parameter changes"""
        # Update current parameters
        if param_name == 'horizon_N':
            self.current_params.horizon_N = int(value)
        elif hasattr(self.current_params, param_name):
            setattr(self.current_params, param_name, value)
        
        # Auto-apply if enabled
        if self.auto_apply_var.get():
            if self.apply_timer:
                self.root.after_cancel(self.apply_timer)
            self.apply_timer = self.root.after(500, self.apply_parameters)
    
    def apply_parameters(self):
        """Apply current parameters to MPC controller"""
        success = self.ros_node.update_parameters(self.current_params)
        
        if success:
            self.status_var.set("Parameters applied successfully")
            self.root.after(2000, lambda: self.status_var.set("Ready"))
        else:
            self.status_var.set("Failed to apply parameters")
            messagebox.showerror("Error", "Failed to apply parameters to MPC controller")
            self.root.after(3000, lambda: self.status_var.set("Ready"))
    
    def emergency_stop(self):
        """Trigger emergency stop"""
        result = messagebox.askyesno("Emergency Stop", 
                                   "This will set max speed to 0.1 and apply safety parameters. Continue?")
        
        if result:
            # Set safety parameters
            self.param_widgets['max_speed'].set_value(0.1)
            self.param_widgets['max_acceleration'].set_value(0.1)
            self.param_widgets['enable_safety_checks'].set_value(True)
            self.param_widgets['safety_timeout'].set_value(0.5)
            
            # Apply immediately
            self.apply_parameters()
            
            self.status_var.set("EMERGENCY STOP ACTIVATED")
    
    def save_config(self):
        """Save current configuration to YAML file"""
        filename = filedialog.asksaveasfilename(
            title="Save Configuration",
            defaultextension=".yaml",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")]
        )
        
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
                
                self.status_var.set(f"Configuration saved")
                self.root.after(3000, lambda: self.status_var.set("Ready"))
                
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save configuration: {e}")
    
    def load_config(self):
        """Load configuration from YAML file"""
        filename = filedialog.askopenfilename(
            title="Load Configuration",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")]
        )
        
        if filename:
            self.load_config_file(filename)
    
    def load_config_file(self, filename: str):
        """Load configuration from specific file"""
        try:
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
            
            self.status_var.set(f"Configuration loaded")
            self.root.after(3000, lambda: self.status_var.set("Ready"))
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load configuration: {e}")
    
    def update_current_params(self):
        """Update current parameters from widgets"""
        for param_name, widget in self.param_widgets.items():
            if hasattr(self.current_params, param_name):
                setattr(self.current_params, param_name, widget.get_value())
        
        # Update horizon_N
        self.current_params.horizon_N = self.horizon_n_var.get()
    
    def update_widgets_from_params(self):
        """Update GUI widgets from current parameters"""
        for param_name, widget in self.param_widgets.items():
            if hasattr(self.current_params, param_name):
                value = getattr(self.current_params, param_name)
                widget.set_value(value)
        
        # Update horizon_N
        self.horizon_n_var.set(self.current_params.horizon_N)
    
    def load_preset(self, preset_name: str):
        """Load predefined parameter presets"""
        BASE_CONFIG_DIR = os.getenv('BASE_CONFIG_DIR', './config')
        config_files = {
            'conservative': os.path.join(BASE_CONFIG_DIR, 'params_conservative.yaml'),
            'balanced': os.path.join(BASE_CONFIG_DIR, 'params.yaml'),
            'aggressive': os.path.join(BASE_CONFIG_DIR, 'params_aggressive.yaml'),
            'precision': os.path.join(BASE_CONFIG_DIR, 'params_precision.yaml')
        }
        
        if preset_name in config_files:
            filename = config_files[preset_name]
            if os.path.exists(filename):
                self.load_config_file(filename)
                self.status_var.set(f"Loaded {preset_name} preset")
                self.root.after(2000, lambda: self.status_var.set("Ready"))
            else:
                messagebox.showerror("Error", f"Preset file not found: {filename}")
        else:
            messagebox.showerror("Error", f"Unknown preset: {preset_name}")
    
    def update_status(self):
        """Update status information"""
        try:
            # Update performance status
            perf_data = self.ros_node.latest_performance
            control_data = self.ros_node.latest_control
            state_data = self.ros_node.latest_state
            
            # Update performance status
            if 'avg_solve_time' in perf_data:
                self.status_labels['solve_time'][0].config(text=f"{perf_data['avg_solve_time']*1000:.1f}")
            if 'success_rate' in perf_data:
                self.status_labels['success_rate'][0].config(text=f"{perf_data['success_rate']*100:.1f}")
            if 'real_time_factor' in perf_data:
                self.status_labels['real_time_factor'][0].config(text=f"{perf_data['real_time_factor']:.1f}")
            if 'total_iterations' in perf_data:
                self.status_labels['total_iterations'][0].config(text=f"{perf_data['total_iterations']}")
            
            # Update control status
            if 'speed' in control_data:
                self.status_labels['speed'][0].config(text=f"{control_data['speed']:.2f}")
            if 'steering' in control_data:
                self.status_labels['steering'][0].config(text=f"{control_data['steering']:.3f}")
            if 'acceleration' in control_data:
                self.status_labels['acceleration'][0].config(text=f"{control_data['acceleration']:.2f}")
            
            # Update state status
            if 'velocity' in state_data:
                self.status_labels['speed'][0].config(text=f"{state_data['velocity']:.2f}")
            
            # Update ROS2 status
            if perf_data or control_data or state_data:
                self.ros_status_label.config(text="Connected ✓", foreground="green")
            else:
                self.ros_status_label.config(text="Waiting for data...", foreground="orange")
                
        except Exception as e:
            # Silently handle errors
            pass
        
        # Schedule next update
        self.root.after(500, self.update_status)
    
    def run(self):
        """Start the GUI"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()
    
    def on_closing(self):
        """Handle application close"""
        self.ros_node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()


def main():
    """Main application entry point"""
    try:
        app = MPCTuningGUI()
        app.run()
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
