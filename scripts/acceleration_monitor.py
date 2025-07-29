#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import time
import threading
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from collections import deque

class AccelerationMonitorGUI(Node):
    def __init__(self):
        super().__init__('acceleration_monitor_gui')
        
        # Data storage
        self.max_data_points = 100
        self.time_data = deque(maxlen=self.max_data_points)
        self.commanded_accel_data = deque(maxlen=self.max_data_points)
        self.actual_accel_data = deque(maxlen=self.max_data_points)
        self.velocity_data = deque(maxlen=self.max_data_points)
        self.steering_data = deque(maxlen=self.max_data_points)
        self.ref_velocity_data = deque(maxlen=self.max_data_points)
        
        # Subscribe to odometry to track velocity changes
        self.odom_sub = self.create_subscription(
            Odometry,
            '/car_state/odom',
            self.odom_callback,
            10
        )
        
        # Subscribe to control commands to see what MPC is outputting
        self.control_sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.control_callback,
            10
        )
        
        self.prev_velocity = None
        self.prev_time = None
        self.start_time = time.time()
        
        # Statistics
        self.total_commands = 0
        self.accel_stats = {'min': float('inf'), 'max': float('-inf'), 'avg': 0}
        self.velocity_stats = {'min': float('inf'), 'max': float('-inf'), 'avg': 0}
        
        # Create GUI
        self.setup_gui()
        
        # Timer to update GUI
        self.timer = self.create_timer(0.1, self.update_gui)
        
    def setup_gui(self):
        """Setup the GUI interface"""
        self.root = tk.Tk()
        self.root.title("MPC Acceleration Monitor")
        self.root.geometry("1200x800")
        
        # Create main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create statistics frame
        stats_frame = ttk.LabelFrame(main_frame, text="Statistics", padding=10)
        stats_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Statistics labels
        self.stats_labels = {}
        stats_grid = [
            ("Commands Received:", "commands"),
            ("Acceleration Range:", "accel_range"),
            ("Current Velocity:", "velocity"),
            ("Current Acceleration:", "acceleration"),
            ("Steering Angle:", "steering"),
            ("Velocity Error:", "vel_error")
        ]
        
        for i, (label, key) in enumerate(stats_grid):
            row = i // 3
            col = (i % 3) * 2
            ttk.Label(stats_frame, text=label).grid(row=row, column=col, sticky=tk.W, padx=(0, 10))
            self.stats_labels[key] = ttk.Label(stats_frame, text="N/A", foreground="blue")
            self.stats_labels[key].grid(row=row, column=col+1, sticky=tk.W, padx=(0, 20))
        
        # Create plots frame
        plots_frame = ttk.Frame(main_frame)
        plots_frame.pack(fill=tk.BOTH, expand=True)
        
        # Create matplotlib figure
        self.fig = Figure(figsize=(12, 8), dpi=100)
        
        # Create subplots
        self.ax1 = self.fig.add_subplot(3, 1, 1)
        self.ax2 = self.fig.add_subplot(3, 1, 2)
        self.ax3 = self.fig.add_subplot(3, 1, 3)
        
        # Configure subplots
        self.ax1.set_title("Acceleration Commands vs Actual", fontsize=12, fontweight='bold')
        self.ax1.set_ylabel("Acceleration (m/s²)")
        self.ax1.grid(True, alpha=0.3)
        self.ax1.legend(['Commanded', 'Actual'])
        
        self.ax2.set_title("Velocity Tracking", fontsize=12, fontweight='bold')
        self.ax2.set_ylabel("Velocity (m/s)")
        self.ax2.grid(True, alpha=0.3)
        self.ax2.legend(['Current', 'Reference'])
        
        self.ax3.set_title("Steering Commands", fontsize=12, fontweight='bold')
        self.ax3.set_ylabel("Steering (rad)")
        self.ax3.set_xlabel("Time (s)")
        self.ax3.grid(True, alpha=0.3)
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, plots_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Control buttons frame
        controls_frame = ttk.Frame(main_frame)
        controls_frame.pack(fill=tk.X, pady=(10, 0))
        
        # Buttons
        ttk.Button(controls_frame, text="Clear Data", command=self.clear_data).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(controls_frame, text="Save Plot", command=self.save_plot).pack(side=tk.LEFT, padx=(0, 10))
        
        # Status
        self.status_var = tk.StringVar()
        self.status_var.set("Waiting for data...")
        ttk.Label(controls_frame, textvariable=self.status_var).pack(side=tk.RIGHT)
        
    def clear_data(self):
        """Clear all collected data"""
        self.time_data.clear()
        self.commanded_accel_data.clear()
        self.actual_accel_data.clear()
        self.velocity_data.clear()
        self.steering_data.clear()
        self.ref_velocity_data.clear()
        self.total_commands = 0
        self.start_time = time.time()
        self.get_logger().info("Data cleared")
        
    def save_plot(self):
        """Save the current plot"""
        filename = f"mpc_acceleration_monitor_{int(time.time())}.png"
        self.fig.savefig(filename, dpi=300, bbox_inches='tight')
        self.get_logger().info(f"Plot saved as {filename}")
        
    def odom_callback(self, msg):
        current_time = time.time() - self.start_time
        current_velocity = (msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)**0.5
        
        # Calculate actual acceleration
        actual_acceleration = 0.0
        if self.prev_velocity is not None and self.prev_time is not None:
            dt = time.time() - self.prev_time
            if dt > 0:
                actual_acceleration = (current_velocity - self.prev_velocity) / dt
        
        # Store data
        self.time_data.append(current_time)
        self.velocity_data.append(current_velocity)
        self.actual_accel_data.append(actual_acceleration)
        
        # Add placeholder for missing data
        if len(self.commanded_accel_data) < len(self.time_data):
            self.commanded_accel_data.append(0.0)
        if len(self.steering_data) < len(self.time_data):
            self.steering_data.append(0.0)
        if len(self.ref_velocity_data) < len(self.time_data):
            self.ref_velocity_data.append(current_velocity)  # Use current as fallback
        
        self.prev_velocity = current_velocity
        self.prev_time = time.time()
        
        # Update statistics
        self.velocity_stats['min'] = min(self.velocity_stats['min'], current_velocity)
        self.velocity_stats['max'] = max(self.velocity_stats['max'], current_velocity)
        
    def control_callback(self, msg):
        current_time = time.time() - self.start_time
        commanded_accel = msg.drive.acceleration
        commanded_steering = msg.drive.steering_angle
        
        self.total_commands += 1
        
        # Store data - align with time data
        if len(self.time_data) > 0:
            # Update the most recent commanded values
            if len(self.commanded_accel_data) > 0:
                self.commanded_accel_data[-1] = commanded_accel
            else:
                self.commanded_accel_data.append(commanded_accel)
                
            if len(self.steering_data) > 0:
                self.steering_data[-1] = commanded_steering
            else:
                self.steering_data.append(commanded_steering)
        
        # Update acceleration statistics
        self.accel_stats['min'] = min(self.accel_stats['min'], commanded_accel)
        self.accel_stats['max'] = max(self.accel_stats['max'], commanded_accel)
        
    def update_gui(self):
        """Update GUI with latest data"""
        if not self.time_data:
            return
            
        try:
            # Update statistics
            self.stats_labels["commands"].config(text=str(self.total_commands))
            
            if self.commanded_accel_data:
                recent_accels = list(self.commanded_accel_data)[-10:]
                accel_range = max(recent_accels) - min(recent_accels) if recent_accels else 0
                self.stats_labels["accel_range"].config(text=f"{accel_range:.3f}")
                self.stats_labels["acceleration"].config(text=f"{recent_accels[-1]:.3f}" if recent_accels else "N/A")
            
            if self.velocity_data:
                self.stats_labels["velocity"].config(text=f"{self.velocity_data[-1]:.3f}")
            
            if self.steering_data:
                self.stats_labels["steering"].config(text=f"{self.steering_data[-1]:.3f}")
            
            # Calculate velocity error if we have reference data
            if self.velocity_data and self.ref_velocity_data:
                vel_error = abs(self.ref_velocity_data[-1] - self.velocity_data[-1])
                self.stats_labels["vel_error"].config(text=f"{vel_error:.3f}")
            
            # Update plots
            self.update_plots()
            
            # Update status
            data_points = len(self.time_data)
            self.status_var.set(f"Data points: {data_points}, Time: {self.time_data[-1]:.1f}s")
            
        except Exception as e:
            self.get_logger().error(f"GUI update error: {e}")
    
    def update_plots(self):
        """Update the matplotlib plots"""
        if len(self.time_data) < 2:
            return
            
        try:
            # Clear previous plots
            self.ax1.clear()
            self.ax2.clear()
            self.ax3.clear()
            
            time_array = np.array(self.time_data)
            
            # Plot 1: Acceleration comparison
            if self.commanded_accel_data and self.actual_accel_data:
                self.ax1.plot(time_array, self.commanded_accel_data, 'b-', label='Commanded', linewidth=2)
                self.ax1.plot(time_array, self.actual_accel_data, 'r-', label='Actual', linewidth=2, alpha=0.7)
                self.ax1.set_title("Acceleration Commands vs Actual", fontweight='bold')
                self.ax1.set_ylabel("Acceleration (m/s²)")
                self.ax1.grid(True, alpha=0.3)
                self.ax1.legend()
            
            # Plot 2: Velocity tracking
            if self.velocity_data:
                self.ax2.plot(time_array, self.velocity_data, 'g-', label='Current Velocity', linewidth=2)
                if self.ref_velocity_data:
                    self.ax2.plot(time_array, self.ref_velocity_data, 'g--', label='Reference Velocity', 
                                 linewidth=2, alpha=0.7)
                self.ax2.set_title("Velocity Tracking", fontweight='bold')
                self.ax2.set_ylabel("Velocity (m/s)")
                self.ax2.grid(True, alpha=0.3)
                self.ax2.legend()
            
            # Plot 3: Steering commands
            if self.steering_data:
                self.ax3.plot(time_array, self.steering_data, 'm-', linewidth=2)
                self.ax3.set_title("Steering Commands", fontweight='bold')
                self.ax3.set_ylabel("Steering (rad)")
                self.ax3.set_xlabel("Time (s)")
                self.ax3.grid(True, alpha=0.3)
            
            # Adjust layout and redraw
            self.fig.tight_layout()
            self.canvas.draw()
            
        except Exception as e:
            self.get_logger().error(f"Plot update error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    # Create the monitor
    monitor = AccelerationMonitorGUI()
    
    # Run ROS2 in a separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(monitor), daemon=True)
    ros_thread.start()
    
    try:
        # Run the GUI
        monitor.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
