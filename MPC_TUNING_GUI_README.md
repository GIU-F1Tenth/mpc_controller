# F1TENTH MPC Real-Time Tuning GUI

A comprehensive real-time parameter tuning system for the F1TENTH MPC controller. This toolkit provides two GUI options for live parameter adjustment during testing and racing.

## 🎯 Features

### Real-Time Parameter Tuning
- **Live parameter updates** via ROS2 parameter server
- **Auto-apply mode** with configurable delay (500ms default)
- **Manual apply** for precise control
- **Emergency stop** functionality

### Parameter Categories
- **Vehicle Parameters**: Wheelbase, limits (speed, acceleration, steering)
- **MPC Horizon**: Prediction horizon (N), time horizon (T), lookahead distance
- **Cost Function Weights**: Steering, acceleration, jerk, heading, position, velocity
- **Hard Constraints**: Stricter safety limits
- **Advanced Features**: Obstacle avoidance, speed control, trajectory tracking
- **Safety Settings**: Timeouts, emergency thresholds, safety distances

### Configuration Management
- **Save/Load YAML configs** compatible with your existing parameter files
- **Parameter presets**: Conservative, Balanced, Aggressive, Precision
- **Keyboard shortcuts** for quick operations

### Real-Time Monitoring
- **Performance metrics**: Solve times, success rates, real-time factors
- **Control status**: Current steering, acceleration, velocity
- **ROS2 connection status**
- **Live plots** (PyQt5 version only)

## 📁 Files Overview

```
scripts/
├── mpc_tuning_gui.py              # Full-featured PyQt5 GUI with plots
├── mpc_tuning_gui_simple.py       # Lightweight tkinter GUI
├── launch_mpc_gui.sh              # Launcher script with dependency management
├── mpc_visualizer.py              # Real-time performance visualizer
└── requirements.txt               # Python dependencies
```

## 🚀 Quick Start

### Option 1: Use the Launcher (Recommended)
```bash
# Navigate to the scripts directory
cd /mpc_controller/scripts

# Run the launcher (handles dependencies automatically)
./launch_mpc_gui.sh
```

### Option 2: Install Dependencies and Run Directly

#### For PyQt5 GUI (Full-featured with plots):
```bash
# Install dependencies
pip3 install --user PyQt5 pyqtgraph numpy pyyaml

# Run the GUI
python3 mpc_tuning_gui.py
```

#### For Simple Tkinter GUI (Lightweight):
```bash
# Only requires standard Python libraries + PyYAML
pip3 install --user pyyaml

# Run the simple GUI
python3 mpc_tuning_gui_simple.py
```

## 🎮 GUI Overview

### PyQt5 GUI (mpc_tuning_gui.py)
**Best for**: Comprehensive tuning with real-time plotting

**Features**:
- Tabbed interface with organized parameter groups
- Real-time performance plots (solve times, success rates)
- Control monitoring plots (steering, velocity)
- Advanced parameter validation
- Professional appearance

### Tkinter GUI (mpc_tuning_gui_simple.py)
**Best for**: Quick parameter adjustments, lower system requirements

**Features**:
- Lightweight, runs on any Python installation
- All parameter adjustment capabilities
- Real-time status monitoring
- Fast startup and low resource usage

### Both GUIs Include:
- **Emergency Stop**: Immediately applies safe parameters
- **Auto-apply**: Automatically applies changes after 500ms
- **Save/Load**: Full YAML configuration management
- **Presets**: Quick access to predefined parameter sets
- **Keyboard shortcuts**: Ctrl+S (save), Ctrl+O (load), Ctrl+A (apply), Ctrl+E (emergency)

## 📊 Parameter Categories Explained

### 1. Vehicle & Horizon Parameters
```yaml
wheelbase: 0.33              # Distance between axles (m)
horizon_N: 12                # Number of prediction steps
horizon_T: 1.2               # Total prediction time (s)
lookahead_distance: 0.8      # Trajectory lookahead (m)
```

### 2. Vehicle Limits
```yaml
max_steering_angle: 0.45     # Maximum steering (rad)
max_acceleration: 1.5        # Maximum acceleration (m/s²)
max_deceleration: 1.8        # Maximum deceleration (m/s²)
min_speed: 0.15              # Minimum speed (m/s)
max_speed: 6.0               # Maximum speed (m/s)
```

### 3. Cost Function Weights
```yaml
cost_function_weights:
  steering_weight: 0.08      # Penalty for steering effort
  acceleration_weight: 0.12  # Penalty for acceleration effort
  jerk_weight: 0.25          # Penalty for smooth motion
  heading_weight: 0.18       # Penalty for heading error
  position_weight: 0.25      # Penalty for position error
  velocity_weight: 0.08      # Penalty for velocity error
```

### 4. Hard Constraints (Safety Limits)
```yaml
hard_constraints:
  max_steering_angle: 0.4    # Stricter steering limit
  max_acceleration: 1.3      # Stricter acceleration limit
  max_deceleration: 1.6      # Stricter deceleration limit
```

## 🎛️ Usage Workflow

### 1. Start the MPC Controller
```bash
# Source your workspace
source /home/mohammedazab/ws/install/setup.bash

# Launch the MPC controller
ros2 launch mpc_controller mpc_controller.launch.py config:=params_precision
```

### 2. Launch the Tuning GUI
```bash
# Use the launcher for automatic setup
./launch_mpc_gui.sh

# Or run directly
python3 mpc_tuning_gui_simple.py
```

### 3. Tuning Process
1. **Load a preset** or existing configuration
2. **Adjust parameters** using sliders/spinboxes
3. **Monitor real-time performance** in the status tab
4. **Fine-tune weights** based on vehicle behavior
5. **Save configurations** for different scenarios

### 4. Parameter Presets

#### Conservative (Safe, Smooth)
- Lower speed limits
- Higher safety margins
- Smooth control actions
- Good for initial testing

#### Balanced (Default)
- Moderate performance
- Balanced safety/speed
- General purpose tuning

#### Aggressive (Racing)
- Higher speed limits
- Tighter safety margins
- Responsive control
- For experienced drivers

#### Precision (Accuracy)
- Optimized for track precision
- Higher cost function weights
- Moderate speeds
- Best trajectory following

## 🔧 Advanced Usage

### Parameter Update Mechanism
The GUI uses ROS2's parameter service to update the MPC controller in real-time:
```python
# Parameters are sent to the running MPC node
/optimized_mpc_node/set_parameters
```

### Emergency Stop Feature
Immediately applies safe parameters:
- Sets max_speed to 0.1 m/s
- Reduces max_acceleration to 0.1 m/s²
- Enables all safety checks
- Reduces safety timeout to 0.5s

### Configuration File Format
Saved configurations are fully compatible with ROS2 launch files:
```yaml
optimized_mpc_controller:
  ros__parameters:
    # All parameters are saved in standard ROS2 format
    wheelbase: 0.33
    horizon_N: 12
    # ... (all other parameters)
```

## 🎯 Tuning Tips

### For Better Trajectory Tracking:
- Increase `position_weight` and `heading_weight`
- Reduce `steering_weight` slightly
- Increase `horizon_N` for better lookahead

### For Smoother Control:
- Increase `jerk_weight`
- Increase `steering_weight` and `acceleration_weight`
- Use longer horizon (`horizon_T`)

### For Faster Response:
- Decrease cost function weights
- Reduce horizon length
- Increase `control_hz`

### For Safety:
- Enable hard constraints
- Increase `safety_check_distance`
- Reduce speed limits
- Lower `safety_timeout`

## 🔍 Troubleshooting

### GUI Won't Start
```bash
# Check dependencies
python3 -c "import tkinter; import yaml; print('Dependencies OK')"

# Check ROS2 environment
echo $ROS_DISTRO

# Source workspace
source /home/mohammedazab/ws/install/setup.bash
```

### Parameters Not Applying
1. **Check MPC node is running**: `ros2 node list | grep mpc`
2. **Verify parameter service**: `ros2 service list | grep set_parameters`
3. **Check ROS2 connection**: Look at the status tab in GUI

### Performance Issues
1. **Reduce update frequency**: Disable auto-apply
2. **Use simple GUI**: Switch to tkinter version
3. **Close unused tabs**: Focus on essential parameters

## 📈 Real-Time Visualizer

For additional performance monitoring, use the included visualizer:
```bash
python3 mpc_visualizer.py
```

Features:
- Real-time acceleration plots (commanded vs actual)
- Velocity tracking visualization
- Steering command monitoring
- Performance statistics logging

## 🤝 Integration with Your Workflow

### Simulation Testing
1. Start your F1TENTH simulator
2. Launch MPC controller with default config
3. Open tuning GUI
4. Adjust parameters based on simulation performance

### Physical Vehicle Testing
1. Ensure safety systems are active
2. Start with conservative preset
3. Gradually tune parameters
4. Always test emergency stop before runs

### Competition Preparation
1. Tune for specific track layouts
2. Save track-specific configurations
3. Practice parameter switching between runs
4. Verify all safety systems

## 🏁 Conclusion

This tuning system provides comprehensive real-time control over your F1TENTH MPC controller parameters. Whether you're doing initial setup, track-specific tuning, or competition preparation, these tools will help you achieve optimal performance while maintaining safety.

The combination of real-time parameter updates, comprehensive monitoring, and intuitive interfaces makes this an essential tool for F1TENTH development and racing.

Happy tuning! 🏎️💨
