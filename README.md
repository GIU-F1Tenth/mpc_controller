
# F1TENTH MPC Controller with Real-Time Tuning GUI

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)

An advanced Model Predictive Controller (MPC) package for F1TENTH autonomous racing platforms with real-time parameter tuning capabilities. Built on ROS 2 and CasADi, this package provides optimal control for high-speed racing with live parameter adjustment through intuitive GUI interfaces.

---

## ✨ Key Features

### 🏎️ **Advanced MPC Controller**
- **Dual Vehicle Models**: Kinematic & Dynamic bicycle models
- **Real-Time Optimization** using CasADi with IPOPT/SQPMethod solvers
- **Optimized Performance** for F1TENTH racing speeds
- **Configurable Cost Functions** with position, heading, velocity, and control effort weights
- **Safety Constraints** with hard limits and emergency stop functionality

### 🎛️ **Real-Time Tuning GUI**
- **PyQt5 Professional Interface** with tabbed parameter controls
- **Live Parameter Updates** via ROS2 parameter server (no restart required)
- **Real-Time Monitoring** with performance plots and solver metrics
- **Configuration Management** with YAML save/load and presets
- **Emergency Controls** with instant safety parameter activation

### 📊 **Monitoring & Visualization**
- **Performance Plots**: Solve time, success rate, real-time factor
- **Control Monitoring**: Steering commands, velocity tracking
- **Parameter Presets**: Conservative, Balanced, Aggressive, Precision modes
- **Status Dashboard** with live solver and vehicle state information

---

## 🛠 System Requirements

- **ROS 2 Humble** or later
- **Python 3.8+** with virtual environment support
- **Ubuntu 20.04/22.04** (tested in dev containers)
- **Dependencies**: `casadi`, `numpy`, `matplotlib`, `PyQt5`, `pyqtgraph`
- **ROS Packages**: `ackermann_msgs`, `nav_msgs`, `diagnostic_msgs`

---

## 📦 Installation

### 1. Clone the Repository
```bash
cd ~/ros2_ws/src
git clone https://github.com/GIU-F1Tenth/mpc_controller.git
cd mpc_controller
```

### 2. Setup Python Environment
```bash
# Create virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install Python dependencies
pip install -r requirements.txt
```

### 3. Install ROS Dependencies
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the Package
```bash
colcon build --packages-select mpc_controller
source install/setup.bash
```

---

## 🚀 Quick Start

### 🔹 Launch MPC Controller
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch MPC controller
ros2 launch mpc_controller mpc_controller.launch.py
```

### 🔹 Launch Real-Time Tuning GUI
```bash
# Navigate to package directory
cd ~/ros2_ws/src/mpc_controller

# Auto-launch with dependency management
./scripts/launch_mpc_gui.sh

# Or manually with virtual environment
source .venv/bin/activate
python scripts/mpc_tuning_gui.py
```

### 🔹 Alternative Lightweight GUI
```bash
# Tkinter-based GUI (no PyQt5 dependency)
python scripts/mpc_tuning_gui_simple.py
```

---

## ⚙️ Configuration

### 📋 **Parameter Categories**

#### **Vehicle Parameters**
- `wheelbase`: Distance between front and rear axles (0.33m)
- `max_steering_angle`: Maximum steering angle in radians
- `max_speed`: Vehicle speed limits for safety

#### **MPC Horizon**
- `horizon_N`: Prediction horizon steps (3-50)
- `horizon_T`: Total prediction time (0.1-3.0s)
- `lookahead_distance`: Trajectory tracking lookahead

#### **Cost Function Weights**
- `position_weight`: Penalty for position error
- `heading_weight`: Penalty for heading deviation
- `velocity_weight`: Penalty for speed error
- `steering_weight`: Control effort penalty
- `acceleration_weight`: Acceleration smoothness
- `jerk_weight`: Acceleration change penalty

#### **Safety & Constraints**
- `enable_hard_constraints`: Strict limit enforcement
- `enable_safety_checks`: Safety monitoring systems
- `emergency_brake_threshold`: Emergency stop trigger

### 📁 **Configuration Files**
```
config/
├── params.yaml              # Balanced settings
├── params_conservative.yaml # Safe, smooth driving
├── params_aggressive.yaml   # High-performance racing
├── params_precision.yaml    # Accurate trajectory following
└── params_tuning_*.yaml     # Experimental configurations
```

---

## 📖 MPC Implementation

### 🔸 **Vehicle Models**

#### Kinematic Bicycle Model
```
x_{k+1} = x_k + v_k * cos(θ_k) * dt
y_{k+1} = y_k + v_k * sin(θ_k) * dt  
θ_{k+1} = θ_k + (v_k / L) * tan(δ_k) * dt
v_{k+1} = v_k + a_k * dt
```

#### Dynamic Bicycle Model (Advanced)
- Includes tire dynamics and slip effects
- Higher fidelity for racing applications
- Configurable via `mpc_type: "dynamic"`

### 🔸 **Cost Function**
```
J = Σ(Q_pos * ||pos_error||² + Q_heading * ||heading_error||² + 
      Q_vel * ||vel_error||² + R_steering * ||δ||² + R_accel * ||a||²)
```

### 🔸 **Constraints**
- **Input Constraints**: `|δ| ≤ δ_max`, `|a| ≤ a_max`
- **State Constraints**: `v_min ≤ v ≤ v_max`
- **Safety Constraints**: Collision avoidance, track boundaries

---

## 📂 Project Structure

```
mpc_controller/
├── launch/
│   └── mpc_controller.launch.py     # ROS2 launch configuration
├── mpc_controller/                  # Core MPC implementation
│   ├── mpc_node.py                 # Main ROS2 node
│   ├── optimized_mpc_controller.py # Optimized MPC solver
│   ├── kinematic_bicycle_model.py  # Kinematic vehicle model
│   └── dynamic_bicycle_model.py    # Dynamic vehicle model
├── scripts/                        # GUI and utility scripts
│   ├── mpc_tuning_gui.py          # PyQt5 real-time tuning GUI
│   ├── mpc_tuning_gui_simple.py   # Lightweight tkinter GUI
│   ├── mpc_visualizer.py          # Real-time data visualization
│   ├── launch_mpc_gui.sh           # Automated GUI launcher
│   └── test_parameter_update.py    # Parameter service testing
├── config/                         # Parameter configurations
│   ├── params_*.yaml              # Various tuning presets
│   └── config.py                  # Configuration utilities
├── test/                          # Test implementations
│   ├── test_mpc_node.py           # MPC node testing
│   └── test_model_switching.py    # Model switching tests
├── resource/                      # Documentation assets
├── docs/                          # Additional documentation
├── requirements.txt               # Python dependencies
├── MPC_TUNING_GUI_README.md      # Detailed GUI documentation
└── README.md                      # This file
```

---

## 🎮 GUI Usage Guide

### 🔹 **Parameter Tuning**
1. **Launch Controller**: Start MPC controller first
2. **Open GUI**: Run tuning GUI with `./scripts/launch_mpc_gui.sh`
3. **Adjust Parameters**: Use sliders or input fields for real-time tuning
4. **Monitor Performance**: Watch solver metrics and control plots
5. **Save Configuration**: Export successful parameter sets to YAML

### 🔹 **Real-Time Monitoring**
- **Solve Time**: Monitor optimization performance
- **Success Rate**: Track solver convergence
- **Control Commands**: Visualize steering and acceleration
- **Vehicle State**: Monitor speed and position

### 🔹 **Emergency Controls**
- **Emergency Stop**: Instantly apply safety parameters
- **Parameter Reset**: Quick return to known-good configurations
- **Preset Loading**: Switch between driving modes instantly

---

## 🧪 Testing & Validation

### **Unit Tests**
```bash
# Run all tests
colcon test --packages-select mpc_controller

# Run specific test
python -m pytest test/test_mpc_node.py -v
```

### **Parameter Service Testing**
```bash
# Test parameter update mechanism
python scripts/test_parameter_update.py
```

### **GUI Testing**
```bash
# Test GUI without ROS dependencies
python scripts/mpc_tuning_gui_simple.py --test-mode
```

---

## 🏁 F1TENTH Integration

### **Simulation Setup**
1. Launch F1TENTH simulator
2. Start MPC controller: `ros2 launch mpc_controller mpc_controller.launch.py`
3. Open tuning GUI: `./scripts/launch_mpc_gui.sh`
4. Begin real-time parameter optimization

### **Physical Vehicle**
1. Ensure ROS2 network connectivity
2. Verify sensor data topics are publishing
3. Start with conservative parameters
4. Gradually tune for performance using GUI

### **Racing Optimization**
- Start with `params_conservative.yaml`
- Use GUI to increase aggressiveness safely
- Monitor solver performance for real-time capability
- Save optimal configurations for different tracks

---

## 🔧 Troubleshooting

### **Common Issues**
- **GUI won't start**: Check PyQt5 installation with `pip install PyQt5 pyqtgraph`
- **Parameter updates fail**: Verify MPC controller is running
- **Solver timeouts**: Reduce horizon length or increase solver timeout
- **Display issues**: Use simple GUI with `mpc_tuning_gui_simple.py`

### **Performance Optimization**
- Reduce `horizon_N` for faster solve times
- Use kinematic model for better real-time performance
- Adjust `control_hz` based on computational capability
- Monitor solve time plots for performance tuning

---

## 🔭 Future Development

- **Enhanced Dynamic Models**: Tire slip and advanced vehicle dynamics
- **Multi-Agent Racing**: Overtaking and defensive maneuvers
- **Track-Specific Optimization**: Automatic parameter adaptation
- **Cloud Integration**: Remote tuning and telemetry
- **Machine Learning**: Neural network cost function learning

---

## 🤝 Contributing

1. Fork the repository
2. Create feature branch: `git checkout -b feature/amazing-feature`
3. Commit changes: `git commit -m 'Add amazing feature'`
4. Push to branch: `git push origin feature/amazing-feature`
5. Open Pull Request

---

## 📜 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENCE) file for details.

---

## 👨‍💻 Author & Support

**Mohammed Azab** - [mohammed@azab.io](mailto:mohammed@azab.io)

- 🌐 **GitHub**: [GIU-F1Tenth/mpc_controller](https://github.com/GIU-F1Tenth/mpc_controller)
- � **Documentation**: See `MPC_TUNING_GUI_README.md` for detailed GUI usage
- 🐛 **Issues**: Report bugs via GitHub Issues
- 💬 **Discussions**: Use GitHub Discussions for questions

---

**Ready to race? 🏎️💨 Launch your MPC controller and start tuning for victory!**
