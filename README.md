# MPC Controller for F1Tenth Autonomous Vehicle

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)

## 📌 Overview

This repository implements a **Model Predictive Controller (MPC)** for F1Tenth autonomous vehicles using **ROS 2**. The controller optimizes acceleration and steering commands to follow reference trajectories while respecting vehicle dynamics and constraints.

### Key Features

- **MPC implementation** using CasADi optimization
- **ROS 2 integration** with configurable parameters
- **Kinematic bicycle model** for vehicle dynamics
- **Trajectory optimization** tools and utilities
- **Comprehensive testing** and documentation
- **Type hints** and modern Python standards compliance

## 🏗️ Architecture

```
mpc_controller/
├── mpc_controller/           # Main package directory
│   ├── __init__.py
│   ├── MPC_Controller.py     # Core MPC implementation
│   ├── MPCtrlNode.py         # ROS 2 node
│   ├── config.py             # Configuration classes
│   ├── utils.py              # Utility functions
│   └── optimize_trajectoryMPC.py  # Trajectory optimization
├── launch/                   # Launch files
│   └── mpc_controller.launch.py
├── test/                     # Unit tests
│   ├── test_mpc_controller.py
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── resource/                 # Resources and images
├── requirements.txt          # Python dependencies
├── package.xml              # ROS 2 package manifest
├── setup.py                # Package setup
├── LICENSE                 # MIT license
└── README.md              # This file
```

## � Installation

### Prerequisites

- **ROS 2 Humble** or later
- **Python 3.8+**
- **CasADi** optimization library

### Clone and Install

```bash
# Clone the repository
cd ~/ros2_ws/src
git clone <repository-url> mpc_controller

# Install Python dependencies
cd mpc_controller
pip install -r requirements.txt

# Install ROS 2 dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select mpc_controller

# Source the workspace
source install/setup.bash
```

### System Dependencies

```bash
# Install CasADi
pip install casadi

# Install ROS 2 dependencies
sudo apt install ros-humble-ackermann-msgs
sudo apt install ros-humble-nav-msgs
```

## 🚀 Usage

### Launch the MPC Controller

```bash
# Launch with default parameters
ros2 launch mpc_controller mpc_controller.launch.py

# Launch with custom parameters
ros2 launch mpc_controller mpc_controller.launch.py \
    wheelbase:=0.33 \
    mpc_horizon:=15 \
    max_velocity:=3.0
```

### Run Trajectory Optimization

```bash
# Optimize a trajectory from CSV file
ros2 run mpc_controller trajectory_optimizer

# Or run directly
python3 -m mpc_controller.optimize_trajectoryMPC
```

### Configuration Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `wheelbase` | Vehicle wheelbase (m) | 0.33 |
| `mpc_horizon` | Prediction horizon steps | 10 |
| `mpc_time_horizon` | Time horizon (s) | 1.0 |
| `max_velocity` | Maximum velocity (m/s) | 5.0 |
| `control_frequency` | Control loop frequency (Hz) | 10.0 |

## � Mathematical Model

### Vehicle Dynamics

The controller uses a **kinematic bicycle model**:

![Kinematic Bicycle Model Equation](resource/ModelEquation.png)

### MPC Optimization Problem

The MPC solves the following optimization problem:

![MPC Optimization Problem](resource/costFunction.png)

## 🧪 Testing

Run the test suite:

```bash
# Run all tests
cd ~/ros2_ws
colcon test --packages-select mpc_controller

# Run specific tests
python -m pytest src/mpc_controller/test/test_mpc_controller.py -v

# Run with coverage
python -m pytest src/mpc_controller/test/ --cov=mpc_controller --cov-report=html
```

## 📈 Performance

### Benchmarks

- **Control frequency**: Up to 50 Hz on modern hardware
- **Optimization time**: < 10ms per iteration (typical)
- **Memory usage**: < 100 MB
- **Prediction horizon**: 10-20 steps (recommended)

### Tuning Guidelines

1. **Increase `Q` weights** for better trajectory tracking
2. **Increase `R` weights** for smoother control inputs
3. **Longer horizon** for better predictive behavior (at computational cost)
4. **Higher frequency** for more responsive control

## 🔄 ROS 2 Interface

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Vehicle odometry |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | Control commands |

### Parameters

All parameters are dynamically reconfigurable through ROS 2 parameter server.

## 🛠️ Development

### Code Style

This project follows:
- **PEP 8** Python style guide
- **Type hints** for all functions
- **Docstrings** in Google format
- **Black** code formatting
- **Flake8** linting

### Pre-commit Hooks

```bash
# Install pre-commit
pip install pre-commit

# Set up hooks
pre-commit install

# Run manually
pre-commit run --all-files
```

### Contributing

1. Fork the repository
2. Create a feature branch
3. Make changes with tests
4. Ensure all tests pass
5. Submit a pull request

## 📚 API Reference

### MPCController Class

```python
from mpc_controller.MPC_Controller import MPCController

# Initialize controller
mpc = MPCController(N=10, T=1.0, L=0.33)

# Solve optimization
acceleration, steering = mpc.solve_mpc(x, y, v, theta, reference_trajectory)
```

### Utility Functions

```python
from mpc_controller.utils import (
    generate_circular_trajectory,
    generate_straight_trajectory,
    normalize_angle
)

# Generate reference trajectory
trajectory = generate_circular_trajectory(radius=2.0, velocity=2.0)
```

## � Troubleshooting

### Common Issues

1. **CasADi installation fails**
   ```bash
   pip install --upgrade pip
   pip install casadi --no-cache-dir
   ```

2. **ROS 2 dependencies missing**
   ```bash
   rosdep update
   rosdep install --from-paths . --ignore-src -r -y
   ```

3. **Optimization fails frequently**
   - Check reference trajectory validity
   - Reduce prediction horizon
   - Adjust weight matrices

4. **Poor tracking performance**
   - Increase state weights in Q matrix
   - Reduce control frequency if too high
   - Verify vehicle parameters

## 📋 Changelog

### Version 1.0.0
- Initial release with professional MPC implementation
- ROS 2 integration with configurable parameters
- Comprehensive testing and documentation
- MIT license

## 🤝 Contributing

Contributions are welcome! Please read our contributing guidelines and submit pull requests for any improvements.

## � License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- F1Tenth community for the vehicle platform
- CasADi developers for the optimization library
- ROS 2 team for the robotics framework

## 📞 Support

For questions and support:
- Create an issue on GitHub
- Contact the maintainers
- Check the F1Tenth community forums

---

**Happy autonomous driving! 🏎️**