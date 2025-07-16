
# Model Predictive Control (MPC) for ROS 2

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)

An advanced Model Predictive Controller (MPC) package for F1TENTH autonomous racing platforms, built on ROS 2 and CasADi. This package computes optimal control inputs (steering and acceleration) to follow a reference trajectory while respecting vehicle dynamics constraints.

---

## ✨ Features

- **Kinematic Bicycle Model** for accurate vehicle dynamics
- **Real-Time Optimization** using CasADi
- **Ackermann Drive Support** with `ackermann_msgs`
- **Configurable Horizon & Cost Weights**
- **ROS 2 Integration** with publishers/subscribers
- **Launch-ready** via ROS 2 launch files

---

## 🛠 System Requirements

- ROS 2 Humble or later
- Python 3.8+
- `casadi`, `numpy`, `matplotlib`
- `ackermann_msgs` ROS 2 interface

---

## 📦 Installation

### 1. Clone the package into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repo-url> mpc_controller
```

### 2. Install Python and ROS dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip install -r src/mpc_controller/requirements.txt
```

> Or install manually:
```bash
pip install casadi numpy matplotlib
sudo apt install ros-humble-ackermann-msgs
```

### 3. Build the package:
```bash
colcon build --packages-select mpc_controller
source install/setup.bash
```

---

## 🚀 Usage

### 🔹 Launch in Simulation

```bash
ros2 launch mpc_controller mpc_controller.launch.py
```

### 🔹 Run Node Directly

```bash
ros2 run mpc_controller MPC_Node
```

---

## ⚙️ Configuration

MPC parameters (like horizon length, weights, etc.) are configurable via YAML or directly in the code. You can modify:

- Prediction horizon `N`
- Time step `dt`
- Cost function weights
- Vehicle parameters (L, max steering angle, etc.)

---

## 📖 MPC Model

### 🔸 Vehicle Model – Kinematic Bicycle Model

![Bicycle Model](resource/ModelEquation.png)

### 🔸 Cost Function

The MPC minimizes a weighted cost function of position error, heading error, and control effort:

![Cost Function](resource/costFunction.png)

Subject to vehicle dynamics and constraints on steering, acceleration, etc.

---

## 📂 Project Structure

```
mpc_controller/
├── launch/
│   └── mpc_controller.launch.py
├── resource/
│   ├── ModelEquation.png
│   └── costFunction.png
├── src/
│   ├── mpc_controller/
│   │   ├── __init__.py
│   │   ├── MPCtrlNode.py
│   │   ├── controller.py
│   │   └── utils.py
├── test/
├── package.xml
├── setup.py
└── README.md
```

---

## 🧪 Testing

```bash
colcon test --packages-select mpc_controller
```

---

## 🔭 Future Work
- Implement obstacle avoidance
- Improve real-time performance
- Test on physical F1TENTH platform

---

## 📜 License

This project is open-source under the **MIT License**.

---

For questions, contact **Mohammed Azab (Mohammed@azab.io)** 🚀.
