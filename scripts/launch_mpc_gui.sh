#!/usr/bin/env bash

# F1TENTH MPC Tuning GUI Launcher
# This script handles dependency installation and launches the GUI

set -e

echo "🏎️  F1TENTH MPC Tuning GUI Launcher"
echo "======================================"

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"

# Function to check if a Python package is installed
check_python_package() {
    python3 -c "import $1" 2>/dev/null
    return $?
}

# Check dependencies
echo "📦 Checking dependencies..."

MISSING_DEPS=()

if ! check_python_package PyQt5; then
    MISSING_DEPS+=("PyQt5")
fi

if ! check_python_package pyqtgraph; then
    MISSING_DEPS+=("pyqtgraph")
fi

if ! check_python_package yaml; then
    MISSING_DEPS+=("pyyaml")
fi

# Install missing dependencies
if [ ${#MISSING_DEPS[@]} -ne 0 ]; then
    echo "⚠️  Missing dependencies: ${MISSING_DEPS[*]}"
    echo "📥 Installing dependencies..."
    
    if command -v pip3 &> /dev/null; then
        pip3 install --user "${MISSING_DEPS[@]}"
    elif command -v pip &> /dev/null; then
        pip install --user "${MISSING_DEPS[@]}"
    else
        echo "❌ Error: pip not found. Please install Python package manager."
        exit 1
    fi
    
    echo "✅ Dependencies installed successfully!"
else
    echo "✅ All dependencies are already installed."
fi

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS2 not sourced. Attempting to source..."
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo "✅ ROS2 Humble sourced"
    elif [ -f "/opt/ros/foxy/setup.bash" ]; then
        source /opt/ros/foxy/setup.bash
        echo "✅ ROS2 Foxy sourced"
    else
        echo "❌ Could not find ROS2 installation. Please source ROS2 manually."
        exit 1
    fi
fi

# Source the workspace if it exists
if [ -f "$PACKAGE_DIR/../../../install/setup.bash" ]; then
    source "$PACKAGE_DIR/../../../install/setup.bash"
    echo "✅ Workspace sourced"
fi

# Check if MPC node is available
echo "🔍 Checking MPC controller node..."
if ros2 node list 2>/dev/null | grep -q "optimized_mpc_node"; then
    echo "✅ MPC controller node is running"
elif ros2 pkg list | grep -q "mpc_controller"; then
    echo "⚠️  MPC controller package found but node not running"
    echo "💡 You can start it with:"
    echo "   ros2 launch mpc_controller mpc_controller.launch.py"
else
    echo "⚠️  MPC controller package not found"
    echo "💡 Make sure the workspace is built with: colcon build --packages-select mpc_controller"
fi

# Launch the GUI
echo ""
echo "🚀 Launching MPC Tuning GUI..."
echo "   Press Ctrl+C to exit"
echo ""

cd "$SCRIPT_DIR"
python3 mpc_tuning_gui.py
