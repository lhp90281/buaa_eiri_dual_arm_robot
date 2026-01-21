#!/bin/bash

# Quick test script for Topic Bridge integration
# This script helps you test the ros2_control integration step by step

set -e

echo "=========================================="
echo "Topic Bridge Integration Test"
echo "=========================================="
echo ""

# Step 1: Build
echo "Step 1: Building eiriarm_controllers..."
cd ~/ros2_ws
colcon build --packages-select eiriarm_controllers
source install/setup.bash
echo "✓ Build complete"
echo ""

# Step 2: Verify plugins
echo "Step 2: Verifying plugins..."
echo ""

# Check if plugin XML files are installed
echo "Checking plugin description files..."
HW_PLUGIN_XML="$HOME/ros2_ws/install/eiriarm_controllers/share/eiriarm_controllers/eiriarm_hardware_interface.xml"
CTRL_PLUGIN_XML="$HOME/ros2_ws/install/eiriarm_controllers/share/eiriarm_controllers/eiriarm_controllers.xml"

if [ -f "$HW_PLUGIN_XML" ]; then
    echo "✓ Hardware Interface plugin XML found"
    echo "  Content:"
    grep "class name" "$HW_PLUGIN_XML" | head -1
else
    echo "✗ Hardware Interface plugin XML NOT found at: $HW_PLUGIN_XML"
    exit 1
fi

if [ -f "$CTRL_PLUGIN_XML" ]; then
    echo "✓ Controller plugin XML found"
    echo "  Content:"
    grep "class name" "$CTRL_PLUGIN_XML" | head -2
else
    echo "✗ Controller plugin XML NOT found at: $CTRL_PLUGIN_XML"
    exit 1
fi

echo ""

# Check if shared libraries are built
echo "Checking plugin libraries..."
HW_LIB="$HOME/ros2_ws/install/eiriarm_controllers/lib/libtopic_based_hardware_interface.so"
CTRL_LIB="$HOME/ros2_ws/install/eiriarm_controllers/lib/libjoint_impedance_controller_plugin.so"

if [ -f "$HW_LIB" ]; then
    echo "✓ Hardware Interface library found"
else
    echo "✗ Hardware Interface library NOT found at: $HW_LIB"
    exit 1
fi

if [ -f "$CTRL_LIB" ]; then
    echo "✓ Controller plugin library found"
else
    echo "✗ Controller plugin library NOT found at: $CTRL_LIB"
    exit 1
fi

echo ""
echo "=========================================="
echo "All checks passed! ✓"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Start MuJoCo simulator:"
echo "   ros2 run eiriarm_mujoco simulate"
echo ""
echo "2. In another terminal, start controller_manager:"
echo "   ros2 run controller_manager ros2_control_node \\"
echo "     --ros-args \\"
echo "     --params-file src/eiriarm_controllers/config/ros2_control_controllers.yaml \\"
echo "     -p robot_description:=\"\$(cat src/description/dual_arm_support/urdf/dual_arm_robot.urdf)\""
echo ""
echo "3. Load and activate controllers:"
echo "   ros2 control load_controller joint_state_broadcaster"
echo "   ros2 control set_controller_state joint_state_broadcaster active"
echo "   ros2 control load_controller joint_impedance_controller"
echo "   ros2 control set_controller_state joint_impedance_controller active"
echo ""
echo "4. Send target commands:"
echo "   ros2 run eiriarm_controllers send_target_command.py"
echo ""
