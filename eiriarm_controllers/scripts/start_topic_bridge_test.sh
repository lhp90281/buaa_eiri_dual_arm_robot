#!/bin/bash

# Quick start script for testing Topic Bridge
# This script helps you start the system step by step

set -e

echo "=========================================="
echo "Topic Bridge Test - Step by Step"
echo "=========================================="
echo ""

# Check if MuJoCo is running
echo "Checking if MuJoCo simulator is running..."
if ros2 topic list | grep -q "/joint_states"; then
    echo "✓ MuJoCo is running (detected /joint_states topic)"
else
    echo "✗ MuJoCo is NOT running!"
    echo ""
    echo "Please start MuJoCo in another terminal:"
    echo "  ros2 run eiriarm_mujoco simulate"
    echo ""
    exit 1
fi

echo ""
echo "Starting Controller Manager with Topic Bridge..."
echo ""

# Use launch file to properly load URDF
ros2 launch eiriarm_controllers test_topic_bridge.launch.py
