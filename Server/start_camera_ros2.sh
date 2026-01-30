#!/bin/bash
# File name   : start_camera_ros2.sh
# Description : Startup script for camera container - runs both FPV.py and FPV_ROS2_simple.py
# Author      : GitHub Copilot
# Date        : 2026-01-30

echo "=========================================="
echo "Starting Camera ROS2 Container"
echo "=========================================="

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Start FPV.py in background (provides camera frames via ZMQ)
echo "Starting FPV.py (camera capture + ZMQ stream)..."
cd /ros2_ws/Server
python3 FPV.py &
FPV_PID=$!

# Wait a moment for FPV.py to initialize camera
sleep 2

# Check if FPV.py is still running
if ! kill -0 $FPV_PID 2>/dev/null; then
    echo "❌ ERROR: FPV.py failed to start or crashed"
    exit 1
fi

echo "✓ FPV.py started (PID: $FPV_PID)"

# Start FPV_ROS2_simple.py in foreground (publishes to ROS2 topics)
echo "Starting FPV_ROS2_simple.py (ROS2 publisher)..."
python3 FPV_ROS2_simple.py

# If FPV_ROS2_simple.py exits, kill FPV.py too
echo "FPV_ROS2_simple.py exited, stopping FPV.py..."
kill $FPV_PID 2>/dev/null
wait $FPV_PID 2>/dev/null

echo "Camera container shutdown complete"
