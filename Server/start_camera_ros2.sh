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

# Check if camera device exists
if [ ! -e /dev/video0 ]; then
    echo "❌ ERROR: Camera device /dev/video0 not found"
    echo "   Camera is not connected or not accessible"
    echo "   Container will sleep to prevent restart loop"
    # Sleep forever to prevent Docker restart loop
    sleep infinity
    exit 1
fi

# Check if camera is already in use by another process
# (e.g., GUIServer running on host)
echo "Checking if camera is already in use..."
if lsof /dev/video0 2>/dev/null | grep -q video0; then
    echo "⚠️  WARNING: Camera is already in use by another process:"
    lsof /dev/video0 2>/dev/null || true
    echo ""
    echo "This is usually caused by:"
    echo "  - GUIServer running on the host"
    echo "  - Another camera application"
    echo ""
    echo "To fix this:"
    echo "  1. Stop GUIServer: bash Server/stop_guiserver.sh"
    echo "  2. Restart this container: docker compose restart raspclaws_camera"
    echo ""
    echo "Container will sleep to prevent restart loop"
    sleep infinity
    exit 1
fi

echo "✓ Camera device is available"

# Start FPV.py in background (provides camera frames via ZMQ)
echo "Starting FPV.py (camera capture + ZMQ stream)..."
cd /ros2_ws/Server
python3 FPV.py &
FPV_PID=$!

# Wait a moment for FPV.py to initialize camera
echo "Waiting 3 seconds for FPV.py to initialize..."
sleep 3

# Check if FPV.py is still running
if ! kill -0 $FPV_PID 2>/dev/null; then
    echo "❌ ERROR: FPV.py failed to start or crashed"
    echo "   Check logs above for camera initialization errors"
    echo ""
    echo "Common causes:"
    echo "  - Camera pipeline already in use (check: lsof /dev/video0)"
    echo "  - libcamera initialization failed"
    echo "  - Insufficient permissions"
    echo ""
    echo "Container will sleep to prevent restart loop"
    # Sleep forever to prevent Docker restart loop
    sleep infinity
    exit 1
fi

echo "✓ FPV.py started successfully (PID: $FPV_PID)"

# Start FPV_ROS2_simple.py in foreground (publishes to ROS2 topics)
echo "Starting FPV_ROS2_simple.py (ROS2 publisher)..."
python3 FPV_ROS2_simple.py

# If FPV_ROS2_simple.py exits, kill FPV.py too
echo "FPV_ROS2_simple.py exited, stopping FPV.py..."
kill $FPV_PID 2>/dev/null
wait $FPV_PID 2>/dev/null

echo "Camera container shutdown complete"


