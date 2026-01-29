#!/bin/bash
# Debug-Script für ROSServer Topics

echo "=========================================="
echo "ROSServer Topic Debugging"
echo "=========================================="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

echo "1. Liste aller Topics:"
ros2 topic list
echo ""

echo "2. Topic Info für /raspclaws/gyro_data:"
ros2 topic info /raspclaws/gyro_data
echo ""

echo "3. Topic Type für /raspclaws/gyro_data:"
ros2 topic type /raspclaws/gyro_data
echo ""

echo "4. Warte 5 Sekunden auf gyro_data (mit timeout):"
timeout 5 ros2 topic echo /raspclaws/gyro_data --once
echo ""

echo "5. Node Info:"
ros2 node info /raspclaws_node
echo ""

echo "6. Topic Hz (Publish-Rate):"
timeout 5 ros2 topic hz /raspclaws/gyro_data
echo ""

echo "=========================================="
echo "Debug-Script fertig"
echo "=========================================="
