#!/bin/bash
# Show sensor_msgs/Imu message structure

source /opt/ros/humble/setup.bash

echo "=========================================="
echo "sensor_msgs/Imu Message Structure"
echo "=========================================="
echo ""

ros2 interface show sensor_msgs/msg/Imu

echo ""
echo "=========================================="
echo "Alternativ: geometry_msgs/Vector3"
echo "=========================================="
echo ""

ros2 interface show geometry_msgs/msg/Vector3

echo ""
echo "=========================================="
