#!/bin/bash
# Test script for ROS 2 Client (PC/Jetson)
# Tests connectivity to RaspClaws ROS 2 Server

set -e

echo "========================================"
echo "RaspClaws ROS 2 Client Test Script"
echo "========================================"
echo ""

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS 2 not sourced!"
    echo "Run: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✓ ROS 2 $ROS_DISTRO sourced"
echo ""

# Check ROS_DOMAIN_ID
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-not set (using default 0)}"
echo ""

# Test 1: List nodes
echo "Test 1: Listing ROS 2 nodes..."
timeout 5 ros2 node list || {
    echo "❌ Cannot list nodes"
    echo "Possible issues:"
    echo "  - Pi not running"
    echo "  - Network issue"
    echo "  - Different ROS_DOMAIN_ID"
    exit 1
}

if timeout 5 ros2 node list | grep -q raspclaws_node; then
    echo "✓ Found raspclaws_node"
else
    echo "❌ raspclaws_node not found"
    exit 1
fi
echo ""

# Test 2: List topics
echo "Test 2: Listing topics..."
timeout 5 ros2 topic list | grep raspclaws || {
    echo "❌ No raspclaws topics found"
    exit 1
}
echo "✓ Topics found"
echo ""

# Test 3: Echo status topic
echo "Test 3: Reading status topic..."
timeout 3 ros2 topic echo /raspclaws/status --once || {
    echo "❌ Cannot read status topic"
}
echo ""

# Test 4: Send movement command
echo "Test 4: Sending test movement command..."
echo "WARNING: Robot will move slightly!"
read -p "Continue? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Sending forward command..."
    ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
    sleep 2
    echo "Sending stop command..."
    ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
    echo "✓ Movement test complete"
else
    echo "⊘ Movement test skipped"
fi
echo ""

# Test 5: Call reset service
echo "Test 5: Calling reset service..."
ros2 service call /raspclaws/reset_servos std_srvs/Trigger || {
    echo "❌ Cannot call reset service"
}
echo ""

# Test 6: Check battery topic
echo "Test 6: Reading battery voltage..."
timeout 3 ros2 topic echo /raspclaws/battery --once || {
    echo "❌ Cannot read battery topic"
}
echo ""

echo "========================================"
echo "✓ All tests completed!"
echo "========================================"
echo ""
echo "You can now:"
echo "  1. Use teleop keyboard:"
echo "     ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/raspclaws/cmd_vel"
echo ""
echo "  2. Monitor topics:"
echo "     ros2 topic echo /raspclaws/status"
echo ""
echo "  3. Get node info:"
echo "     ros2 node info /raspclaws_node"
