#!/bin/bash
# ROS2 Camera Test Script
# Tests if camera topics are being published correctly

echo "=========================================="
echo "ROS2 Camera Topic Test"
echo "=========================================="

# Setup ROS2 environment
export MAMBA_EXE='/usr/local/bin/micromamba'
export MAMBA_ROOT_PREFIX='/home/pi/micromamba'
eval "$($MAMBA_EXE shell hook --shell bash --root-prefix $MAMBA_ROOT_PREFIX)"
micromamba activate ros_env
export ROS_DOMAIN_ID=1

echo ""
echo "Step 1: Checking if ROSServer is running..."
if pgrep -f "python3.*ROSServer" > /dev/null; then
    echo "✓ ROSServer is running"
else
    echo "✗ ROSServer is NOT running!"
    echo "  Please start it with: python3 ./Server/ROSServer.py"
    exit 1
fi

echo ""
echo "Step 2: Checking ROS2 topics..."
ros2 topic list | grep raspclaws > /tmp/ros2_topics.txt
echo "Available raspclaws topics:"
cat /tmp/ros2_topics.txt

echo ""
echo "Step 3: Checking if camera topics exist..."
if grep -q "camera" /tmp/ros2_topics.txt; then
    echo "✓ Camera topics found"
else
    echo "⚠ Camera topics NOT found - triggering hardware init..."
    echo "  Calling /raspclaws/reset_servos service..."
    timeout 10s ros2 service call /raspclaws/reset_servos std_srvs/srv/Trigger
    sleep 3

    # Check again
    ros2 topic list | grep camera
    if [ $? -eq 0 ]; then
        echo "✓ Camera topics now available"
    else
        echo "✗ Camera topics still not available"
        echo "  Hardware might not be initialized properly"
    fi
fi

echo ""
echo "Step 4: Checking camera status in GUIServer..."
if grep -q "RESUMED" /tmp/guiserver.log | tail -5; then
    echo "✓ Camera is RESUMED in GUIServer"
elif grep -q "PAUSED" /tmp/guiserver.log | tail -5; then
    echo "⚠ Camera is PAUSED in GUIServer"
    echo "  Please activate camera via Web-GUI"
else
    echo "? Unable to determine camera status"
fi

echo ""
echo "Step 5: Testing camera topic rate..."
echo "Checking /raspclaws/camera/image_compressed for 5 seconds..."
timeout 5s ros2 topic hz /raspclaws/camera/image_compressed

if [ $? -eq 124 ]; then
    echo "⚠ No messages received on camera topic"
    echo ""
    echo "Troubleshooting:"
    echo "1. Make sure camera is ACTIVE in GUIServer (Web-GUI)"
    echo "2. Check GUIServer log: tail -20 /tmp/guiserver.log | grep camera"
    echo "3. Check ROSServer log: tail -20 /tmp/rosserver.log | grep camera"
elif [ $? -eq 0 ]; then
    echo "✓ Camera topic is publishing!"
fi

echo ""
echo "Step 6: Checking ZMQ connection..."
echo "ROSServer camera_publisher status:"
tail -30 /tmp/rosserver.log | grep -E "camera_publisher|ZMQ|Receiving frames" | tail -10

echo ""
echo "=========================================="
echo "Test complete!"
echo "=========================================="
echo ""
echo "If camera topics are not publishing:"
echo "1. Activate camera via Web-GUI"
echo "2. Check: tail -f /tmp/rosserver.log | grep camera"
echo "3. Restart ROSServer if needed"
