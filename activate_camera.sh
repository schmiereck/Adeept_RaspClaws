#!/bin/bash
# Activate Camera via ROS2 Service
# This script resumes the camera stream in GUIServer via ROS2

echo "=========================================="
echo "Activating RaspClaws Camera"
echo "=========================================="

# Setup ROS2 environment
if [ -x "/usr/local/bin/micromamba" ]; then
    # On raspclaws-1 (with micromamba)
    export MAMBA_EXE='/usr/local/bin/micromamba'
    export MAMBA_ROOT_PREFIX='/home/pi/micromamba'
    eval "$($MAMBA_EXE shell hook --shell bash --root-prefix $MAMBA_ROOT_PREFIX)"
    micromamba activate ros_env
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    # On ubuntu1 (native ROS2)
    source /opt/ros/humble/setup.bash
else
    echo "ERROR: No ROS2 installation found"
    exit 1
fi

export ROS_DOMAIN_ID=1

echo ""
echo "Step 1: Checking if ROSServer is running..."
if ros2 node list 2>&1 | grep -q "raspclaws_node"; then
    echo "✓ ROSServer is running"
else
    echo "✗ ROSServer is NOT running!"
    echo "  Start it first: cd /home/pi/Adeept_RaspClaws && ./start_rosserver.sh"
    exit 1
fi

echo ""
echo "Step 2: Checking current camera status..."
# Get current status from the status topic
timeout 2s ros2 topic echo /raspclaws/status --once 2>/dev/null | grep -i camera

echo ""
echo "Step 3: Activating camera (resume stream)..."
# Set camera_pause to False to resume camera
ros2 service call /raspclaws/set_camera_pause std_srvs/srv/SetBool "data: false"

if [ $? -eq 0 ]; then
    echo "✓ Camera activation command sent successfully"

    echo ""
    echo "Step 4: Waiting for camera to start (3 seconds)..."
    sleep 3

    echo ""
    echo "Step 5: Verifying camera is streaming..."
    if timeout 3s ros2 topic hz /raspclaws/camera/image_compressed 2>&1 | grep -q "average rate"; then
        echo "✓ Camera is streaming!"
        echo ""
        echo "You can now view the camera stream:"
        echo "  rqt_image_view    # Select /raspclaws/camera/image_raw"
        echo "  rviz2             # Add Image display"
    else
        echo "⚠ Camera topics exist but no frames are being published"
        echo ""
        echo "Troubleshooting:"
        echo "1. Check if camera is paused in GUIServer:"
        echo "   tail -20 /tmp/guiserver.log | grep camera"
        echo ""
        echo "2. Check ROSServer camera publisher:"
        echo "   tail -30 /tmp/rosserver.log | grep camera"
        echo ""
        echo "3. Try toggling camera via Web-GUI"
    fi
else
    echo "✗ Failed to send camera activation command"
    echo "  Check if service is available:"
    echo "  ros2 service list | grep camera"
fi

echo ""
echo "=========================================="
echo "Done!"
echo "=========================================="
