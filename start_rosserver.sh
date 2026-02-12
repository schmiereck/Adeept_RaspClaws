#!/bin/bash
# Start ROSServer with correct ROS2 environment

export MAMBA_EXE='/usr/local/bin/micromamba'
export MAMBA_ROOT_PREFIX='/home/pi/micromamba'
eval "$($MAMBA_EXE shell hook --shell bash --root-prefix $MAMBA_ROOT_PREFIX)"
micromamba activate ros_env

# IMPORTANT: Set ROS_DOMAIN_ID
export ROS_DOMAIN_ID=1

# Source ROS2 workspace for custom interfaces (if available)
if [ -f /home/pi/ros2_ws/install/setup.bash ]; then
    echo "Sourcing ROS2 workspace with custom interfaces..."
    source /home/pi/ros2_ws/install/setup.bash
else
    echo "WARNING: ROS2 workspace not found at /home/pi/ros2_ws"
    echo "  (Action servers will not be available)"
fi

cd /home/pi/Adeept_RaspClaws/Server
echo "=== Starting ROSServer ==="
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Python: $(which python3)"
python3 ROSServer.py
