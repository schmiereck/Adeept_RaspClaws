#!/bin/bash
# Start ROSServer with correct ROS2 environment

export MAMBA_EXE='/usr/local/bin/micromamba'
export MAMBA_ROOT_PREFIX='/home/pi/micromamba'
eval "$($MAMBA_EXE shell hook --shell bash --root-prefix $MAMBA_ROOT_PREFIX)"
micromamba activate ros_env

# IMPORTANT: Set ROS_DOMAIN_ID
export ROS_DOMAIN_ID=1

cd /home/pi/Adeept_RaspClaws/Server
echo "=== Starting ROSServer ==="
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Python: $(which python3)"
python3 ROSServer.py
