#!/bin/bash
export MAMBA_EXE='/usr/local/bin/micromamba'
export MAMBA_ROOT_PREFIX='/home/pi/micromamba'
eval "$($MAMBA_EXE shell hook --shell bash --root-prefix $MAMBA_ROOT_PREFIX)"
micromamba activate ros_env
export ROS_DOMAIN_ID=1
ros2 topic list | grep raspclaws
