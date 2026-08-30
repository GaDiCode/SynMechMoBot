#!/bin/bash
# Entrypoint cho Docker container ROS Noetic
set -e

# Source ROS
source /opt/ros/noetic/setup.bash

# Build workspace
if [ ! -f /catkin_ws/devel/setup.bash ]; then
    echo "[synmech] First time running — building catkin workspace..."
    cd /catkin_ws
    catkin_make -DCATKIN_BLACKLIST_PACKAGES="synmech_sim;synmech_navigation"
    echo "[synmech] Builded!"
fi

# Source workspace
source /catkin_ws/devel/setup.bash

if [ -f /catkin_ws/src/synmech_mobot/network.env ]; then
    export $(grep -v '^#' /catkin_ws/src/synmech_mobot/network.env | xargs)
fi

# Set ROS network explicitly from network.env variables
export ROS_MASTER_URI=http://${OPI_IP}:${ROS_PORT}
export ROS_IP=${OPI_IP}

echo "============================================"
echo " SynMech Robot — ROS Noetic (Docker)"
echo " ROS_MASTER_URI: $ROS_MASTER_URI"
echo " ROS_IP:         $ROS_IP"
echo "============================================"

exec "$@"
