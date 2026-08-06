#!/bin/bash
# Entrypoint cho Docker container ROS Noetic
set -e

# Source ROS
source /opt/ros/noetic/setup.bash

# Build workspace nếu chưa build
if [ ! -f /catkin_ws/devel/setup.bash ]; then
    echo "[synmech] Lần đầu chạy — đang build catkin workspace..."
    cd /catkin_ws
    # Chỉ build các package cần thiết cho OPi (bỏ qua sim và nav để tránh lỗi thiếu thư viện Gazebo/MoveBase)
    catkin_make -DCATKIN_BLACKLIST_PACKAGES="synmech_sim;synmech_navigation"
    echo "[synmech] Build xong!"
fi

# Source workspace
source /catkin_ws/devel/setup.bash

# Set ROS network (đọc từ biến môi trường)
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://192.168.0.99:2710}
export ROS_IP=${ROS_IP:-192.168.0.99}

echo "============================================"
echo " SynMech Robot — ROS Noetic (Docker)"
echo " ROS_MASTER_URI: $ROS_MASTER_URI"
echo " ROS_IP:         $ROS_IP"
echo "============================================"

exec "$@"
