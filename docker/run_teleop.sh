#!/bin/bash
# ============================================
# Run teleop_twist_keyboard inside the running roscore container
# Usage: ./run_teleop.sh  (from OPi, in docker/ directory)
# ============================================
docker exec -it synmech_roscore bash -c '
  source /opt/ros/noetic/setup.bash
  source /catkin_ws/devel/setup.bash 2>/dev/null
  export ROS_MASTER_URI=http://${OPI_IP}:${ROS_PORT}
  export ROS_IP=${OPI_IP}
  echo "============================================"
  echo " Teleop Twist Keyboard"
  echo " ROS_MASTER_URI: $ROS_MASTER_URI"
  echo " ROS_IP:         $ROS_IP"
  echo "============================================"
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py
'
