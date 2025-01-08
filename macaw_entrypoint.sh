#!/bin/bash
. /opt/ros/${ROS_DISTRO}/setup.sh
. /ros_ws/install/setup.sh
echo $SITL_HOSTNAME
export SITL_IP=$(getent ahostsv4 $SITL_HOSTNAME | head -1 | awk '{print $1}')
echo $SITL_IP
exec "$@"
