#!/bin/bash
. /opt/ros/${ROS_DISTRO}/setup.sh
. /ros_ws/install/setup.sh
exec "$@"
