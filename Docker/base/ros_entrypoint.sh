#!/bin/bash
set -e

# setup ros2 environment
# source /opt/ros/dashing/setup.sh
. ~/ros2_dashing/install/local_setup.bash
source "$ROS2_WS/install/setup.bash"
exec "$@"
