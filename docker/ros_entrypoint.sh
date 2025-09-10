#!/bin/bash

#source workspace
source /opt/ros/$ROS_DISTRO/setup.bash
source $ROS_WS/install/setup.bash

exec "$@"
