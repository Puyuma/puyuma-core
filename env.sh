#!/bin/bash

ROS_WS=~/catkin_ws
. ${ROS_WS}/devel/setup.bash
. /opt/ros/kinetic/setup.bash

export PATH=$ROS_ROOT/bin:$PATH

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROS_WS
export ROS_HOSTNAME=$HOSTNAME.local
export ROS_IP=`hostname -I`

. ${ROS_WS}/devel/setup.sh

exec "$@"
