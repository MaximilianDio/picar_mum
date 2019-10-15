#!/bin/bash
export ROS_HOSTNAME="$HOSTNAME.local"
output=( $SSH_CLIENT )
export ROS_MASTER_URI="http://${output[0]}:11311"
source /home/pi/picar_mum/catkin_ws/devel/setup.bash

exec "$@"
