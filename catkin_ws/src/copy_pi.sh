#!/usr/bin/env bash

if [ -z "$1" ]
then
  echo "usage: 'copy_pi.sh picarXX'"
  echo "Replace XX with your car number"
  exit
fi


scp -r 00_common pi@$1.local:/home/pi/picar_mum/catkin_ws/src/
scp -r 10_hardware pi@$1.local:/home/pi/picar_mum/catkin_ws/src/
scp -r 30_lane_follower pi@$1.local:/home/pi/picar_mum/catkin_ws/src/
scp -r 40_leader_follower pi@$1.local:/home/pi/picar_mum/catkin_ws/src/
scp -r 50_project pi@$1.local:/home/pi/picar_mum/catkin_ws/src/
scp -r color_normalization pi@$1.local:/home/pi/picar_mum/catkin_ws/src/