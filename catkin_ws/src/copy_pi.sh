#!/usr/bin/env bash
scp -r 00_common pi@picar01.local:/home/pi/picar_mum/catkin_ws/src/
scp -r 10_hardware pi@picar01.local:/home/pi/picar_mum/catkin_ws/src/
scp -r 30_lane_follower pi@picar01.local:/home/pi/picar_mum/catkin_ws/src/
scp -r color_normalization pi@picar01.local:/home/pi/picar_mum/catkin_ws/src/