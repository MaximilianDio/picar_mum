#!/usr/bin/env python
import serial
import time
import struct
import numpy as np
from picar_msgs.msg import WheelSpeedStamped
import rospy

ticks_per_sec = 16000000.0/256.0
pulses_per_rev = 10


class Node(object):
    def __init__(self):
        self.serial = serial.Serial("/dev/ttyUSB0", 57600, timeout=1.0)
        time.sleep(2.0)
        rospy.init_node("rpm")
        self.publisher = rospy.Publisher(
            "wheel_speed",
            WheelSpeedStamped,
            queue_size=1)





if __name__ == "__main__":
    n = Node()
    speeds = [0, 0, 0]
    index = 0
    data = n.serial.read()
    print(len(data))
    while not rospy.is_shutdown():
        time.sleep(0.1)
        n.serial.write(b"\x01")
        data = n.serial.read(8)
        print(len(data))
        data = struct.unpack('>HHHH', data)
        speed = []
        for ticks in data:
            speed.append(ticks_per_sec/ticks/pulses_per_rev*2*3.1416 if ticks > 0 else 0.0)
        print(data)
        print(speed)
        message = WheelSpeedStamped()
        message.front_left = speed[0]
        message.front_right = speed[1]
        message.rear_left = speed[2]
        message.rear_right = speed[3]
        n.publisher.publish(message)
