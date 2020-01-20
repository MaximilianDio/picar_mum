#!/usr/bin/python

import os
import time
import pygame
import rospy
from picar_msgs.msg import CarCmd
from picar_msgs.srv import SetValue
from velocity_controller import *
from std_msgs.msg import Float32


class VelocityControllerTuning(object):

    def __init__(self):
        self.controller = VelocityController(0.12, 0.025)
        self.publisher = rospy.Publisher("motor_node/car_cmd",
                                         CarCmd,
                                         queue_size=1)

        # listen to encoder node
        self.subscriber = rospy.Subscriber("~velocity_estimated",
                                           Float32,
                                           self.update_current_velocity,
                                           queue_size=1)


    def update_current_velocity(self, message):
        self.cur_vel = message.data

    def run(self):
        """Main logic loop. Should be called periodically in a loop"""
        running = True
        while running:
            self.pub_message()
            time.sleep(0.03)


    def pub_message(self):
        """Publishes a CarCmd mzessage to control the car.
        Args:
            keys(list): List of bools corresponding to pressed/unpressed keys
        """
        car_cmd = CarCmd()
        car_cmd.velocity = 0.0
        car_cmd.angle = 0.0

        car_cmd.velocity = self.controller.get_velocity_output(self.cur_vel, 0.5)

        self.publisher.publish(car_cmd)


def main():
    """Inits the ROS node and KeyboardController."""
    rospy.init_node("keyboard_control_node")

    node = VelocityControllerTuning()
    try:
        node.run()
    except rospy.ROSInitException as error:
        rospy.signal_shutdown("{}".format(error))


if __name__ == "__main__":
    main()