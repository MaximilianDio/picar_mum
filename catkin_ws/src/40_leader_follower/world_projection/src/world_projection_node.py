#!/usr/bin/env python
import rospy
import yaml
import numpy as np
from geometry_msgs.msg import Point32
from picar_common.picar_common import get_param, get_config_file_path, get_camera_info


class GroundProjectionNode(object):
    def __init__(self):
        self.pos_blue_ball = None
        self.pos_green_ball = None

        self._params = {}
        self.publishers = {}
        self.services = {}

        # register all publishers
        self.init_subscribers()
        # register all publishers
        self.init_publishers()

    def get_position_blue_ball_cb(self, data):
        self.pos_blue_ball = data

    def get_position_green_ball_cb(self, data):
        self.pos_green_ball = data

    def init_subscribers(self):
        """ initialize ROS subscribers and stores them in a dictionary"""
        # relative position of leaders blue ball to picar
        rospy.Subscriber("~leader_blue_ball_position_input", Point32, self.get_position_blue_ball_cb)
        # relative position of leaders green ball to picar
        rospy.Subscriber("~leader_green_ball_position_input", Point32, self.get_position_green_ball_cb)

    def init_publishers(self):
        """ initialize ROS publishers and stores them in a dictionary"""
        # relative position of leaders blue ball to picar
        self.publishers["leader_blue_ball_position_output"] = rospy.Publisher("~leader_blue_ball_position_output",
                                                                              Point32,
                                                                              queue_size=1)

        # relative position of leaders green ball to picar
        self.publishers["leader_green_ball_position_output"] = rospy.Publisher("~leader_green_ball_position_output",
                                                                               Point32,
                                                                               queue_size=1)


if __name__ == "__main__":
    rospy.init_node("world_projection_node")
    GroundProjectionNode()
    rospy.spin()
