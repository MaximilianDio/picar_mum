#!/usr/bin/env python
import rospy
import yaml
import numpy as np
from geometry_msgs.msg import Point32
from picar_common.picar_common import get_param, get_config_file_path, get_camera_info
import world_projection


class WorldProjectionNode(object):
    def __init__(self):
        self.pos_blue_ball = [-1, -1]
        self.pos_green_ball = [-1, -1]

        self._params = {}
        self.publishers = {}
        self.services = {}

        # distorted_input is set to False only in simulation mode
        distorted_input = get_param("~distorted_input", "False")

        camera_info = get_camera_info()

        extrinsics_file_name = rospy.get_namespace().strip("/")
        extrinsics_file_path = get_config_file_path(
            "extrinsics",
            extrinsics_file_name)
        intrinsics_file_path = get_config_file_path(
            "intrinsics",
            extrinsics_file_name)

        with open(extrinsics_file_path, "r") as file_handle:
            data = yaml.safe_load(file_handle)
            h_matrix = np.array(data["H"]["data"]).reshape(3, 3)
            s_matrix = np.array(data["S"]["data"]).reshape(3, 3)
            t_matrix = np.array(data["t"]["data"]).reshape(3, 1)

        with open(intrinsics_file_path, "r") as file_handle:
            data = yaml.safe_load(file_handle)
            k_matrix = np.array(data["projection_matrix"]["data"]).reshape(3, 4)
            k_matrix = k_matrix[:, 0:3]

        # calculate projection matrix P = K*[S|t]
        p_matrix = np.matmul(k_matrix, np.concatenate((s_matrix, t_matrix), axis=1))


        # create instance of the WorldProjector
        self.projector = world_projection.WorldProjector(
            camera_info,
            h_matrix,
            p_matrix,
            distorted_input)

        # register all publishers
        self.init_publishers()

        # register all publishers
        self.init_subscribers()

    def __run(self):
        ball_blue = self.projector.pixel2world(self.pos_blue_ball)
        ball_green = self.projector.pixel2world(self.pos_green_ball)

        self.publishers["leader_blue_ball_position_output"].publish(ball_blue)
        self.publishers["leader_green_ball_position_output"].publish(ball_green)

    def get_position_blue_ball_cb(self, data):
        self.pos_blue_ball[0] = data.x
        self.pos_blue_ball[1] = data.y

    def get_position_green_ball_cb(self, data):
        self.pos_green_ball[0] = data.x
        self.pos_green_ball[1] = data.y

        # run node operation only once - choice is arbitrary!
        self.__run()

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
    WorldProjectionNode()

    rospy.spin()
