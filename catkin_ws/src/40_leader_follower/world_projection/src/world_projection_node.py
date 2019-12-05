#!/usr/bin/env python
import rospy
import yaml
import numpy as np
from geometry_msgs.msg import Point32
from picar_common.picar_common import get_param, get_config_file_path, get_camera_info
import world_projection


def array2point32msg(array):
    """ transforms a array into a Point32 message
        :param array
    """
    msg = Point32()

    msg.x = array[0]
    msg.y = array[1]
    msg.z = array[2]

    return msg


class WorldProjectionNode(object):
    def __init__(self):
        # TODO init with None and find way to deal with None declaration
        self.pos_blue_ball = None
        self.pos_green_ball = None

        self._params = {}

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

        # load intrinsic and extrinsic camera data
        with open(extrinsics_file_path, "r") as file_handle:
            data = yaml.safe_load(file_handle)
            r_matrix = np.array(data["S"]["data"]).reshape(3, 3)
            t_matrix = np.array(data["t"]["data"]).reshape(3, 1)

        with open(intrinsics_file_path, "r") as file_handle:
            data = yaml.safe_load(file_handle)
            k_matrix = np.array(data["projection_matrix"]["data"]).reshape(3, 4)

        # create instance of the WorldProjector
        self.projector = world_projection.WorldProjector(
            camera_info,
            r_matrix,
            t_matrix,
            k_matrix,
            distorted_input)

        # register all publishers
        self.publishers = {}
        self.init_publishers()

        # register all publishers
        self.init_subscribers()

    def __run(self):
        # wait until both positions where initialized by ros message
        if self.pos_blue_ball is not None and self.pos_green_ball is not None:
            # if pixle coordinates are [-1 -1] then blob could not be detected
            if self.pos_blue_ball[0] < 0 or self.pos_green_ball[0] < 0:
                # TODO: @Paul und @Matti check if values [0 0 0] are ok for the controller or choose new ones!
                ball_blue = array2point32msg([0, 0, 0])
                ball_green = array2point32msg([0, 0, 0])
            else:
                ball_blue = array2point32msg(self.projector.pixel2world(self.pos_blue_ball))
                ball_green = array2point32msg(self.projector.pixel2world(self.pos_green_ball))

            # start publishing the first time the blobs could be detected
            self.publishers["leader_blue_ball_position_output"].publish(ball_blue)
            self.publishers["leader_green_ball_position_output"].publish(ball_green)

    def get_position_blue_ball_cb(self, data):
        self.pos_blue_ball = [data.x, data.y]

    def get_position_green_ball_cb(self, data):
        self.pos_green_ball = [data.x, data.y]

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
