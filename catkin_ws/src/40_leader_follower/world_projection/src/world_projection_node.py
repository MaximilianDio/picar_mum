#!/usr/bin/env python
import rospy
import yaml
import numpy as np
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Point32
from picar_common.picar_common import get_config_file_path, get_param, get_camera_info
from world_projection import WorldProjector


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

        name = get_param("~extrinsics_file_name", "default")
        path = get_config_file_path("extrinsics", name)
        camera_info = get_camera_info()

        if path is None:
            rospy.logfatal("No extrinsics found!")
            rospy.signal_shutdown("No extrinsics found!")
        with open(path, "r") as f:
            data = yaml.safe_load(f)
            H = np.array(data["H"]["data"])
            H = np.resize(H, (3, 3))

        rospy.loginfo("[{}] Waiting for camera_info message..."
                      "".format(rospy.get_name()))

        camera_info = rospy.wait_for_message("camera_node/camera_info",
                                             CameraInfo)
        rospy.loginfo("[{}] Received camera_info message."
                      "".format(rospy.get_name()))

        if camera_info.D[0] == 0.0:
            self.projector = WorldProjector(camera_info, H, False)
        else:
            self.projector = WorldProjector(camera_info, H, True)

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
                ball_blue = array2point32msg(self.projector.pixel2ground(self.pos_blue_ball))
                ball_green = array2point32msg(self.projector.pixel2ground(self.pos_green_ball))

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
