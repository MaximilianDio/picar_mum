#!/usr/bin/env python
import rospy
import ground_projection.ground_projection as ground_projection
import yaml
import numpy as np
from geometry_msgs.msg import Point32
from picar_common.picar_common import get_param, get_config_file_path, get_camera_info


class GroundProjectionNode(object):
    def __init__(self):
        rospy.init_node("ground_projection_node")
        # distorted_input is set to False only in simulation mode
        distorted_input = get_param("~distorted_input", "False")

        camera_info = get_camera_info()

        extrinsics_file_name = rospy.get_namespace().strip("/")
        extrinsics_file_path = get_config_file_path(
            "extrinsics",
            extrinsics_file_name)

        with open(extrinsics_file_path, "r") as file_handle:
            data = yaml.safe_load(file_handle)
            h_matrix = np.array(data["H"]["data"]).reshape(3, 3)

        # create instance of the GroundProjector
        self.projector = ground_projection.GroundProjector(
            camera_info,
            h_matrix,
            distorted_input)

        self.pub_track_position = rospy.Publisher(
            "~track_position_output",
            Point32,
            queue_size=1)

        self.sub_track_position = rospy.Subscriber(
            "~track_position_input",
            Point32,
            self.track_position_callback,
            queue_size=1)

        rospy.loginfo("[{}] Started.".format(rospy.get_name()))

    def track_position_callback(self, msg):

        # calculate ground coordinates
        point = self.projector.pixel2ground((msg.x, msg.y))

        self.pub_track_position.publish(point)


if __name__ == "__main__":
    GroundProjectionNode()
    rospy.spin()
