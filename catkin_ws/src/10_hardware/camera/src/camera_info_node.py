#!/usr/bin/env python
import os
import yaml
import rospy
import rospkg
from sensor_msgs.msg import CompressedImage, CameraInfo
from picar_common import get_config_file_path, get_param


class CameraInfoReader(object):
    def __init__(self):
        intrinsics_file_name = get_param("~intrinsics_file_name", "default")
        intrinsics_file_path = get_config_file_path("intrinsics",
                                                    intrinsics_file_name)

        if intrinsics_file_path is None:
            rospy.signal_shutdown("Could not find intrinsics config file. "
                                  "Shutting down!")

        # create publisher to publish CameraInfo messages
        self.pub_camera_info = rospy.Publisher("~camera_info",
                                               CameraInfo,
                                               queue_size=1)

        self.camera_info_msg = None

        # try to read calibration information from the calibration file
        with open(intrinsics_file_path, 'r') as intrinsics_file:
            try:
                data = yaml.safe_load(intrinsics_file)
            except yaml.YAMLError as e:
                rospy.signal_shutdown("Could not read yaml file... Shutting down!\n{}".format(e))
            else:
                self.camera_info_msg = CameraInfo()
                self.camera_info_msg.width = data["image_width"]
                self.camera_info_msg.height = data["image_height"]
                self.camera_info_msg.K = data["camera_matrix"]["data"]
                self.camera_info_msg.D = data["distortion_coefficients"]["data"]
                self.camera_info_msg.R = data["rectification_matrix"]["data"]
                self.camera_info_msg.P = data["projection_matrix"]["data"]
                self.camera_info_msg.distortion_model = data["distortion_model"]
                self.camera_info_msg.header.frame_id = rospy.get_namespace() + "camera_optical_frame"

        rospy.loginfo("[{}] Camera Info: {}".format(rospy.get_name(), self.camera_info_msg))

        # subscribe to generic topic of image messages of camera (needs to be remapped in launch file)
        self.sub_img = rospy.Subscriber("~compressed_image", CompressedImage, self.img_cb, queue_size=1)

    def img_cb(self, msg):
        """
        If node receives images from the subscribed topic, this function is called and publishes info message
        :param msg: should be compressed image of the camera
        :return:
        """
        if self.camera_info_msg is not None:
            self.camera_info_msg.header.stamp = msg.header.stamp
            self.pub_camera_info.publish(self.camera_info_msg)

    def on_shutdown(self):
        rospy.loginfo("[{}] shutting down".format(rospy.get_name()))


if __name__ == '__main__':
    rospy.init_node("camera_info_node", anonymous=False)
    node = CameraInfoReader()
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()
