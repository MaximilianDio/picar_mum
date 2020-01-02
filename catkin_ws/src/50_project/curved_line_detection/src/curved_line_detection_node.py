#!/usr/bin/env python
import rospy
import yaml
import numpy as np
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from picar_msgs.srv import SetValue
from sensor_msgs.msg import CompressedImage, CameraInfo
from picar_common.picar_common import get_config_file_path, get_param, set_param, get_camera_info
from curved_line_detection import CurvePointExtractor, CurveEstimator
import time
import cv2


class CurveDetector():
    def __init__(self):

        # dictionaries for easy access
        self._params = {}
        self.services = {}
        self.publishers = {}
        self.subscribers = {}

        # import parameters from config yaml files
        config_file_name = get_param("~config_file_name", "default")

        config_file_path = get_config_file_path("leader_detection",
                                                config_file_name)

        # shut down node if no config file could be found
        if config_file_path is None:
            rospy.signal_shutdown("Could not find any config file... "
                                  "Shutting down!")

        # read the config parameters form .yaml file
        self.setup_params(config_file_path)

        # get Camera data
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

        # create curve point detector
        # TODO use values from yaml file
        self.__curve_point_detector = CurvePointExtractor(np.matrix([[90, 124, 124], [180, 255, 255]]), 10,
                                                          [1, 0.35, 0.35])

        # create Curve estimator
        if camera_info.D[0] == 0.0:
            self.projector = CurveEstimator(camera_info, H, False)
        else:
            self.projector = CurveEstimator(camera_info, H, True)

        # register all publishers
        self.init_publishers()

        # register all publishers
        self.init_subscribers()

    # --------------------------------------------------------------------
    # ----------------------- initialization -----------------------------
    # --------------------------------------------------------------------
    def setup_params(self, config_file_path):
        with open(config_file_path, "r") as file_handle:
            data = yaml.safe_load(file_handle)
        self._params = data
        for param_name in self._params:
            set_param("~" + param_name, self._params[param_name])

    def init_subscribers(self):
        """ initialize ROS subscribers and stores them in a dictionary"""
        self.subscribers["image_input"] = rospy.Subscriber("~input_image/raw", Image, self.rcv_img_cb)

    def init_publishers(self):
        """ initialize ROS publishers and stores them in a dictionary"""
        pass

    # --------------------------------------------------------------------
    # ----------------------- main callback ------------------------------
    # --------------------------------------------------------------------
    def rcv_img_cb(self, image_data):
        """
            receive image callback: process image and processes it.
            :param image_data: form camera
            """
        t0 = time.time()
        # convert to open cv format
        img_bgr = self.bridge.imgmsg_to_cv2(image_data)

        # extract curve points from image
        curve_points_image = self.__curve_point_detector.detect_curve(img_bgr)

        # transform curve image points to world and extract curve data
        # TODO: transform curve points to world and extract curve data

        print time.time() - t0

        # DEBUG
        for curve_point in curve_points_image:
            cv2.circle(img_bgr, (curve_point[0], curve_point[1]), 5, (255, 0, 0))
        cv2.imshow("line", img_bgr)
        cv2.waitKey(0)


if __name__ == "__main__":
    rospy.init_node("world_projection_node")
    CurveDetector()
    rospy.spin()
