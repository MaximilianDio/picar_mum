#!/usr/bin/env python
import rospy
import yaml
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from picar_msgs.msg import MsgCurvePoint2D
from picar_common.picar_common import get_config_file_path, get_param, set_param, get_camera_info
from curved_line_detection import CurvePointExtractor, CurveEstimator
import time
import cv2
import matplotlib.pyplot as plt


class CurveDetector:
    # TODO define rate for picar
    PUBLISH_RATE = 10

    def __init__(self):
        # set true for testing
        self.visualize = True

        # counts how many messages were send
        self.msg_counter = 0
        # publish rate
        self.rate = rospy.Rate(self.PUBLISH_RATE)

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
        hsv_mask_interval = np.matrix([[0, 50, 100], [30, 255, 255]])
        self.__curve_point_detector = CurvePointExtractor(hsv_mask_interval, 10,
                                                          [0.7, 0.35, 0.35], self.visualize)

        if camera_info.D[0] == 0.0:
            self.curve_estimator = CurveEstimator(camera_info, H, False)
        else:
            self.curve_estimator = CurveEstimator(camera_info, H, True)

        # register all publishers
        self.init_publishers()

        # register all publishers
        self.init_subscribers()

        # ros Image has to be bridged to openCV
        self.bridge = CvBridge()

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
        self.publishers["curve_point"] = rospy.Publisher("~curve_point", MsgCurvePoint2D,
                                                         queue_size=1)

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
        curve_points = self.curve_estimator.estimate_curve(np.array(curve_points_image))

        calc_time = time.time() - t0

        if self.visualize:
            self.visualize_data(img_bgr, curve_points_image, curve_points, calc_time)

        # publish world curve points for visualization
        self.publish_world_curve_points(curve_points)

    def visualize_data(self, img_bgr, curve_points_image, curve_points, calc_time):
        # DEBUG: REMOVE
        if len(curve_points_image) != 0:
            for curve_points_same_x_value in curve_points_image:
                for i, curve_point in enumerate(curve_points_same_x_value):
                    cv2.circle(img_bgr, (curve_point[0], curve_point[1]), 5, (50 + 50 * i, 0, 0))

        cv2.putText(img_bgr, str(calc_time), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow("line", img_bgr)
        cv2.waitKey(1)

        # create world plot for every 10th message publish
        if self.msg_counter % 10 == 0:
            plt.cla()
            for curve in curve_points:
                xs = []
                ys = []
                for i, curve_point in enumerate(curve):
                    xs.append(curve_point.x)
                    ys.append(curve_point.y)

                    if i == 1:
                        try:
                            r = curve_point.circle.radius
                            cx = curve_point.circle.center.x
                            cy = curve_point.circle.center.y

                            theta = np.linspace(0, 2 * np.pi, 100)

                            x1 = cx + r * np.cos(theta)
                            x2 = cy + r * np.sin(theta)

                            plt.plot(x1, x2)
                        except AttributeError:
                            # point has no circle - do nothing
                            pass

                plt.plot(xs, ys, ':')

            plt.draw()
            plt.axis("equal")
            plt.grid(color='gray', linestyle='-', linewidth=1)
            plt.ylim((-1, 1))
            plt.xlim(0, 3)
            plt.pause(0.00000000001)

    def publish_world_curve_points(self, curve_points):

        DEFAULT_FALSE_VALUE = float("inf")

        self.msg_counter += 1
        curve_point_msg = MsgCurvePoint2D()

        try:
            curve_point = curve_points[0][0]

            try:
                # point on curve
                curve_point_msg.x = curve_point.x
                curve_point_msg.y = curve_point.y
            except AttributeError:
                # point on curve
                curve_point_msg.x = DEFAULT_FALSE_VALUE
                curve_point_msg.y = DEFAULT_FALSE_VALUE

            try:
                # slope of point
                curve_point_msg.slope = curve_point.slope
            except AttributeError:
                curve_point_msg.slope = DEFAULT_FALSE_VALUE

            try:
                # circle at point
                curve_point_msg.cR = curve_point.circle.radius
                curve_point_msg.cx = curve_point.circle.center.x
                curve_point_msg.cy = curve_point.circle.center.y
            except AttributeError:
                curve_point_msg.cR = DEFAULT_FALSE_VALUE
                curve_point_msg.cx = DEFAULT_FALSE_VALUE
                curve_point_msg.cy = DEFAULT_FALSE_VALUE
        except IndexError:
            # point on curve
            curve_point_msg.x = DEFAULT_FALSE_VALUE
            curve_point_msg.y = DEFAULT_FALSE_VALUE
            curve_point_msg.slope = DEFAULT_FALSE_VALUE
            curve_point_msg.cR = DEFAULT_FALSE_VALUE
            curve_point_msg.cx = DEFAULT_FALSE_VALUE
            curve_point_msg.cy = DEFAULT_FALSE_VALUE

        self.publishers["curve_point"].publish(curve_point_msg)
        # publish message with given frequency
        self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("world_projection_node")
    CurveDetector()
    # plt.ion()
    # plt.show()
    rospy.spin()
