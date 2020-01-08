#!/usr/bin/env python
import rospy
from cv_bridge import CvBridgeError, CvBridge
import yaml
import cv2
import numpy as np
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image
from picar_common.picar_common import get_param, get_config_file_path, set_param, get_camera_info
from apriltag_detection import AprilTagDetector


class ApriltagDetectionNode:

    def __init__(self, isDebug):
        self.DEBUG = isDebug

        # TODO remove if TAGS exist in simulation
        if self.DEBUG:
            # start webcam capture
            self.cap = cv2.VideoCapture(0)

        # ros Image has to be bridged to openCV
        self.bridge = CvBridge()

        self._params = {}
        self.publishers = {}
        self.services = {}

        # read the cofig parameters form .yaml file
        self.setup_params()

        # set publish rate of node
        self.rate = rospy.Rate(self._params["Rate"])

        # register all publishers
        self.init_publishers()
        # create all services
        self.init_services()

        # subscribe to image and publish position of leader
        self.sub_img = rospy.Subscriber("~input_image/raw", Image, self.rcv_img_cb)

        self.init_apriltags()

    def setup_params(self):

        # import parameters from config yaml files
        config_file_name = get_param("~config_file_name", "default")

        config_file_path = get_config_file_path("apriltag_detection",
                                                config_file_name)

        # shut down node if no config file could be found
        if config_file_path is None:
            rospy.signal_shutdown("Could not find any config file... "
                                  "Shutting down!")

        with open(config_file_path, "r") as file_handle:
            data = yaml.safe_load(file_handle)
        self._params = data
        for param_name in self._params:
            set_param("~" + param_name, self._params[param_name])

        # get camera data - intrinsics

        # distorted_input is set to False only in simulation mode
        distorted_input = get_param("~distorted_input", "False")
        camera_info = get_camera_info()
        self._params["dist"] = np.array(camera_info.D)
        self._params["mtx"] = np.array(camera_info.K).reshape([3, 3])

        # extrinsics - tvec and rmat from car to camera
        extrinsics_file_name = rospy.get_namespace().strip("/")
        extrinsics_file_path = get_config_file_path("extrinsics", extrinsics_file_name)

        with open(extrinsics_file_path, "r") as file_handle:
            data = yaml.safe_load(file_handle)
        self._params["tvec_cam"] = np.array(data["t"]["data"]).reshape([3, 1])
        self._params["rmat_cam"] = np.array(data["S"]["data"]).reshape([3, 3])

    def init_services(self):
        """Initialize ROS services to configure the controller during runtime"""
        pass

    def init_publishers(self):
        """ initialize ROS publishers and stores them in a dictionary"""
        # relative position of leaders blue ball to picar
        self.publishers["apriltag_position"] = rospy.Publisher("~apriltag_position", Point32,
                                                               queue_size=1)

    def init_apriltags(self):
        # Init used apriltags
        tag_ID = self._params["TagID"]
        tag_size = self._params["TagSize"]

        # translation vector from reference coordinate system
        tag_tvec = np.array(self._params["Tagtvec"]).reshape([3, 1])
        # rotation matrix from reference coordination system
        tag_rmat = np.array(self._params["Tagrmat"]).reshape([3, 3])

        dist = self._params["dist"]
        mtx = self._params["mtx"]
        tvec_cam = self._params["tvec_cam"]
        rmat_cam = self._params["rmat_cam"]

        self.tag_detector = AprilTagDetector(tag_ID, tag_size, tag_tvec, tag_rmat, mtx, dist, tvec_cam, rmat_cam)

    # MAIN CALLBACK
    def rcv_img_cb(self, image_data):
        tag_position = Point32()

        # TODO remove if TAGS exist in simulation
        if self.DEBUG:
            # Capture frame-by-frame
            ret, img_bgr = self.cap.read()
        else:
            img_bgr = self.bridge.imgmsg_to_cv2(image_data)
        # convert image to gray scale
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        tvec = self.tag_detector.get_tag_pos(gray)

        if self.DEBUG:
            self.tag_detector.draw_tag(img_bgr)
            # Display the resulting frame
        cv2.imshow('frame', img_bgr)
        cv2.waitKey(1)

        if tvec is not None:
            tag_position.x = tvec[0][0]
            tag_position.y = tvec[1][0]
            tag_position.z = tvec[2][0]
        else:
            tag_position.x = float("inf")
            tag_position.y = float("inf")
            tag_position.z = float("inf")

        rospy.logdebug("position of Apriltag: x: %f, y: %f z: %f",
                       tag_position.x, tag_position.y, tag_position.z)

        self.publishers["apriltag_position"].publish(tag_position)

        self.rate.sleep()


if __name__ == "__main__":
    DEBUG = True
    node_name = 'april_detection_node'
    if DEBUG:
        rospy.init_node(node_name, log_level=rospy.DEBUG)
    else:
        rospy.init_node(node_name)

    node = ApriltagDetectionNode(DEBUG)
    rospy.spin()
