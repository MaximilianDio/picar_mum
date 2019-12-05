#!/usr/bin/env python
import rospy
import yaml
import cv2
from cv_bridge import CvBridgeError, CvBridge
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image
from picar_msgs.srv import SetValue
import leader_detection as leader_detection
from picar_common.picar_common import get_param, get_config_file_path, set_param


class LeaderDetectionNode(object):
    def __init__(self):
        self._params = {}
        self.publishers = {}
        self.services = {}

        # import parameters from config yaml files
        config_file_name = get_param("~config_file_name", "default")

        config_file_path = get_config_file_path("leader_detection",
                                                config_file_name)

        # shut down node if no config file could be found
        if config_file_path is None:
            rospy.signal_shutdown("Could not find any config file... "
                                  "Shutting down!")

        # read the cofig parameters form .yaml file
        self.setup_params(config_file_path)

        # create leader detection object
        self.leader_detector = leader_detection.LeaderGetter(self._params)

        # subscribe to image and publish position of leader
        self.sub_img = rospy.Subscriber("~input_image/raw", Image, self.rcv_img_cb)

        # register all publishers
        self.init_publishers()
        # create all services
        self.init_services()

        # ros Image has to be bridged to openCV
        self.bridge = CvBridge()

    def setup_params(self, config_file_path):
        with open(config_file_path, "r") as file_handle:
            data = yaml.safe_load(file_handle)
        self._params = data
        for param_name in self._params:
            set_param("~" + param_name, self._params[param_name])

    def init_services(self):
        """Initialize ROS services to configure the controller during runtime"""

        self.services["set_mask_param_blue_low_H"] = rospy.Service(
            "~set_mask_param_blue_low_H",
            SetValue,
            self.set_mask_param_blue_low_H)

        self.services["set_mask_param_blue_low_S"] = rospy.Service(
            "~set_mask_param_blue_low_S",
            SetValue,
            self.set_mask_param_blue_low_S)

        self.services["set_mask_param_blue_low_V"] = rospy.Service(
            "~set_mask_param_blue_low_V",
            SetValue,
            self.set_mask_param_blue_low_V)

        self.services["set_mask_param_blue_high_H"] = rospy.Service(
            "~set_mask_param_blue_high_H",
            SetValue,
            self.set_mask_param_blue_high_H)

        self.services["set_mask_param_blue_high_S"] = rospy.Service(
            "~set_mask_param_blue_high_S",
            SetValue,
            self.set_mask_param_blue_high_S)

        self.services["set_mask_param_blue_high_V"] = rospy.Service(
            "~set_mask_param_blue_high_V",
            SetValue,
            self.set_mask_param_blue_high_V)

        self.services["set_mask_param_green_low_H"] = rospy.Service(
            "~set_mask_param_green_low_H",
            SetValue,
            self.set_mask_param_green_low_H)

        self.services["set_mask_param_green_low_S"] = rospy.Service(
            "~set_mask_param_green_low_S",
            SetValue,
            self.set_mask_param_green_low_S)

        self.services["set_mask_param_green_low_V"] = rospy.Service(
            "~set_mask_param_green_low_V",
            SetValue,
            self.set_mask_param_green_low_V)

        self.services["set_mask_param_green_high_H"] = rospy.Service(
            "~set_mask_param_green_high_H",
            SetValue,
            self.set_mask_param_green_high_H)

        self.services["set_mask_param_green_high_S"] = rospy.Service(
            "~set_mask_param_green_high_S",
            SetValue,
            self.set_mask_param_green_high_S)

        self.services["set_mask_param_green_high_V"] = rospy.Service(
            "~set_mask_param_green_high_V",
            SetValue,
            self.set_mask_param_green_high_V)


    def init_publishers(self):
        """ initialize ROS publishers and stores them in a dictionary"""
        # relative position of leaders blue ball to picar
        self.publishers["leader_blue_ball_position"] = rospy.Publisher("~leader_blue_ball_position", Point32,
                                                                       queue_size=1)

        # relative position of leaders green ball to picar
        self.publishers["leader_green_ball_position"] = rospy.Publisher("~leader_green_ball_position", Point32,
                                                                        queue_size=1)

        # Publish (mask_green+mask_blue) for debugging
        self.publishers["mask_added"] = rospy.Publisher("~mask_added", Image, queue_size=1)

        # # relative position of leaders green ball to picar
        # self.publishers["leader_detection_image"] = rospy.Publisher("~leader_green_ball_position", Image,
        #                                                                 queue_size=1)

    def rcv_img_cb(self, image_data):
        """
            receive image callback: process image and publish positions of blue and green ball
            :param image_data: form camera
            """
        img_bgr = self.bridge.imgmsg_to_cv2(image_data)

        (positions, output_img), mask_img = self.leader_detector.process_image(img_bgr)

        if positions is None:
            return

        # format output msgs
        msg_out_blue_ball_pos = Point32()
        msg_out_blue_ball_pos.x = positions[0]
        msg_out_blue_ball_pos.y = positions[1]

        msg_out_green_ball_pos = Point32()
        msg_out_green_ball_pos.x = positions[2]
        msg_out_green_ball_pos.y = positions[3]

        # publish position and orientation of leader
        self.publishers["leader_blue_ball_position"].publish(msg_out_blue_ball_pos)
        self.publishers["leader_green_ball_position"].publish(msg_out_green_ball_pos)

        # Publish mask

        image_mask_msg = self.bridge.cv2_to_imgmsg(mask_img, encoding="passthrough")
        self.publishers["mask_added"].publish(image_mask_msg)

        # output image
        cv2.imshow('mask',mask_img)
        cv2.imshow("leader_detection",output_img)
        cv2.waitKey(1)
        # self.publishers["leader_detection_image"].publish(output_img)

    def set_mask_param_blue_low_H(self, request):
        """Sets the mask_param_blue_low_H of leadergetter"""
        self._params["mask_param_blue_low_H"] = request.value


        set_param("~mask_param_blue_low_H", request.value)

        self.update_leader_detector()
        return 1

    def set_mask_param_blue_low_S(self, request):
        """Sets the mask_param_blue_low_S of leadergetter"""
        self._params["mask_param_blue_low_S"] = request.value
        set_param("~mask_param_blue_low_S", request.value)
        self.update_leader_detector()
        return 1

    def set_mask_param_blue_low_V(self, request):
        """Sets the mask_param_blue_low_Vof leadergetter"""
        self._params["mask_param_blue_low_V"] = request.value
        set_param("~mask_param_blue_low_V", request.value)
        self.update_leader_detector()
        return 1

    def set_mask_param_blue_high_H(self, request):
        """Sets the mask_param_blue_low_H of leadergetter"""
        self._params["mask_param_blue_high_H"] = request.value
        set_param("~mask_param_blue_high_H", request.value)
        self.update_leader_detector()
        return 1

    def set_mask_param_blue_high_S(self, request):
        """Sets the mask_param_blue_low_S of leadergetter"""
        self._params["mask_param_blue_high_S"] = request.value
        set_param("~mask_param_blue_high_S", request.value)
        self.update_leader_detector()
        return 1

    def set_mask_param_blue_high_V(self, request):
        """Sets the mask_param_blue_low_Vof leadergetter"""
        self._params["mask_param_blue_high_V"] = request.value
        set_param("~mask_param_blue_high_V", request.value)
        self.update_leader_detector()
        return 1

    def set_mask_param_green_low_H(self, request):
        """Sets the mask_param_blue_low_H of leadergetter"""
        self._params["mask_param_green_low_H"] = request.value
        set_param("~mask_param_green_low_H", request.value)
        self.update_leader_detector()
        return 1

    def set_mask_param_green_low_S(self, request):
        """Sets the mask_param_blue_low_S of leadergetter"""
        self._params["mask_param_green_low_S"] = request.value
        set_param("~mask_param_green_low_S", request.value)
        self.update_leader_detector()
        return 1

    def set_mask_param_green_low_V(self, request):
        """Sets the mask_param_blue_low_Vof leadergetter"""
        self._params["mask_param_green_low_V"] = request.value
        set_param("~mask_param_green_low_V", request.value)
        self.update_leader_detector()
        return 1

    def set_mask_param_green_high_H(self, request):
        """Sets the mask_param_blue_low_H of leadergetter"""
        self._params["mask_param_green_high_H"] = request.value
        set_param("~mask_param_green_high_H", request.value)
        self.update_leader_detector()
        return 1

    def set_mask_param_green_high_S(self, request):
        """Sets the mask_param_green_low_S of leadergetter"""
        self._params["mask_param_green_high_S"] = request.value
        set_param("~mask_param_green_high_S", request.value)
        self.update_leader_detector()
        return 1

    def set_mask_param_green_high_V(self, request):
        """Sets the mask_param_green_low_Vof leadergetter"""
        self._params["mask_param_green_high_V"] = request.value
        set_param("~mask_param_green_high_V", request.value)
        self.update_leader_detector()
        return 1


    def update_leader_detector(self):
        """Updates the controller object's picar"""
        self.leader_detector.update_params(
            self._params
        )


if __name__ == "__main__":
    rospy.init_node('leader_detection_node')
    node = LeaderDetectionNode()
    rospy.spin()
