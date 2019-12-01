#!/usr/bin/env python
import rospy
import yaml
from cv_bridge import CvBridgeError, CvBridge
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
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
        """ initialization of ROS services to configure parameters of leader detection during runtime"""
        self.services["set_123_TEST"] = rospy.Service("~set_123_TEST", SetValue, self.set_123_TEST)

    def set_123_TEST(self, request):
        """ this is a test service callback function"""
        print request.value
        return 1

    def init_publishers(self):
        """ initialize ROS publishers and stores them in a dictionary"""
        # relative position of leader to car
        self.publishers["leader_position"] = rospy.Publisher("~leader_position", Point32, queue_size=1)

        # relative z-orientation of leader to car
        self.publishers["leader_orientation"] = rospy.Publisher("~leader_orientation", Float32, queue_size=1)

    def rcv_img_cb(self, image_data):
        """
            receive image callback: process image and publish position and orientation
            :param image_data: form camera
            """
        img_bgr = self.bridge.imgmsg_to_cv2(image_data)

        position, orientation = self.leader_detector.process_image(img_bgr)

        if position is None or orientation is None:
            return

        # format output msgs
        msg_out_pos = Point32()
        msg_out_pos.x = position[0]
        msg_out_pos.y = position[1]

        msg_out_orientation = orientation

        # publish position and orientation of leader
        self.publishers["leader_position"].publish(msg_out_pos)
        self.publishers["leader_orientation"].publish(msg_out_orientation)


if __name__ == "__main__":
    rospy.init_node('leader_detection_node')
    node = LeaderDetectionNode()
    rospy.spin()
