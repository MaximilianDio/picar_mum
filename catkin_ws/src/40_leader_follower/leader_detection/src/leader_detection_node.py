#!/usr/bin/env python
import rospy
from cv_bridge import CvBridgeError, CvBridge
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import leader_detection as leader_detection
from picar_common.picar_common import get_param, get_config_file_path


class LeaderDetectionNode(object):
    def __init__(self):

        # import parameters from config yaml files
        config_file_name = get_param("~config_file_name", "default")

        print config_file_name

        config_file_path = get_config_file_path("leader_detection",
                                                config_file_name)

        # shut down node if no config file could be found
        if config_file_path is None:
            rospy.signal_shutdown("Could not find any config file... "
                                  "Shutting down!")

        self.leader_detector = leader_detection.LeaderGetter(config_file_path)

        # --------------------------------------------------------------------------------------------------------------
        # ----------------------------------------------- SUBSCRIBER ---------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        # subscribe to image and publish position of leader
        self.sub_img = rospy.Subscriber("~input_image/raw",
                                        Image,
                                        self.rcv_img_cb)

        # --------------------------------------------------------------------------------------------------------------
        # ------------------------------------------------ PUBLISHER ---------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        # relative position of leader to car
        self.pub_position = rospy.Publisher("~leader_position",
                                            Point32,
                                            queue_size=1)

        # relative z-orientation of leader to car
        self.pub_orientation = rospy.Publisher("~leader_orientation",
                                               Float32,
                                               queue_size=1)

        # ros Image has to be bridged to openCV
        self.bridge = CvBridge()

    '''
    receive image callback: process image and publish position and orientation
    :param image_data: form camera
    '''
    def rcv_img_cb(self, image_data):
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
        self.pub_position.publish(msg_out_pos)
        self.pub_orientation.publish(msg_out_orientation)


if __name__ == "__main__":
    rospy.init_node('leader_detection_node')
    node = LeaderDetectionNode()
    rospy.spin()
