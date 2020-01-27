#!/usr/bin/env python
import rospy
from cv_bridge import CvBridgeError, CvBridge
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image
import line_detection.line_detection as line_detection
from picar_common.picar_common import get_param, get_config_file_path


class LineDetectionNode(object):
    def __init__(self):

        config_file_name = get_param("~config_file_name", "default")

        config_file_path = get_config_file_path("line_detection",
                                                config_file_name)

        if config_file_path is None:
            rospy.signal_shutdown("Could not find any config file... "
                                  "Shutting down!")

        self.line_detector = line_detection.LineGetter(config_file_path)

        self.sub_img = rospy.Subscriber("~input_image/raw",
                                        Image,
                                        self.rcv_img_cb,
                                        queue_size=1)
        self.pub_position = rospy.Publisher("~track_position",
                                            Point32,
                                            queue_size=1)

        self.bridge = CvBridge()

    def rcv_img_cb(self, msg_out):
        img_bgr = self.bridge.imgmsg_to_cv2(msg_out)
        position = self.line_detector.process_image(img_bgr)

        if position is None:
            return

        msg_out = Point32()
        msg_out.x = position[0]
        msg_out.y = position[1]
        self.pub_position.publish(msg_out)


if __name__ == "__main__":
    rospy.init_node("line_detection_node")
    node = LineDetectionNode()
    rospy.spin()
