#!/usr/bin/env python
import rospy
import color_normalization.color_normalization as color_normalization
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from picar_common.picar_common import get_param, get_config_file_path


class NormalizerNode(object):
    def __init__(self):
        config_file_name = get_param("~config_file_name", "default")

        config_file_path = get_config_file_path("color_normalization",
                                                config_file_name)

        if config_file_path is None:
            rospy.signal_shutdown("Could not find any config file... "
                                  "Shutting down!")

        self.bridge = CvBridge()
        self.normalizer = color_normalization.Normalizer(config_file_path)

        self.img_sub = rospy.Subscriber("~input_image/compressed",
                                        CompressedImage,
                                        self.rcv_img_cb,
                                        queue_size=1)
        self.img_pub = rospy.Publisher("~output_image/raw",
                                       Image,
                                       queue_size=1)

    def rcv_img_cb(self, msg):
        try:
            img_bgr = self.bridge.compressed_imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # normalize image
        img_normalized = self.normalizer.process_image(img_bgr)

        # prepare message
        msg = self.bridge.cv2_to_imgmsg(img_normalized)

        # publish message
        try:
            self.img_pub.publish(msg)
        except:
            pass


if __name__ == "__main__":
    rospy.init_node("color_normalization_node")
    node = NormalizerNode()
    rospy.spin()
