#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError


class Decompressor(object):
    def __init__(self):
        self.sub = rospy.Subscriber("camera/image/compressed", CompressedImage, self.img_cb)
        self.pub = rospy.Publisher("camera/image/raw", Image, queue_size=1)

        self.bridge = CvBridge()

    def img_cb(self, msg):
        try:
            cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        except CvBridgeError, e:
            print(e)
        else:
            try:
                self.pub.publish(self.bridge.cv2_to_imgmsg(cv2_img, "bgr8"))
            except CvBridgeError, e:
                print(e)

    def on_shutdown_cb(self):
        rospy.loginfo("Shutting down {}".format(rospy.get_name()))


def main():
    rospy.init_node('decompress_node')
    dc = Decompressor()
    rospy.on_shutdown(dc.on_shutdown_cb)
    rospy.spin()
        

if __name__ == "__main__":
    main()
