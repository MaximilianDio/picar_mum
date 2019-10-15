#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv_bridge
from sensor_msgs.msg import CameraInfo
from picar_common.picar_common import get_param
import intrinsics.intrinsics_gui as intrinsics_gui
from intrinsics.intrinsics import Calibrator

class Node(object):
    def __init__(self, gui):
        """

        :type gui: intrinsics_gui.MainWidget
        """
        size_parameter = get_param("~size", "7x4")
        self.chessboard_size = self.parse_chessboard_size(size_parameter)

        square_parameter = get_param("~square", "0.04")
        self.square_size = float(square_parameter)

        self.bridge = cv_bridge.CvBridge()

        fisheye = self.is_fisheye()

        if fisheye:
            rospy.loginfo("[{}] using fisheye model".format(rospy.get_name()))
        else:
            rospy.loginfo("[{}] using pinhole model".format(rospy.get_name()))

        self.calibrator = Calibrator(self.chessboard_size,
                                     self.square_size,
                                     fisheye)
        self.gui = gui
        self.gui.set_calibrator(self.calibrator)

        self.image_subscriber = rospy.Subscriber("~image/raw",
                                                 Image,
                                                 self.image_callback,
                                                 queue_size=1)

    def is_fisheye(self):
        is_fisheye = False
        rospy.loginfo("[{}] Waiting for camera_info message..."
                      "".format(rospy.get_name()))

        camera_info = rospy.wait_for_message("camera_node/camera_info",
                                             CameraInfo)

        rospy.loginfo("[{}] Received camera_info message."
                      "".format(rospy.get_name()))
        if camera_info.distortion_model == "fisheye":
            is_fisheye = True
        return is_fisheye

    def parse_chessboard_size(self, size_string):
        size = size_string.split("x")
        size = [int(number_string) for number_string in size]
        return tuple(size)

    def image_callback(self, message):
        image = self.bridge.imgmsg_to_cv2(message)
        result = self.calibrator.process_image(image)
        try:
            self.gui.canvas.new_image.emit(result.image)
        except AttributeError as error:
            pass

        n_images = len(self.calibrator.database)
        if n_images > 0:
            try:
                self.gui.side_pane.update_params.emit(
                    result.parameters,
                    n_images,
                    self.calibrator.max_images
                )
            except AttributeError as error:
                pass
        if self.calibrator.good_enough:
            try:
                self.gui.side_pane.calibration_ready.emit()
            except AttributeError as error:
                pass




def main():
    rospy.init_node("calibrate_intrinsics_node")
    app = intrinsics_gui.App()
    node = Node(app.root)
    app.exec_()


if __name__ == "__main__":
    main()
