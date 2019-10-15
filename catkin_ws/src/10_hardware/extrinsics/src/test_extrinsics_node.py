#!/usr/bin/env python
import sys
import os
import yaml
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QIcon, QPixmap, QImage, QPainter
from PyQt5.QtCore import *
import rospy
import rospkg
from sensor_msgs.msg import CompressedImage, CameraInfo
from image_geometry import PinholeCameraModel
import numpy as np
import cv2
import cv_bridge
import ground_projection
from picar_common import get_config_file_path, get_param, FisheyeCameraModel

class App(QWidget):
    def __init__(self):
        super(App, self).__init__()
        self.title = 'Extrinsics Test'
        self.projector = None

        self.init_ros()
        self.init_ui()




    def init_ros(self):
        rospy.init_node("test_extrinsics_node")
        name = get_param("~extrinsics_file_name", "default")
        path = get_config_file_path("extrinsics", name)

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

        if camera_info.D[0] == 0.0:
            self.projector = ground_projection.GroundProjector(camera_info,
                                                               H,
                                                               False)
        else:
            self.projector = ground_projection.GroundProjector(camera_info,
                                                               H,
                                                               True)

    def init_ui(self):
        self.setWindowTitle(self.title)

        image_widget = ImageWidget(self.projector)
        layout = QHBoxLayout()
        layout.addWidget(image_widget)
        self.setLayout(layout)


class ImageWidget(QWidget):
    def __init__(self, projector):
        """

        :type projector: ground_projection.GroundProjector
        """
        super(ImageWidget, self).__init__()
        self.projector = projector
        self.image = QImage()
        self.canvas = QLabel()
        self.x_pixel_label = QLabel("x: {:4d}".format(0))
        self.y_pixel_label = QLabel("y: {:4d}".format(0))
        self.x_car_label = QLabel("x: {:.3f}".format(0.0))
        self.y_car_label = QLabel("y: {:.3f}".format(0.0))
        self.canvas.mousePressEvent = self.on_canvas_click
        self.bridge = cv_bridge.CvBridge()
        self.get_image_button = QPushButton("Update Image")
        self.get_image_button.clicked.connect(self.on_get_image_button)

        side_pane = QWidget()
        side_pane_layout = QVBoxLayout()

        pixel_group = QGroupBox("Pixel Coordinates")
        pixel_layout = QHBoxLayout()
        pixel_layout.addWidget(self.x_pixel_label)
        pixel_layout.addWidget(self.y_pixel_label)
        pixel_group.setLayout(pixel_layout)
        side_pane_layout.addWidget(pixel_group)

        car_group = QGroupBox("Car Coordinates")
        car_layout = QHBoxLayout()
        car_layout.addWidget(self.x_car_label)
        car_layout.addWidget(self.y_car_label)
        car_group.setLayout(car_layout)
        side_pane_layout.addWidget(car_group)

        side_pane_layout.addWidget(self.get_image_button)
        side_pane_layout.setAlignment(Qt.AlignTop)
        side_pane.setLayout(side_pane_layout)

        layout = QHBoxLayout()
        layout.addWidget(self.canvas)
        layout.addWidget(side_pane)
        self.setLayout(layout)

        self.on_get_image_button()


    def get_cv_image(self):
        try:
            image_compressed = rospy.wait_for_message(
                "camera_node/image/compressed",
                CompressedImage,
                5.0)
        except rospy.ROSException:
            rospy.logwarn("Could not get image from camera. "
                          "Are you sure it's launched?")
            return None

        image_cv = self.bridge.compressed_imgmsg_to_cv2(image_compressed)
        return image_cv

    def get_qt_image(self, image_cv):
        height, width = image_cv.shape[0:2]
        bytes_per_line = 3 * width
        qt_image = QImage(image_cv.data, width, height,
                          bytes_per_line, QImage.Format_RGB888)
        qt_image = qt_image.rgbSwapped()
        return qt_image

    def on_get_image_button(self):
        image = self.get_cv_image()
        if image is not None:
            self.image = self.get_qt_image(image)
            self.canvas.setPixmap(QPixmap.fromImage(self.image))
            self.update()

    def on_canvas_click(self, event):
        x = event.pos().x()
        y = event.pos().y()
        self.x_pixel_label.setText("x: {}".format(x))
        self.y_pixel_label.setText("y: {}".format(y))

        pixel = np.array([x, y])
        point = self.projector.pixel2ground(pixel)
        self.x_car_label.setText("x: {:.3f}".format(point.x))
        self.y_car_label.setText("y: {:.3f}".format(point.y))


def main():
    app = QApplication([])

    main_window = QMainWindow()
    main_widget = App()
    main_window.setCentralWidget(main_widget)
    icon_path = os.path.join(rospkg.RosPack().get_path("extrinsics"),
                             "images/icon.png")
    main_window.setWindowIcon(QIcon(icon_path))
    main_window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
