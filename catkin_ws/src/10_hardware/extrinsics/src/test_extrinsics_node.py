#!/usr/bin/env python
import sys
import os
import yaml
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QIcon, QPixmap, QImage
from PyQt5.QtCore import *
import rospy
import rospkg
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Point32
import numpy as np
import cv_bridge
from image_geometry import PinholeCameraModel
from picar_common.picar_common import get_config_file_path, get_param, get_camera_info, FisheyeCameraModel


def normalize_image_coordinates(pixel_coord, image_width, image_height):
    """Computes normalized pixel coordinates.

    Args:
        pixel_coord (list/tuple): Coordinates of the pixel in uv-coordinate
            frame.
        image_width (int): Width of the image in pixels.
        image_height (int): Height of the image in pixels.

    Returns (numpy.ndarray): Numpy array of normalized pixel coordinates.

    """
    x_normalized = float(pixel_coord[0]) / (image_width - 1)
    y_normalized = float(pixel_coord[1]) / (image_height - 1)
    return np.array([x_normalized, y_normalized])


class Projector(object):
    def __init__(self, camera_info, h_matrix, distorted_input=True):
        self.distorted_input = distorted_input
        self.camera_info = camera_info
        self.fisheye = False
        if camera_info.distortion_model == "fisheye":
            self.camera = FisheyeCameraModel()
            self.camera.from_camera_info(camera_info)
            self.fisheye = True
        else:
            self.camera = PinholeCameraModel()
            self.camera.fromCameraInfo(camera_info)
        self.h_matrix = h_matrix

    def pixel2ground(self, pixel_coord, distorted_input=None):
        """Returns the ground projection of a pixel."""
        # use default value if 'distorted_input' is not set
        if distorted_input is None:
            distorted_input = self.distorted_input
        if distorted_input:
            pixel_coord = self.rectify_pixel(pixel_coord)
        pixel_coord = np.array(pixel_coord)

        # normalize coordinates
        pixel_normalized = normalize_image_coordinates(
            pixel_coord,
            self.camera.width,
            self.camera.height)

        point = self._project_normalized_pixel(pixel_normalized)
        return point

    def _project_normalized_pixel(self, pixel_normalized):
        pixel_coord = np.append(pixel_normalized, 1.0)
        ground_coord = np.dot(self.h_matrix, pixel_coord)
        # scale the coordinates appropriately
        ground_coord = ground_coord / ground_coord[2]

        point = Point32()
        point.x = ground_coord[0]
        point.y = ground_coord[1]
        point.z = 0.0

        return point

    def rectify_pixel(self, pixel_coord):
        """Calculates the rectified undistorted image coordinate of a pixel.

        Args:
            pixel_coord: Distorted pixel coordinates

        Returns: Undistorted pixel coordinates
        """
        if self.fisheye:
            return self.camera.undistort_point(pixel_coord)
        else:
            return self.camera.rectifyPoint(pixel_coord)


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

        if camera_info.D[0] == 0.0:
            self.projector = Projector(camera_info, H, False)
        else:
            self.projector = Projector(camera_info, H, True)

    def init_ui(self):
        self.setWindowTitle(self.title)

        image_widget = ImageWidget(self.projector)
        layout = QHBoxLayout()
        layout.addWidget(image_widget)
        self.setLayout(layout)


class ImageWidget(QWidget):
    def __init__(self, projector):
        """

        :type projector: Projector
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
