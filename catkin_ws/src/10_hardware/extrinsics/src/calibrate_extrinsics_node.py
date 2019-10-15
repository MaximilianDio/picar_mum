#!/usr/bin/env python
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospkg
import os
import yaml
import extrinsics
import picar_common
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel


class Node(QWidget):
    def __init__(self):
        super(Node, self).__init__()
        self.bridge = None

        camera_info = self.init_ros()
        self.calibrator = extrinsics.Calibrator((7, 4), 0.04, camera_info)

        layout = QHBoxLayout()
        self.canvas = Canvas()
        self.side_pane = SidePane()
        self.side_pane.update_image_request.connect(self.update_image)
        self.side_pane.calibrate_request.connect(self.calibrate)
        layout.addWidget(self.canvas)
        layout.addWidget(self.side_pane)
        self.setLayout(layout)

        self.update_image()

        policy = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        policy.setHeightForWidth(True)
        self.setSizePolicy(policy)

        rospy.loginfo("[{}] Started.".format(rospy.get_name()))

    def init_ros(self):
        rospy.init_node("calibrate_extrinsics_node")
        camera_info = picar_common.get_camera_info()
        self.bridge = CvBridge()
        return camera_info

    def update_image(self):
        message = rospy.wait_for_message(
            "camera_node/image/compressed",
            CompressedImage)
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(message)
        except CvBridgeError as e:
            rospy.logwarn("[{}] {}".format(rospy.get_name(), e))
            return
        success, corners = self.calibrator.get_corners(cv_image)
        image_corners = self.calibrator.draw_corners(np.copy(cv_image),
                                                     success,
                                                     corners)
        if success:
            self.side_pane.calibrate_button.setDisabled(False)
        else:
            self.side_pane.calibrate_button.setDisabled(True)
        self.canvas.update_request.emit(cv_image, image_corners)

    def calibrate(self):
        H, R, t, angles = self.calibrator.calibrate(self.canvas.original_image)
        self.side_pane.update_calibration.emit(H, R, t, angles)

    def image_callback(self, msg):
        try:
            cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        except CvBridgeError, e:
            rospy.logwarn("{}".format(e))
            return

        self.success = self.calibrator.calculate_extrinsics(cv2_img,
                                                            (7, 4),
                                                            0.04)
        if self.success:
            self.save_calibration()
            rospy.signal_shutdown("Successfully completed extrinsic "
                                  "calibration.")

    def save_calibration(self):
        # ros namespace is always the same as the vehicle name
        veh = rospy.get_namespace()
        veh = veh.strip("/")
        extrinsics_file_path = os.path.join(rospkg.RosPack().get_path("picar"),
                                            "config",
                                            "extrinsics",
                                            veh + ".yaml")
        with open(extrinsics_file_path, "w") as f:
            yaml.dump(self.success, f)


class SidePane(QWidget):
    update_image_request = pyqtSignal()
    calibrate_request = pyqtSignal()
    update_calibration = pyqtSignal(np.ndarray, np.ndarray, np.ndarray, np.ndarray)
    def __init__(self):
        super(SidePane, self).__init__()
        self.update_button = QPushButton("Update Image")
        self.update_button.clicked.connect(self.on_update_button)

        self.calibrate_button = QPushButton("Calibrate")
        self.calibrate_button.clicked.connect(self.on_calibrate_button)

        self.homography_box = MatrixBox("Homography")
        self.homography_box.set_data(np.full([3, 3], np.inf))

        self.rotation_box = MatrixBox("Rotation")
        self.rotation_box.set_data(np.full([3, 3], np.inf))

        self.translation_box = MatrixBox("Translation")
        self.translation_box.set_data(np.full([3, 1], np.inf))

        self.angle_box = MatrixBox("Roll/Pitch/Yaw")
        self.angle_box.set_data(np.full([3, 1], np.inf))

        self.save_button = QPushButton("Save")
        self.save_button.setDisabled(True)
        self.save_button.clicked.connect(self.on_save_button)

        self.update_calibration.connect(self.on_update_calibration)

        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignTop)
        layout.addWidget(self.update_button)
        layout.addWidget(self.calibrate_button)
        layout.addWidget(self.homography_box)
        layout.addWidget(self.rotation_box)
        hbox = QHBoxLayout()
        hbox.addWidget(self.translation_box)
        hbox.addWidget(self.angle_box)
        layout.addLayout(hbox)
        layout.addWidget(self.save_button)
        self.setLayout(layout)
        self.setSizePolicy(QSizePolicy.Minimum+QSizePolicy.MinimumExpanding,
                           QSizePolicy.Minimum)

    def on_update_button(self):
        self.update_image_request.emit()

    def on_calibrate_button(self):
        self.calibrate_request.emit()

    def on_save_button(self):
        startup_path = os.path.join(
            rospkg.RosPack().get_path("picar"),
            "config",
            "extrinsics")

        veh_name = "".join([rospy.get_namespace().strip("/"), ".yaml"])
        startup_path = os.path.join(startup_path, veh_name)
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Calibration Data",
            startup_path,
            "(*.yaml)",
            "(*.yaml")
        if file_path:
            file_path = file_path.strip(".yaml")
            file_path = "".join([file_path, ".yaml"])
            H, S, t = self.get_calibration_data()
            with open(file_path, "w") as file_handle:
                data = dict(
                    H=dict(
                        rows= H.shape[0],
                        cols = H.shape[1],
                        data=H.ravel().tolist(),
                    ),
                    S=dict(
                        rows=S.shape[0],
                        cols=S.shape[1],
                        data=S.ravel().tolist(),
                    ),
                    t=dict(
                        rows=t.shape[0],
                        cols=t.shape[1],
                        data=t.ravel().tolist(),
                    ),
                )
                yaml.safe_dump(data, file_handle)

    def get_calibration_data(self):
        H = self.homography_box.get_data()
        S = self.rotation_box.get_data()
        t = self.translation_box.get_data()
        return H, S, t


    def on_update_calibration(self, H, R, t, angles):
        self.save_button.setDisabled(False)
        self.homography_box.set_data(H)
        self.rotation_box.set_data(R)
        self.translation_box.set_data(t)
        self.angle_box.set_data(angles)


class MatrixBox(QGroupBox):
    def __init__(self, name):
        super(MatrixBox, self).__init__()
        layout = QHBoxLayout()
        self.setTitle(name)
        self.matrix = Matrix((3, 3))
        layout.addWidget(self.matrix)
        self.setLayout(layout)

    def set_data(self, ndarray):
        self.matrix.set_data(ndarray)

    def get_data(self):
        return self.matrix.get_data()


class Matrix(QTableWidget):
    def __init__(self, (rows, columns)):
        super(Matrix, self).__init__()
        self.setRowCount(rows)
        self.setColumnCount(columns)
        self.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.data = None


    def set_data(self, ndarray):
        self.data = ndarray
        rows, columns = ndarray.shape
        self.setRowCount(rows)
        self.setColumnCount(columns)
        for row in range(rows):
            for col in range(columns):
                item = QTableWidgetItem("{:01.3f}"
                                        "".format(ndarray[row, col]))
                item.setFlags(item.flags()
                              ^ (Qt.ItemIsEditable+Qt.ItemIsSelectable))
                self.setItem(row, col, item)
        self.resizeColumnsToContents()
        self.setFixedSize(
            self.horizontalHeader().length()+self.verticalHeader().width(),
            self.verticalHeader().length()+self.horizontalHeader().height())

    def get_data(self):
        return self.data


class Canvas(QLabel):
    update_request = pyqtSignal(np.ndarray, np.ndarray)
    def __init__(self):
        super(Canvas, self).__init__()
        # self.setFixedSize(QSize(640, 480))
        self.update_request.connect(self.set_image)
        self.setMinimumWidth(320)
        self.setMinimumHeight(240)
        self.original_image = None
        self.pix = None
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)


    def set_image(self, cv_image, image_corners):
        self.original_image = cv_image
        height, width, _ = cv_image.shape
        # cv_image = cv2.resize(cv_image, (self.width(), self.height()))
        height, width, channels = cv_image.shape
        bytes_per_line = channels * width
        qt_image = QImage(image_corners.data, width, height,
                          bytes_per_line, QImage.Format_RGB888)
        qt_image = qt_image.rgbSwapped()
        self.pix = QPixmap.fromImage(qt_image)
        pixmap = QPixmap.fromImage(qt_image).scaled(self.width(),
                                                    self.height(),
                                                    Qt.KeepAspectRatio)
        self.setPixmap(pixmap)

    def resizeEvent(self, QResizeEvent):
        if self.original_image is None:
            return
        self.setPixmap(self.pix.scaled(self.width(), self.height(), Qt.KeepAspectRatio))



def main():
    app = QApplication([])
    node = Node()
    icon_path = os.path.join(rospkg.RosPack().get_path("extrinsics"),
                             "images/icon.png")
    app.setWindowIcon(QIcon(icon_path))
    node.show()
    app.exec_()

if __name__ == "__main__":
    main()
