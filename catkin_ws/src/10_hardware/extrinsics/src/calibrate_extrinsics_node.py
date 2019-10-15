#!/usr/bin/env python

import os
import PyQt5.QtWidgets as QtWidgets
import PyQt5.QtGui as QtGui
import PyQt5.QtCore as QtCore
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospkg
import yaml
import extrinsics.extrinsics as extrinsics
import picar_common.picar_common as picar_common


class Node(QtWidgets.QWidget):
    def __init__(self):
        super(Node, self).__init__()
        self.bridge = None

        camera_info = self.init_ros()
        self.calibrator = extrinsics.Calibrator((7, 4), 0.04, camera_info)

        layout = QtWidgets.QHBoxLayout()
        self.canvas = Canvas()
        self.side_pane = SidePane()
        self.side_pane.update_image_request.connect(self.update_image)
        self.side_pane.calibrate_request.connect(self.calibrate)
        layout.addWidget(self.canvas)
        layout.addWidget(self.side_pane)
        self.setLayout(layout)

        self.update_image()

        policy = QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Preferred,
            QtWidgets.QSizePolicy.Preferred)

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
        except CvBridgeError as error:
            rospy.logwarn("[{}] {}".format(rospy.get_name(), error))
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
        (homography,
         rotation_matrix,
         translation,
         angles) = self.calibrator.calibrate(self.canvas.original_image)

        self.side_pane.update_calibration.emit(
            homography,
            rotation_matrix,
            translation,
            angles)


class SidePane(QtWidgets.QWidget):
    update_image_request = QtCore.pyqtSignal()
    calibrate_request = QtCore.pyqtSignal()
    update_calibration = QtCore.pyqtSignal(
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray)

    def __init__(self):
        super(SidePane, self).__init__()
        self.update_button = QtWidgets.QPushButton("Update Image")
        self.update_button.clicked.connect(self.on_update_button)

        self.calibrate_button = QtWidgets.QPushButton("Calibrate")
        self.calibrate_button.clicked.connect(self.on_calibrate_button)

        self.homography_box = MatrixBox("Homography")
        self.homography_box.set_data(np.full([3, 3], np.inf))

        self.rotation_box = MatrixBox("Rotation")
        self.rotation_box.set_data(np.full([3, 3], np.inf))

        self.translation_box = MatrixBox("Translation")
        self.translation_box.set_data(np.full([3, 1], np.inf))

        self.angle_box = MatrixBox("Roll/Pitch/Yaw")
        self.angle_box.set_data(np.full([3, 1], np.inf))

        self.save_button = QtWidgets.QPushButton("Save")
        self.save_button.setDisabled(True)
        self.save_button.clicked.connect(self.on_save_button)

        self.update_calibration.connect(self.on_update_calibration)

        layout = QtWidgets.QVBoxLayout()
        layout.setAlignment(QtCore.Qt.AlignTop)
        layout.addWidget(self.update_button)
        layout.addWidget(self.calibrate_button)
        layout.addWidget(self.homography_box)
        layout.addWidget(self.rotation_box)
        hbox = QtWidgets.QHBoxLayout()
        hbox.addWidget(self.translation_box)
        hbox.addWidget(self.angle_box)
        layout.addLayout(hbox)
        layout.addWidget(self.save_button)
        self.setLayout(layout)
        self.setSizePolicy(
            (QtWidgets.QSizePolicy.Minimum
             +QtWidgets.QSizePolicy.MinimumExpanding),
            QtWidgets.QSizePolicy.Minimum)

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
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save Calibration Data",
            startup_path,
            "(*.yaml)",
            "(*.yaml")
        if file_path:
            file_path = file_path.strip(".yaml")
            file_path = "".join([file_path, ".yaml"])

            (homography,
             rotation_matrix,
             translation) = self.get_calibration_data()

            with open(file_path, "w") as file_handle:
                data = dict(
                    H=dict(
                        rows=homography.shape[0],
                        cols=homography.shape[1],
                        data=homography.ravel().tolist(),
                    ),
                    S=dict(
                        rows=rotation_matrix.shape[0],
                        cols=rotation_matrix.shape[1],
                        data=rotation_matrix.ravel().tolist(),
                    ),
                    t=dict(
                        rows=translation.shape[0],
                        cols=translation.shape[1],
                        data=translation.ravel().tolist(),
                    ),
                )
                yaml.safe_dump(data, file_handle)

    def get_calibration_data(self):
        homography = self.homography_box.get_data()
        rotation_matrix = self.rotation_box.get_data()
        translation = self.translation_box.get_data()
        return homography, rotation_matrix, translation

    def on_update_calibration(
            self,
            homography,
            rotation_matrix,
            translation,
            angles):

        self.save_button.setDisabled(False)
        self.homography_box.set_data(homography)
        self.rotation_box.set_data(rotation_matrix)
        self.translation_box.set_data(translation)
        self.angle_box.set_data(angles)


class MatrixBox(QtWidgets.QGroupBox):
    def __init__(self, name):
        super(MatrixBox, self).__init__()
        layout = QtWidgets.QHBoxLayout()
        self.setTitle(name)
        self.matrix = Matrix((3, 3))
        layout.addWidget(self.matrix)
        self.setLayout(layout)

    def set_data(self, ndarray):
        self.matrix.set_data(ndarray)

    def get_data(self):
        return self.matrix.get_data()


class Matrix(QtWidgets.QTableWidget):
    def __init__(self, (rows, columns)):
        super(Matrix, self).__init__()
        self.setRowCount(rows)
        self.setColumnCount(columns)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Minimum,
            QtWidgets.QSizePolicy.Minimum)

        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.data = None


    def set_data(self, ndarray):
        self.data = ndarray
        rows, columns = ndarray.shape
        self.setRowCount(rows)
        self.setColumnCount(columns)
        for row in range(rows):
            for col in range(columns):
                item = QtWidgets.QTableWidgetItem(
                    "{:01.3f}"
                    "".format(ndarray[row, col]))
                item.setFlags(item.flags()
                              ^ (QtCore.Qt.ItemIsEditable
                                 +QtCore.Qt.ItemIsSelectable))
                self.setItem(row, col, item)
        self.resizeColumnsToContents()
        self.setFixedSize(
            self.horizontalHeader().length()+self.verticalHeader().width(),
            self.verticalHeader().length()+self.horizontalHeader().height())

    def get_data(self):
        return self.data


class Canvas(QtWidgets.QLabel):
    update_request = QtCore.pyqtSignal(np.ndarray, np.ndarray)
    def __init__(self):
        super(Canvas, self).__init__()
        self.update_request.connect(self.set_image)
        self.setMinimumWidth(320)
        self.setMinimumHeight(240)
        self.original_image = None
        self.pix = None
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding)

    def set_image(self, cv_image, image_corners):
        self.original_image = cv_image
        height, width, _ = cv_image.shape
        height, width, channels = cv_image.shape
        bytes_per_line = channels * width
        qt_image = QtGui.QImage(
            image_corners.data,
            width, height,
            bytes_per_line,
            QtGui.QImage.Format_RGB888)

        qt_image = qt_image.rgbSwapped()
        self.pix = QtGui.QPixmap.fromImage(qt_image)
        pixmap = QtGui.QPixmap.fromImage(qt_image).scaled(
            self.width(),
            self.height(),
            QtCore.Qt.KeepAspectRatio)
        self.setPixmap(pixmap)

    def resizeEvent(self, QResizeEvent):
        if self.original_image is None:
            return
        self.setPixmap(
            self.pix.scaled(self.width(),
                            self.height(),
                            QtCore.Qt.KeepAspectRatio))



def main():
    app = QtWidgets.QApplication([])
    node = Node()
    icon_path = os.path.join(rospkg.RosPack().get_path("extrinsics"),
                             "images/icon.png")
    app.setWindowIcon(QtGui.QIcon(icon_path))
    node.show()
    app.exec_()


if __name__ == "__main__":
    main()
