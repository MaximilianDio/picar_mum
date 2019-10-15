from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import cv2
import numpy as np
import yaml
from .intrinsics import Camera
import os
import rospkg
import rospy


class App(QApplication):
    def __init__(self):
        super(App, self).__init__([])
        self.root = MainWidget()
        self.root.show()


class MainWidget(QWidget):
    def __init__(self):

        super(MainWidget, self).__init__()

        self.canvas = Canvas()
        self.side_pane = SidePane()
        self.calibrator = None

        layout = QHBoxLayout()
        layout.addWidget(self.canvas)
        layout.addWidget(self.side_pane)
        self.setLayout(layout)
        icon_path = os.path.join(rospkg.RosPack().get_path("extrinsics"),
                                 "images/icon.png")
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle("Intrinsic Calibration")

    def set_calibrator(self, calibrator):
        self.calibrator = calibrator
        self.side_pane.calibrator = calibrator


class Canvas(QLabel):
    new_image = pyqtSignal(np.ndarray, name="new_image")

    def __init__(self):
        super(Canvas, self).__init__()
        self.setFixedSize(QSize(640, 480))
        self.image = None
        self.new_image.connect(self.on_new_image)

    def on_new_image(self, data):
        self._set_image(data)

    def _set_image(self, image):
        # resize image to canvas size
        image = cv2.resize(image, (self.width(), self.height()))
        height, width, colors = image.shape
        bytes_per_line = colors * width
        qt_image = QImage(image.data, width, height,
                          bytes_per_line, QImage.Format_RGB888)
        qt_image = qt_image.rgbSwapped()
        self.setPixmap(QPixmap.fromImage(qt_image))
        # self.update()


class SidePane(QWidget):
    update_params = pyqtSignal(list, int, int, name="update_params")
    calibration_ready = pyqtSignal(name="calibration_ready")

    def __init__(self):
        super(SidePane, self).__init__()

        self.calibrator = None

        self.widget_list = []
        self.x = LabelAndBar("left/right")
        self.widget_list.append(self.x)

        self.y = LabelAndBar("up/down")
        self.widget_list.append(self.y)

        self.size = LabelAndBar("big/small")
        self.widget_list.append(self.size)

        self.skew = LabelAndBar("skew")
        self.widget_list.append(self.skew)

        self.n_images = NumberImages("Collected Images")
        self.widget_list.append(self.n_images)

        self.scale_slider = SliderGroup(self)
        self.scale_slider.update_slider.connect(self.on_slider_changed)
        self.widget_list.append(self.scale_slider)

        self.calibrate_button = QPushButton("Calibrate")
        self.calibrate_button.clicked.connect(self.do_calibration)
        self.calibrate_button.setDisabled(True)
        self.widget_list.append(self.calibrate_button)

        self.save_button = QPushButton("Save")
        self.save_button.clicked.connect(self.save_calibration)
        self.save_button.setDisabled(True)
        self.widget_list.append(self.save_button)

        self.v_layout = self.create_layout()
        self.setLayout(self.v_layout)

        self.update_params.connect(self.on_update_params)
        self.calibration_ready.connect(self.on_calibration_ready)
        self.calibration_callback = None

    def on_slider_changed(self, scale, val):
        self.calibrator.change_projection_matrix(scale*0.01, val)


    def on_update_params(self, params, n_images, n_max):
        for param in params:
            name = param[0]
            minimum = param[1]
            maximum = param[2]
            progress = param[3]
            getattr(self, name).progress_bar.progress_min = minimum
            getattr(self, name).progress_bar.progress_max = maximum
            getattr(self, name).progress_bar.progress = progress
        self.n_images.progress_bar.setRange(0, n_max)
        self.n_images.progress_bar.setValue(n_images)
        self.n_images.progress_bar.setTextVisible(True)
        self.update()

    def on_calibration_ready(self):
        self.calibrate_button.setDisabled(False)

    def create_layout(self):
        layout = QVBoxLayout()
        for index, value in enumerate(self.widget_list):
            layout.addWidget(self.widget_list[index])
        return layout

    def do_calibration(self):
        if self.calibrator is not None:
            self.calibrator.calibrate()
            self.save_button.setDisabled(False)
            self.scale_slider.enable()

    def save_calibration(self):
        if self.calibrator is not None:
            images = self.calibrator.get_images()
            startup_path = os.path.join(
                rospkg.RosPack().get_path("picar"),
                "config",
                "intrinsics")
            veh_name = "".join([rospy.get_namespace().strip("/"), ".yaml"])
            startup_path = os.path.join(startup_path, veh_name)
            calibration_data = self.calibrator.get_calibration_data()
            file_path, _ = QFileDialog().getSaveFileName(
                self,
                "Save Calibration Data",
                startup_path,
                "(*.yaml)",
                "(*.yaml)"
            )
            if file_path:
                file_path = file_path.strip(".yaml")
                file_path = "".join([file_path, ".yaml"])
                with open(file_path, "w") as file_handle:
                    self.write_intrinsics_file(file_handle, calibration_data)

    def write_intrinsics_file(self, file_handle, calibration_data):
        """

        :type file_handle: file
        :type calibration_data: Camera
        """
        K = calibration_data.K
        D = calibration_data.D
        P = np.zeros([3, 4], dtype=np.float32)
        P[:, :3] = calibration_data.P
        width = calibration_data.width
        height = calibration_data.height
        distortion_model = calibration_data.distortion_model
        data = dict(
            image_width=width,
            image_height=height,
            camera_matrix=dict(
                rows=K.shape[0],
                cols=K.shape[1],
                data=K.ravel().tolist(),
            ),
            distortion_coefficients=dict(
                rows=D.shape[0],
                cols=D.shape[1],
                data=D.ravel().tolist(),
            ),
            rectification_matrix=dict(
                rows=3,
                cols=3,
                data=np.eye(3).ravel().tolist(),
            ),
            projection_matrix=dict(
                rows=P.shape[0],
                cols=P.shape[1],
                data=P.ravel().tolist(),
            ),
            distortion_model=distortion_model
        )
        yaml.safe_dump(data, file_handle)


class LabelAndBar(QWidget):
    def __init__(self, name):
        super(LabelAndBar, self).__init__()
        self.label = QLabel(name, parent=self)
        self.progress_bar = ProgressBar(parent=self)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.progress_bar)
        self.setLayout(layout)


class NumberImages(QWidget):
    def __init__(self, name):
        super(NumberImages, self).__init__()
        self.label = QLabel(name, parent=self)
        self.progress_bar = QProgressBar(parent=self)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.progress_bar)
        self.setLayout(layout)


class ProgressBar(QWidget):
    def __init__(self, parent=None, size=(100, 20)):
        super(ProgressBar, self).__init__(parent=parent)
        self.progress_min = 0.5
        self.progress_max = 0.5
        self.progress = 0.0
        self.setMinimumHeight(size[1])
        self.setMinimumWidth(size[0])

    def paintEvent(self, event):
        width = self.width()
        height = self.height()
        painter = QPainter(self)
        color = QColor(255, 255, 255)
        painter.setBrush(QBrush(color, Qt.SolidPattern))
        painter.drawRect(0, 0, width-1, height-1)
        color = QColor(255, int(255*self.progress), 0)
        if self.progress >= 1.0:
            color = QColor(0, 255, 0)
        painter.setBrush(QBrush(color, Qt.SolidPattern))
        l = width*self.progress_min
        r = width*self.progress_max
        painter.drawRect(l, 0, r-l, height-1)


class SliderGroup(QWidget):
    update_slider = pyqtSignal(int, bool, name="update_slider")

    def __init__(self, parent=None):
        super(SliderGroup, self).__init__(parent=parent)
        self.slider = GenericSlider("scale", (0, 100))
        self.slider.update_slider.connect(self.on_slider_update)
        self.checkbox = QCheckBox()
        self.label = QLabel("use scale")
        self.checkbox.toggled.connect(self.on_checkbox_update)

        self.disable()

        layout_vertical = QVBoxLayout()
        layout_vertical.addWidget(self.label)
        layout_vertical.addWidget(self.checkbox)
        layout_vertical.setAlignment(Qt.AlignCenter)

        layout = QHBoxLayout()
        layout.addLayout(layout_vertical)
        layout.addWidget(self.slider)
        self.setLayout(layout)

    def on_slider_update(self, scale):
        val = self.checkbox.isChecked()
        self.update_slider.emit(scale, val)

    def on_checkbox_update(self, val):
        scale = self.slider.slider.value()
        self.update_slider.emit(scale, val)

    def enable(self):
        self.checkbox.setDisabled(False)
        self.slider.slider.setDisabled(False)

    def disable(self):
        self.checkbox.setDisabled(True)
        self.slider.slider.setDisabled(True)


class GenericSlider(QWidget):
    update_slider = pyqtSignal(int, name="update_slider")

    def __init__(self, name, range):
        super(GenericSlider, self).__init__()
        self.slider = QSlider(Qt.Horizontal)
        self.name_label = QLabel(name)
        self.value_label = QLabel(str(range[0]))
        self.range = range
        self.name = name
        self.init_ui()

    def init_ui(self):
        layout_horizontal = QHBoxLayout()
        layout_horizontal.addWidget(self.slider)
        layout_horizontal.addWidget(self.value_label)

        layout_vertical = QVBoxLayout()
        layout_vertical.addWidget(self.name_label)
        layout_vertical.addLayout(layout_horizontal)

        self.setLayout(layout_vertical)

        self.slider.setRange(self.range[0], self.range[1])
        self.slider.valueChanged.connect(self.slider_changed)

    def slider_changed(self, value):
        self.value_label.setText(str(value))
        self.update_slider.emit(value)




if __name__ == "__main__":
    app = QApplication([])
    main_window = MainWidget()
    main_window.show()
    app.exec_()