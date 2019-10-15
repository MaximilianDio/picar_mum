#!/usr/bin/env python
import rospy
from picamera import PiCamera
import io
import yaml
import time
import numpy as np
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from picar_common import get_param, get_config_file_path
from dynamic_reconfigure.server import Server
from camera.cfg import cam_paramsConfig


class FrameSplitter(object):
    def __init__(self, publish_intrinsic_info,
                 camera_info, camera_info_intrinsics):
        self.done = False
        self.pub_img = rospy.Publisher(
            "~image/compressed",
            CompressedImage,
            queue_size=1
        )

        self.pub_camera_info = rospy.Publisher("~camera_info",
                                               CameraInfo,
                                               queue_size=1)

        self.pub_camera_info_intrinsics = rospy.Publisher(
            "~camera_info_intrinsic_calibration",
            CameraInfo,
            queue_size=1
        )

        self.camera_info = camera_info
        self.camera_info_intrinsics = camera_info_intrinsics

        self.msg_img = CompressedImage()
        self.stream = io.BytesIO()
        self.publish_intrinsic_info = publish_intrinsic_info

    def write(self, buf):
        # JPEGs start with the magic number FF D8, so it is used to identify
        # the beginning of a new frame
        if buf.startswith(b'\xff\xd8'):
            size = self.stream.tell()
            if size > 0:
                self.stream.seek(0)
                self.msg_img.data = self.stream.read(size)
                self.msg_img.format = "jpeg"
                self.msg_img.header.stamp = rospy.Time.now()
                try:
                    # publish image
                    self.pub_img.publish(self.msg_img)
                    # publish camera info
                    self.pub_camera_info.publish(self.camera_info)
                    # publish camera info of intrinsic calibration, if known
                    if self.publish_intrinsic_info:
                        self.pub_camera_info_intrinsics.publish(
                            self.camera_info_intrinsics
                        )
                except rospy.ROSException:
                    pass
                self.stream.seek(0)
                self.stream.truncate()
        self.stream.write(buf)

class CameraParams:
    def __init__(self, camera):
        self.camera = camera
        # in microseconds. 0 for automatic
        self.shutter_speed = 0
        #
        self.exposure_mode = 'off'
        # range [0, 100]
        self.brightness = 50
        self.iso = 1600
        # available modes: 'off' 'auto' 'sunlight' 'cloudy' 'shade' 'tungsten' 'fluorescent' 'incandescent' 'flash'
        # 'horizon'
        self.awb_mode = 'off'

        self.awb_gain_red = 1.0
        self.awb_gain_blue = 1.0
        # between 0.0 and 8.0 as tuple (red, blue)
        self.awb_gains = (self.awb_gain_red, self.awb_gain_blue)


    def reconfigure_cb(self, config, level):
        self.get_params_from_config(config)
        self.write_params_to_camera()
        self.get_params_from_camera()
        self.write_params_to_config(config)
        return config

    def get_params_from_config(self, config):
        self.shutter_speed = config["shutter_speed"]

        self.exposure_mode = config["exposure_mode"]

        self.brightness = config["brightness"]

        self.iso = config["iso"]

        self.awb_mode = config["awb_mode"]

        self.awb_gain_red = config["awb_gain_red"]
        self.awb_gain_blue = config["awb_gain_blue"]
        self.awb_gains = (self.awb_gain_red, self.awb_gain_blue)

    def write_params_to_camera(self):
        self.camera.shutter_speed = self.shutter_speed
        self.camera.exposure_mode = self.exposure_mode
        self.camera.brightness = self.brightness
        self.camera.iso = self.iso
        self.camera.awb_mode = self.awb_mode
        self.camera.awb_gains = self.awb_gains

    def get_params_from_camera(self):
        self.shutter_speed = self.camera.shutter_speed
        self.exposure_mode = self.camera.exposure_mode
        self.brightness = self.camera.brightness
        self.iso = self.camera.iso
        self.awb_mode = self.camera.awb_mode
        self.awb_gains = self.camera.awb_gains
        rospy.logwarn("{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n".format(
            self.shutter_speed, self.exposure_mode,
            self.brightness, self.iso, self.awb_mode,
            self.awb_gains, self.awb_gain_red, self.awb_gain_blue
        ))
        self.awb_gains = (float(self.awb_gains[0]), float(self.awb_gains[1]))
        self.awb_gain_red = self.awb_gains[0]
        self.awb_gain_blue = self.awb_gains[1]

    def write_params_to_config(self, config):
        config["shutter_speed"] = self.shutter_speed
        config["exposure_mode"] = self.exposure_mode
        config["brightness"] = self.brightness
        config["iso"] = self.iso
        config["awb_mode"] = self.awb_mode
        config["awb_gain_red"] = self.awb_gain_red
        config["awb_gain_blue"] = self.awb_gain_blue


def read_config_from_file(file_path):
    with open(file_path, "r") as f:
        data = yaml.safe_load(f)
        resolution = (data["width"], data["height"])
        fps = data["fps"]
        fisheye = data["fisheye"]
        return resolution, fps, fisheye


def camera_info_from_intrinsics_file(file_path):
    with open(file_path, "r") as f:
        data = yaml.safe_load(f)
        camera_info = CameraInfo()
        camera_info.height = data["image_height"]
        camera_info.width = data["image_width"]
        camera_info.K = data["camera_matrix"]["data"]
        camera_info.D = data["distortion_coefficients"]["data"]
        camera_info.distortion_model = data["distortion_model"]
        camera_info.R = data["rectification_matrix"]["data"]
        camera_info.P = data["projection_matrix"]["data"]
    return camera_info


def camera_info_scaled(camera_info_intrinsics, image_size):
    camera_info = CameraInfo()
    camera_info.height = image_size[1]
    camera_info.width = image_size[0]

    if camera_info_intrinsics is not None:
        scale = float(image_size[0]) / camera_info_intrinsics.width

        # camera matrix
        camera_info.K = np.array(camera_info_intrinsics.K)*scale
        camera_info.K[-1] = 1.0

        # distortion does not need to be scaled
        camera_info.D = camera_info_intrinsics.D
        camera_info.distortion_model = camera_info_intrinsics.distortion_model

        # rectification
        camera_info.R = camera_info_intrinsics.R

        # projection matrix
        camera_info.P = np.array(camera_info_intrinsics.P)*scale
        camera_info.P[-2] = 1.0

    return camera_info


def main():
    rospy.init_node('camera_node', anonymous=False)
    rate = rospy.Rate(2)

    config_file_name = get_param("~config_file_name", "default")
    config_file_path = get_config_file_path("camera", config_file_name)

    if config_file_path is None:
        rospy.signal_shutdown("Could not find camera config file. "
                              "Shutting down!")

    resolution, fps, fisheye = read_config_from_file(config_file_path)

    intrinsics_file_name = get_param("~intrinsics_file_name", "default")
    intrinsics_file_path = get_config_file_path("intrinsics",
                                                intrinsics_file_name)

    publish_intrinsics_info = True

    if intrinsics_file_path is None:
        camera_info_intrinsics = None
        publish_intrinsics_info = False
    else:
        camera_info_intrinsics = camera_info_from_intrinsics_file(
            intrinsics_file_path
        )

    camera_info = camera_info_scaled(camera_info_intrinsics, resolution)

    if fisheye:
        camera_info.distortion_model = "fisheye"

    output = FrameSplitter(publish_intrinsics_info,
                           camera_info,
                           camera_info_intrinsics)

    rospy.set_param("~width", resolution[0])
    rospy.set_param("~height", resolution[1])
    rospy.set_param("~fps", fps)
    rospy.set_param("~fisheye", fisheye)

    with PiCamera(resolution=resolution, framerate=fps, sensor_mode=4) as camera:
        cam_config = CameraParams(camera)
        srv = Server(cam_paramsConfig, cam_config.reconfigure_cb)
        camera.iso = 400
        if fisheye:
            w = 0.7
            h = 0.7
            x = 0.15
            y = 0.15
            camera.zoom = (x, y, w, h)
        time.sleep(2)
        camera.shutter_speed = camera.exposure_speed
        camera.exposure_mode = 'off'
        g = camera.awb_gains
        camera.awb_mode = 'off'
        camera.awb_gains = g
        camera.start_recording(output, format='mjpeg', quality=100)

        rospy.loginfo("[{}] Camera started".format(rospy.get_name()))
        while not rospy.is_shutdown():
            try:
                camera.wait_recording(0)
                rate.sleep()
            except KeyboardInterrupt:
                rospy.loginfo("Shutting down by KeyboardInterrupt")
                break
        camera.stop_recording()
        rospy.loginfo("Shutting down camera node")


if __name__ == '__main__':
    main()
