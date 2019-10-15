#!/usr/bin/env python
import rospy
import os
import rospkg
import sensor_msgs.msg
import numpy
import cv2


def get_param(param_name, default_val):
    """
    Tries to get a ros parameter. Sets parameter to 'default_val' if no value was set
    :param param_name: Name of the parameter to get
    :param default_val: Default value to use, if parameter does not exist
    :return: Returns the actual parameter value
    """
    val = rospy.get_param(param_name, default_val)
    rospy.set_param(param_name, val)
    rospy.loginfo("[{}] {} = {}".format(rospy.get_name(), param_name, val))
    return val


def get_config_file_path(config_dir_name, file_name):
    """
    Gets file path of config file in picar package
    :param config_dir_name: Name of the directory, not the complete path!
    :param file_name: File name without extension
    :return: Returns file path if file exists. Returns None otherwise.
    """
    default_is_file = True
    config_file_path = os.path.join(rospkg.RosPack().get_path("picar"),
                                    "config",
                                    config_dir_name,
                                    file_name+".yaml")

    default_config_file_path = os.path.join(rospkg.RosPack().get_path("picar"),
                                            "config",
                                            config_dir_name,
                                            "default.yaml")

    if not os.path.isfile(default_config_file_path):
        rospy.logwarn("No default configuration file in "
                      "{}!".format(os.path.dirname(default_config_file_path)))
        default_is_file = False

    if not os.path.isfile(config_file_path):
        rospy.logwarn("No file with name "
                      "\'{}\' in "
                      "\'{}\'!".format(file_name,
                                       os.path.dirname(config_file_path)))

        if not default_is_file:
            return None
        else:
            rospy.logwarn("Falling back to default config file in "
                          "{}.".format(os.path.dirname(config_file_path)))
            return default_config_file_path

    return config_file_path


def get_camera_info():
    rospy.loginfo("[{}] "
                  "Waiting for camera_info message."
                  "".format(rospy.get_name())
                  )

    camera_info = rospy.wait_for_message("camera_node/camera_info",
                                         sensor_msgs.msg.CameraInfo)

    rospy.loginfo("[{}] "
                  "Received camera_info message"
                  "".format(rospy.get_name())
                  )
    if camera_info.K[0] == 0.0:
        rospy.logfatal("[{}] "
                       "No intrinsic information available. "
                       "Are you sure you have done the calibration?")
        rospy.signal_shutdown("Missing intrinsic calibration data.\n"
                              "Shutting down!")
    return camera_info


class FisheyeCameraModel:
    def __init__(self):
        self.K = numpy.empty([3, 3], dtype=numpy.float32)
        self.D = numpy.empty([1, 4], dtype=numpy.float32)
        self.P = numpy.empty([3, 4], dtype=numpy.float32)
        self.R = numpy.eye(3, dtype=numpy.float32)
        self.width = 0
        self.height = 0
        self.map1 = None
        self.map2 = None

    def from_camera_info(self, camera_info):
        """

        :type camera_info: CameraInfo
        """
        self.K = numpy.resize(camera_info.K, [3, 3]).astype(dtype=numpy.float32)
        self.D = numpy.resize(camera_info.D, [1, 4]).astype(dtype=numpy.float32)
        self.P = numpy.resize(camera_info.P, [3, 4]).astype(dtype=numpy.float32)
        self.P = self.P[:3, :3]
        self.width = camera_info.width
        self.height = camera_info.height
        self._init_undistort_map()

    def _init_undistort_map(self):
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            self.K,
            self.D,
            self.R,
            self.P,
            (self.width, self.height),
            cv2.CV_32FC1)

    def undistort_point(self, point):
        src = numpy.array(point, dtype=numpy.float32)
        src.resize([1, 1, 2])
        dst = cv2.fisheye.undistortPoints(src,
                                          self.K,
                                          self.D,
                                          R=self.R,
                                          P=self.P)
        return dst[0, 0]

    def undistort_image(self, image):
        return cv2.remap(image, self.map1, self.map2, cv2.INTER_CUBIC)




