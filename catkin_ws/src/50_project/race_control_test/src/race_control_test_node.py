#!/usr/bin/env python
import rospy
import yaml
from picar_msgs.srv import SetValue, SetBool, SetSteeringParametersList, SetVelocityParametersList
from std_msgs.msg import Float32
from picar_msgs.msg import CarCmd, MsgCurvePoint2D
from picar_common.picar_common import get_param, get_config_file_path, set_param
import numpy as np

# import controllers
from steering_control import steering_controller
from velocity_control import velocity_controller
from velocity_picker import velocity_picker

# ros message for float values if nothing was detected
DEFAULT_FALSE_FLOAT_VALUE = float("inf")


class RaceControlNode:
    """race controller ROS node."""

    def __init__(self):
        self._params = {}
        self.publishers = {}
        self.services = {}

        # init attributes
        self.time = 0.0
        self.own_velocity_est = 0.0
        self.curve_point = None  # CurvePointMessage

        self.init_time = rospy.get_rostime()
        self.init_time = self.init_time.secs + float(self.init_time.nsecs * 1e-9)

        self.cur_time = rospy.get_rostime()
        self.cur_time = self.cur_time.secs + float(self.cur_time.nsecs * 1e-9)

        self.cur_time = self.cur_time - self.init_time

        # import parameters from config yaml files
        config_file_name = get_param("~config_file_name", "default")

        config_file_path = get_config_file_path("race_control",
                                                config_file_name)

        # shut down node if no config file could be found
        if config_file_path is None:
            rospy.signal_shutdown("Could not find any config file... "
                                  "Shutting down!")

        # read the config parameters form .yaml file
        self.setup_params(config_file_path)

        self.DEBUG = False

        # register all publishers
        self.init_subscribers()

        # register all publishers
        self.init_publishers()

        # create all services
        self.init_services()

        # create all controller
        self.steering_control = None
        self.velocity_picker = None
        self.velocity_control = None

        self.init_controller()

    # ----------------------- initialization -----------------------------

    def setup_params(self, config_file_path):
        with open(config_file_path, "r") as file_handle:
            data = yaml.safe_load(file_handle)
        self._params = data
        for param_name in self._params:
            set_param("~" + param_name, self._params[param_name])

    def init_subscribers(self):
        """ initialize ROS subscribers and stores them in a dictionary"""
        # Kalmanfilter output - get velocity est.
        rospy.Subscriber("~velocity_estimated", Float32, self.update_own_velocity, queue_size=1)

    def init_publishers(self):
        """ initialize ROS publishers and stores them in a dictionary"""
        # Commanded Values to Picar
        self.publishers["car_cmd"] = rospy.Publisher("motor_node/car_cmd", CarCmd, queue_size=1)

    def init_services(self):




        self.services["update_velocity_parameters"] = rospy.Service(
            "~update_velocity_parameters",
            SetVelocityParametersList,
            self.service_velocity_params)

    def init_controller(self):

        # velocity control
        self.velocity_control = velocity_controller.VelocityController(self._params["Kp_velocity"],
                                                                       self._params["Kd_velocity"])

    # ---------------- controller params -------------------------

    def service_velocity_params(self, request):
        self.velocity_control.kp = request.Kp
        self.velocity_control.kd = request.Kd
        print ("<------ Take me to the limit ------> ")
        return 1

    # ---------------------- Topic Callbacks -----------------------------

    def update_own_velocity(self, message):
        self.own_velocity_est = message.data

        self.control_picar()

    def control_picar(self):

        self.cur_time = rospy.get_rostime()
        self.cur_time = self.cur_time.secs + float(self.cur_time.nsecs * 1e-9)

        self.cur_time = self.cur_time - self.init_time

        if self.cur_time < 8:
            des_velocity = 0
        else:
            des_velocity = 0.3 * np.sin(1 * np.pi * 2 * self.cur_time) + 0.9

        angle = 0
        print "velocity picked: " + str(des_velocity)
        velocity = self.velocity_control.get_velocity_output(des_velocity, self.own_velocity_est)
        print "velocity cmd: " + str(velocity)

        try:
            if self.DEBUG == True:
                print "-----------------------------------------------------------------------------------"
                print "-----------------------------------------------------------------------------------"
                print "-----------------------" + str(
                    rospy.get_rostime()) + "----------------------------"  # TODO change time
                print "curve point: x: " + str(self.curve_point.x) + " y: " + str(self.curve_point.y)
                print "own velocity: " + str(self.own_velocity_est)
                print "angle cmd: " + str(angle)
                print "velocity picked: " + str(des_velocity)
                print "velocity cmd: " + str(velocity)
        except AttributeError:
            pass
        # ---------------- publish CarCmd -------------------------
        des_car_command = CarCmd()

        des_car_command.header.stamp = rospy.get_rostime()
        des_car_command.velocity = velocity
        des_car_command.angle = angle
        self.publishers["car_cmd"].publish(des_car_command)


if __name__ == "__main__":
    rospy.init_node("race_control_node")
    RaceControlNode()
    rospy.spin()
