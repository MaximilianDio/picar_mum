#!/usr/bin/env python
import rospy
import yaml
from picar_msgs.srv import SetValue, SetBool, SetSteeringParametersList, SetVelocityParametersList
from std_msgs.msg import Float32
from picar_msgs.msg import CarCmd, MsgCurvePoint2D
from picar_common.picar_common import get_param, get_config_file_path, set_param

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

        self.DEBUG = True

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
        # point on curve with position and circle
        rospy.Subscriber("~curve_point", MsgCurvePoint2D, self.update_curved_point)
        # Kalmanfilter output - get velocity est.
        rospy.Subscriber("~velocity_estimated", Float32, self.update_own_velocity)

    def init_publishers(self):
        """ initialize ROS publishers and stores them in a dictionary"""
        # Commanded Values to Picar
        self.publishers["car_cmd"] = rospy.Publisher("motor_node/car_cmd", CarCmd, queue_size=1)

    def init_services(self):

        self.services["set_vel_reference"] = rospy.Service(
            "~set_vel_reference",
            SetValue,
            self.service_vel_reference)

        self.services["set_switching_radius"] = rospy.Service(
            "~set_switching_radius",
            SetValue,
            self.service_switching_radius)

        # ---------------- controller params -------------------------

        self.services["update_steering_parameters"] = rospy.Service(
            "~update_steering_parameters",
            SetSteeringParametersList,
            self.service_steering_params)

        self.services["update_velocity_parameters"] = rospy.Service(
            "~update_velocity_parameters",
            SetVelocityParametersList,
            self.service_steering_params)

    def init_controller(self):

        # ---------------- init controllers -------------------------
        # steering control
        self.steering_control = steering_controller.PathTrackingFF(self._params["Kp_steering"],
                                                                   self._params["Kp_c_steering"],
                                                                   self._params["xLA_steering"])
        # velocity picker
        self.velocity_picker = velocity_picker.VelocityPicker(self._params["vel_reference"],
                                                              self._params["switch_bound"])

        # velocity control
        self.velocity_control = velocity_controller.VelocityController(self._params["Kp_velocity"],
                                                                       self._params["Kd_velocity"])

    # ---------------- controller params -------------------------

    def service_vel_reference(self, request):
        """Sets values via service"""
        self.velocity_picker.minimal_velocity = request.value
        print ("<<<<< Hebel auf den Tisch >>>>>")
        return 1

    def service_switching_radius(self, request):
        """ set switching radius of velocity picker"""
        self.velocity_picker.switch_bound = request.value
        print("%%%%% Schnipp Schnapp %%%%%")
        return 1

    def service_steering_params(self, request):
        self.steering_control.Kp = request.Kp
        self.steering_control.Kp_c = request.Kp_c
        self.steering_control.xLA = request.xLA
        print ("<------ Heiz Heiz ------> ")
        return 1

    def service_velocity_params(self, request):
        self.velocity_control.kp = request.Kp
        self.velocity_control.kd = request.Kd
        print ("<------ Take me to the limit ------> ")
        return 1

    # ---------------------- Topic Callbacks -----------------------------

    def update_curved_point(self, message):
        # message of type MsgCurvePoint2D
        curve_point = message
        # update curvepoint
        if curve_point.x == DEFAULT_FALSE_FLOAT_VALUE or curve_point.y == DEFAULT_FALSE_FLOAT_VALUE:
            self.curve_point = None
        else:
            self.curve_point = curve_point

        self.control_picar()

    def update_own_velocity(self, message):
        self.own_velocity_est = message.data

    def control_picar(self):

        if self.curve_point is None:
            angle = 0
            velocity = 0

        else:
            angle = self.steering_control.get_steering_output(self.curve_point, self.own_velocity_est)

            des_velocity = self.velocity_picker.get_velocity(self.curve_point)

            #velocity = des_velocity

            velocity = self.velocity_control.get_velocity_output(des_velocity, self.own_velocity_est)
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
