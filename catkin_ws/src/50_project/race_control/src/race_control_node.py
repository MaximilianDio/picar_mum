#!/usr/bin/env python
import rospy
import yaml
from picar_common.picar_common import get_param, get_config_file_path, set_param
from picar_msgs.msg import CarCmd, MsgCurvePoint2D
from std_msgs.msg import Float64, Float32
from picar_msgs.srv import SetValue
from steering_control import path_tracking_ff
from velocity_control import velocity_controller


class RaceControlNode(object):
    """race controller ROS node."""

    def __init__(self):
        pass
        self._params = {}
        self.publishers = {}
        self.services = {}

        # initialize timestamp
        self.timestamp = rospy.get_rostime()
        self.last_time = 0.0
        self.velocity_est = 0.0

        config_file_name = get_param("~config_file_name", "default")

        config_file_path = get_config_file_path("steering_control",
                                                config_file_name)

        if config_file_path is None:
            rospy.signal_shutdown("Could not find any config file... "
                                  "Shutting down!")

        # read the controller configuration from config file
        self.setup_params(config_file_path)

        # initialize controllers
        self.steering_control = path_tracking_ff.PathTrackingFF(self._params["kp"], self._params["kp_c"],
                                                                self._params["xLA"])

        self.velocity_control = velocity_controller.VelocityController(0.12, 0.025)

        # register all publishers
        self.init_publishers()
        # create all services
        self.init_services()

        self.sub_velocity_est = rospy.Subscriber("~velocity_estimated",
                                                 Float32,
                                                 self.update_velocity_estimation,
                                                 queue_size=1)

        self.sub_curved_points = rospy.Subscriber("~curve_point",
                                                  MsgCurvePoint2D,
                                                  self.vehicle_control,  # TODO determine whos callbacking ;)
                                                  queue_size=1)

    def init_services(self):
        """Initialize ROS services to configure the controller during runtime"""

        self.services["set_kp"] = rospy.Service(
            "~set_kp",
            SetValue,
            self.set_kp)

        self.services["set_kp_c"] = rospy.Service(
            "~set_kp_c",
            SetValue,
            self.set_kp_c)

        self.services["set_xLA"] = rospy.Service(
            "~set_xLA",
            SetValue,
            self.set_xLA)

        self.service = rospy.Service("~set_p_gain",
                                     SetValue,
                                     self.set_p_gain)

        self.service = rospy.Service("~set_d_gain",
                                     SetValue,
                                     self.set_d_gain)

    def init_publishers(self):
        """Initialize ROS publishers and stores them in a dictionary

        """

        self.publishers["motor_node/car_cmd"] = rospy.Publisher(
            "motor_node/car_cmd",
            CarCmd,
            queue_size=1)

    def setup_params(self, config_file_path):
        """Reads configs from configuration file.

        Args:
            config_file_path (str): Path of the configuration file.
        """
        with open(config_file_path, "r") as file_handle:
            data = yaml.safe_load(file_handle)
        self._params = data
        for param_name in self._params:
            set_param("~" + param_name, self._params[param_name])

    def set_kp(self, request):
        """Sets the k_pvel_gain parameter of the controller."""
        self._params["kp"] = request.value
        set_param("~kp", request.value)
        self.update_controller()
        return 1

    def set_kp_c(self, request):
        """Sets the k_pvel_gain parameter of the controller."""
        self._params["kp_c"] = request.value
        set_param("~kp_c", request.value)
        self.update_controller()
        return 1

    def set_xLA(self, request):
        """Sets the k_psteer_gain parameter of the controller."""
        self._params["xLA"] = request.value
        set_param("~xLA", request.value)
        self.update_controller()
        return 1

    def set_p_gain(self, message):
        self.velocity_control.kp = message.value
        return 1

    def set_d_gain(self, message):
        self.velocity_control.kd = message.value
        return 1

    def update_controller(self):
        """Updates the controller object's picar"""
        self.steering_control.update_parameters(
            self._params["kp"],
            self._params["kp_c"],
            self._params["xLA"])

    def update_velocity_estimation(self, message):
        self.velocity_est = message.data

    def vehicle_control(self, message):

        controller_values = self.steering_control.format_control_inputs(message, self.velocity_est)

        delta = self.steering_control.get_steering_output(controller_values)

        #velocity = self.velocity_control.get_velocity_output(self.velocity_est, 0.6)  # input in meters pro second

        #if velocity < 0:
         #   print("Velocity negativ!!!")
          #  velocity = 0
        # else:
          #  velocity = velocity

        self.publish_car_cmd(delta, 0.4)

    def publish_car_cmd(self, steering_angle, velocity):
        """Publishes car command to control the car

        Args:
            steering_angle (float): Steering angle command computed by the
                controller.
            velocity (float): Velocity command
                computed by the controller.
        """
        message = CarCmd()
        message.header.stamp = rospy.get_rostime()
        message.angle = steering_angle
        message.velocity = velocity
        self.publishers["motor_node/car_cmd"].publish(message)


def main():
    """Starts the leader control node"""
    rospy.init_node("race_control_node")
    RaceControlNode()
    rospy.spin()


if __name__ == "__main__":
    main()
