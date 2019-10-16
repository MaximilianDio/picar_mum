#!/usr/bin/env python
import rospy
import yaml
from lane_control.controller import Controller, ControllerValues
from picar_common.picar_common import get_param, get_config_file_path, set_param
from picar_msgs.msg import LanePose, CarCmd
from std_msgs.msg import Float64
from picar_msgs.srv import SetValue


class LaneControlNode(object):
    """Lane controller ROS node."""

    def __init__(self):
        self._params = {}
        self.publishers = {}
        self.services = {}
        config_file_name = get_param("~config_file_name", "default")

        config_file_path = get_config_file_path("lane_control",
                                                config_file_name)

        if config_file_path is None:
            rospy.signal_shutdown("Could not find any config file... "
                                  "Shutting down!")

        # read the controller configuration from config file
        self.setup_params(config_file_path)

        self.controller = Controller(self._params["p_gain"],
                                     self._params["weight_distance"],
                                     self._params["weight_angle"])

        # register all publishers
        self.init_publishers()
        # create all services
        self.init_services()

        self.sub_pose = rospy.Subscriber("~pose_input",
                                         LanePose,
                                         self.pose_cb,
                                         queue_size=1)

    def init_services(self):
        """Initialize ROS services to configure the controller during runtime"""
        self.services["set_p_gain"] = rospy.Service(
            "~set_p_gain",
            SetValue,
            self.set_p_gain)
        self.services["set_distance_desired"] = rospy.Service(
            "~set_distance_desired",
            SetValue,
            self.set_distance_desired)
        self.services["set_velocity_desired"] = rospy.Service(
            "~set_velocity_desired",
            SetValue,
            self.set_velocity_desired)


    def init_publishers(self):
        """Initialize ROS publishers and stores them in a dictionary

        """
        self.publishers["distance_error"] = rospy.Publisher(
            "~distance_error",
            Float64,
            queue_size=1)
        self.publishers["angle_error"] = rospy.Publisher(
            "~angle_error",
            Float64,
            queue_size=1)

        self.publishers["combined_error"] = rospy.Publisher(
            "~combined_error",
            Float64,
            queue_size=1)

        self.publishers["car_cmd"] = rospy.Publisher(
            "~car_cmd",
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
            set_param("~"+param_name, self._params[param_name])

    def set_p_gain(self, request):
        """Sets the p_gain parameter of the controller."""
        self._params["p_gain"] = request.value
        set_param("~p_gain", request.value)
        self.update_controller()
        return 1

    def set_distance_desired(self, request):
        """Sets the distance_desired parameter of the controller"""
        self._params["distance_desired"] = request.value
        set_param("~distance_desired", request.value)
        return 1

    def set_velocity_desired(self, request):
        """Sets the velocity_desired parameter of the controller"""
        self._params["velocity_desired"] = float(request.value)
        set_param("~velocity_desired", request.value)
        return 1

    def update_controller(self):
        """Updates the controller object's parameters"""
        self.controller.update_parameters(
            self._params["p_gain"],
            self._params["weight_distance"],
            self._params["weight_angle"]
        )

    def pose_cb(self, message):
        """Handles incoming pose messages to send angle/velocity commands to the vehicle.

        Args:
            message (LanePose):
        """
        desired_values = ControllerValues(self._params["distance_desired"],
                                          self._params["angle_desired"],
                                          self._params["velocity_desired"])

        actual_values = ControllerValues(message.d,
                                         message.phi,
                                         self._params["velocity_desired"])
        (steering_angle,
         velocity,
         errors) = self.controller.get_control_output(desired_values,
                                                      actual_values)

        self.publish_car_cmd(steering_angle, velocity)

        self.publish_errors(errors)

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
        self.publishers["car_cmd"].publish(message)

    def publish_errors(self, errors):
        """ Publishes error messages.

        Args:
            errors (iterable): Contains distance, angle and combined error.
        """
        distance_error = Float64()
        distance_error.data = errors[0]

        angle_error = Float64()
        angle_error.data = errors[1]

        combined_error = Float64()
        combined_error.data = errors[2]

        self.publishers["distance_error"].publish(distance_error)
        self.publishers["angle_error"].publish(angle_error)
        self.publishers["combined_error"].publish(combined_error)


def main():
    """Starts the lane control node"""
    rospy.init_node("lane_control_node")
    LaneControlNode()
    rospy.spin()


if __name__ == "__main__":
    main()
