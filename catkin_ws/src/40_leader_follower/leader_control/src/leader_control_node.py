#!/usr/bin/env python
import rospy
import yaml

from geometry_msgs.msg import Point32
from leader_control.controller import Controller, ControllerValues, ControllerValuesDesired
from picar_common.picar_common import get_param, get_config_file_path, set_param
from picar_msgs.msg import LeaderPose, CarCmd
from std_msgs.msg import Float64
from picar_msgs.srv import SetValue


class LeaderControlNode(object):
    """Leader controller ROS node."""

    def __init__(self):
        self._params = {}
        self.publishers = {}
        self.services = {}
        config_file_name = get_param("~config_file_name", "default")

        config_file_path = get_config_file_path("leader_control",
                                                config_file_name)

        if config_file_path is None:
            rospy.signal_shutdown("Could not find any config file... "
                                  "Shutting down!")

        # read the controller configuration from config file
        self.setup_params(config_file_path)

        self.controller = Controller(self._params["k_pvel"],
                                     self._params["k_psteer"],
                                     self._params["k_dvel"],
                                     self._params["k_dsteer"])

        # register all publishers
        self.init_publishers()
        # create all services
        self.init_services()

        self.last_values = ControllerValues(0.0,0.0)

#TODO set subscriber
        self.sub_pose = rospy.Subscriber("~leader_relative_pos_input",
                                         Point32,
                                         self.pose_cb,
                                         queue_size=1)
    #### Main code ends

    def init_services(self):
        """Initialize ROS services to configure the controller during runtime"""

        self.services["set_k_pvel"] = rospy.Service(
            "~set_k_pvel",
            SetValue,
            self.set_k_pvel)

        self.services["set_k_psteer"] = rospy.Service(
            "~set_k_psteer",
            SetValue,
            self.set_k_psteer)

        self.services["set_k_dvel"] = rospy.Service(
            "~set_k_dvel",
            SetValue,
            self.set_k_dvel)

        self.services["set_k_dsteer"] = rospy.Service(
            "~set_k_dsteer",
            SetValue,
            self.set_k_dsteer)

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

        self.publishers["error_distance"] = rospy.Publisher(
            "~error_distance",
            Float64,
            queue_size=1)

        self.publishers["error_velocity"] = rospy.Publisher(
            "~error_velocity",
            Float64,
            queue_size=1)

        self.publishers["error_y"] = rospy.Publisher(
            "~error_x",
            Float64,
            queue_size=1)

        self.publishers["error_dy"] = rospy.Publisher(
            "~error_dx",
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
            set_param("~" + param_name, self._params[param_name])

    def set_k_pvel(self, request):
        """Sets the k_pvel_gain parameter of the controller."""
        self._params["k_pvel"] = request.value
        set_param("~k_pvel", request.value)
        self.update_controller()
        return 1

    def set_k_psteer(self, request):
        """Sets the k_psteer_gain parameter of the controller."""
        self._params["k_psteer"] = request.value
        set_param("~k_psteer", request.value)
        self.update_controller()
        return 1

    def set_k_dvel(self, request):
        """Sets the k_dvel_gain parameter of the controller."""
        self._params["k_dvel"] = request.value
        set_param("~k_dvel", request.value)
        self.update_controller()
        return 1

    def set_k_dsteer(self, request):
        """Sets the k_dsteer_gain  parameter of the controller."""
        self._params["k_dsteer"] = request.value
        set_param("~k_dsteer", request.value)
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
        """Updates the controller object's picar"""
        self.controller.update_parameters(
            self._params["k_pvel"],
            self._params["k_psteer"],
            self._params["k_dvel"],
            self._params["k_dsteer"]
        )

    def pose_cb(self, message):
        """Handles incoming pose messages to send angle/velocity commands to the vehicle.

        Args:
            message (LeaderPose):
        """
        desired_values = ControllerValuesDesired(self._params["distance_desired"],
                                          self._params["velocity_desired"])

        actual_values = ControllerValues(message.x,
                                         message.y)

        last_values = self.last_values

        (steering_angle,
         velocity,
         errors) = self.controller.get_control_output(desired_values,
                                                      actual_values,last_values)

        self.publish_car_cmd(steering_angle, velocity)

        self.last_values=actual_values

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
            errors (iterable): Contains distance, velocity, x and dx
        """

        error_distance = Float64()
        error_distance.data = errors[0]

        error_velocity = Float64
        error_velocity.data = errors[1]

        error_y = Float64
        error_y.data = errors[2]

        error_dy = Float64
        error_dy.data = errors[3]

        self.publishers["error_distance"].publish(error_distance)
        self.publishers["error_velocity"].publish(error_velocity)

        self.publishers["error_x"].publish(error_y)
        self.publishers["error_dx"].publish(error_dy)


def main():
    """Starts the leader control node"""
    rospy.init_node("leader_control_node")
    LeaderControlNode()
    rospy.spin()


if __name__ == "__main__":
    main()
