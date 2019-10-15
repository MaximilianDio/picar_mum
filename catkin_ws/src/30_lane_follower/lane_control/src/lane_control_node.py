#!/usr/bin/env python
import rospy
import yaml
from lane_control.controller import Controller, ControllerValues
from picar_common.picar_common import get_param, get_config_file_path
from picar_msgs.msg import LanePose, CarCmd
from std_msgs.msg import Float64


class LaneControlNode(object):
    """Lane controller ROS node."""

    def __init__(self):
        self._params = {}
        self.publishers = {}
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

        self.sub_pose = rospy.Subscriber("~pose_input",
                                         LanePose,
                                         self.pose_cb,
                                         queue_size=1)

    def init_publishers(self):
        """Initialize ROS publishers and stores them in a dictionary

        """
        self.publishers["distance_error"] = rospy.Publisher("~distance_error",
                                                            Float64,
                                                            queue_size=1)
        self.publishers["angle_error"] = rospy.Publisher("~angle_error",
                                                         Float64,
                                                         queue_size=1)

        self.publishers["combined_error"] = rospy.Publisher("~combined_error",
                                                            Float64,
                                                            queue_size=1)

        self.publishers["car_cmd"] = rospy.Publisher("~car_cmd",
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
