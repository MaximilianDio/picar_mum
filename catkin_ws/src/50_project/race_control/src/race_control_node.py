#!/usr/bin/env python
import rospy
import yaml
from picar_common.picar_common import get_param, get_config_file_path, set_param
from picar_msgs.msg import CarCmd, MsgCurvePoint2D
from std_msgs.msg import Float64, Float32
from picar_msgs.srv import SetValue
from steering_control import steering_controller
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

        # import parameters from config yaml files
        config_file_name = get_param("~config_file_name", "default")

        config_file_path = get_config_file_path("race_control",
                                                config_file_name)

        # shut down node if no config file could be found
        if config_file_path is None:
            rospy.signal_shutdown("Could not find any config file... "
                                  "Shutting down!")

        # read the controller configuration from config file
        self.setup_params(config_file_path)

        # register all publishers
        self.init_subscribers()

        # register all publishers
        self.init_publishers()

        # create all services
        self.init_services()

        # initialize controllers
        self.steering_control = steering_controller.SteeringController(self._params["Kp_steering"],
                                                                       self._params["Kp_c_steering"],
                                                                       self._params["xLA_steering"])

        self.velocity_control = velocity_controller.VelocityController(self._params["Kp_velocity"],
                                                                       self._params["Kd_velocity"])

    def init_subscribers(self):
        """ initialize ROS subscribers and stores them in a dictionary"""
        # Kalmanfilter output to get velocity est.
        rospy.Subscriber("~velocity_estimated", Float32, self.update_velocity_estimation,
                         queue_size=1)
        # Get Vision data
        rospy.Subscriber("~curve_point", MsgCurvePoint2D, self.vehicle_control, queue_size=1)

    def init_publishers(self):
        """Initialize ROS publishers and stores them in a dictionary

        """

        self.publishers["motor_node/car_cmd"] = rospy.Publisher(
            "motor_node/car_cmd",
            CarCmd,
            queue_size=1)

    def init_services(self):
        """Initialize ROS services to configure the controller during runtime"""

        self.services["set_kp"] = rospy.Service("~set_kp",
                                                SetValue,
                                                self.set_kp)

        self.services["set_kp_c"] = rospy.Service("~set_kp_c",
                                                  SetValue,
                                                  self.set_kp_c)

        self.services["set_xLA"] = rospy.Service("~set_xLA",
                                                 SetValue,
                                                 self.set_xLA)

        self.services["set_p_gain"] = rospy.Service("~set_p_gain",
                                                    SetValue,
                                                    self.set_p_gain)

        self.services["set_d_gain"] = rospy.Service("~set_d_gain",
                                                    SetValue,
                                                    self.set_d_gain)

    def setup_params(self, config_file_path):
        with open(config_file_path, "r") as file_handle:
            data = yaml.safe_load(file_handle)
        self._params = data
        for param_name in self._params:
            set_param("~" + param_name, self._params[param_name])

    def set_kp(self, message):
        self.steering_control.Kp = message.value
        return 1

    def set_kp_c(self, message):
        self.steering_control.Kp_c = message.value
        return 1

    def set_xLA(self, message):
        self.steering_control.xLA = message.value
        return 1

    def set_p_gain(self, message):
        self.velocity_control.kp = message.value
        return 1

    def set_d_gain(self, message):
        self.velocity_control.kd = message.value
        return 1

    def update_velocity_estimation(self, message):
        self.velocity_est = message.data

    def vehicle_control(self, message):

        # get steering angle
        angle = self.steering_control.get_steering_output(message, self.velocity_est)

        # get velocity
        velocity = self._params["vel_reference"] #self.velocity_control.get_velocity_output(self._params["vel_reference"],self.velocity_est)  # input in meters pro second

        self.publish_car_cmd(angle, velocity)

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
