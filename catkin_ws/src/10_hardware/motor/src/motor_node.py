#!/usr/bin/env python
import time
import os
import yaml
import rospy
import rospkg
import picar_common.picar_common as picar_common
from picar_msgs.msg import CarCmd
from dynamic_reconfigure.server import Server
from std_srvs.srv import Trigger, TriggerResponse
from motor.cfg import motorConfig
import Adafruit_PCA9685


class MotorController(object):
    """Object to control the steering servo and the motor"""
    def __init__(self, freq=200, address=0x40):
        """

        Args:
            freq (int): The PWM frequency of the servo. The servo gain depends
                on the frequency, so it needs to be redetermined, if the
                frequency changes.
            address (int): I2C address of the PCA9685 PWM module.
        """
        # find config file
        config_file_name = picar_common.get_param(
            "~config_file_name",
            "default")
        config_file_path = picar_common.get_config_file_path(
            "motor",
            config_file_name)

        if config_file_path is None:
            rospy.signal_shutdown("Could not find motor config file. "
                                  "Shutting down!")
            return

        # init motor parameters
        self.params = MotorParams(config_file_path)

        # create a timeout variable which is set to false, if a message was received.
        # Once per second it is set to true. So the car won't get crazy of it loses contact
        # to some motor command publisher
        self.msg_timeout = False
        self.timer = rospy.Timer(rospy.Duration(1), self._msg_timeout_cb)

        # init pwm
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(freq)
        self.pwm.set_pwm(self.params.in_1_channel, 0, 4095)
        self.pwm.set_pwm(self.params.in_2_channel, 0, 4095)
        self.pwm.set_pwm(self.params.servo_channel, 0, 1228)
        self.pwm.set_pwm(self.params.motor_forward_channel, 0, 0)
        self.pwm.set_pwm(self.params.motor_backward_channel, 0, 0)

        self.services = {}
        self.init_services()

        # handle shutdowns properly -> release motor
        rospy.on_shutdown(self.on_shutdown)

        # subscribe to motor commands
        self.sub_car_cmd = rospy.Subscriber("~car_cmd",
                                            CarCmd,
                                            self.car_cmd_callback,
                                            queue_size=1)

    def init_services(self):
        self.services["save_config"] = rospy.Service(
            "~save_config",
            Trigger,
            self.save_config)

    def car_cmd_callback(self, msg):
        self.msg_timeout = False
        throttle = msg.velocity
        if throttle < 0:
            throttle = -int(abs(throttle * self.params.throttle_min))
        else:
            throttle = int(throttle * self.params.throttle_max)

        self.set_throttle_pwm(throttle)

        self.set_steering_angle(msg.angle)

    def on_shutdown(self):
        self.set_throttle_pwm(0)
        time.sleep(0.5)
        self.set_steering_angle(0.0)
        time.sleep(0.5)
        self.pwm.set_all_pwm(0, 0)
        time.sleep(1)

    def set_servo_pwm(self, pwm):
        self.pwm.set_pwm(self.params.servo_channel, 0, int(pwm))

    def set_steering_angle(self, angle):
        # angle in radian
        # left steering is positive

        # limit angle to [ANGLE_MIN; ANGLE_MAX]
        angle = max(angle, self.params.angle_min)
        angle = min(angle, self.params.angle_max)

        pwm = int(float(self.params.servo_gain) * angle
                  + self.params.servo_offset)
        self.set_servo_pwm(pwm)

    def set_throttle_pwm(self, pwm):
        if pwm > self.params.throttle_max:
            pwm = self.params.throttle_max
        elif pwm < self.params.throttle_min:
            pwm = self.params.throttle_min

        if pwm < 0:
            self.pwm.set_pwm(self.params.motor_forward_channel, 0, 0)
            self.pwm.set_pwm(self.params.motor_backward_channel, 0, -pwm)
        else:
            self.pwm.set_pwm(self.params.motor_backward_channel, 0, 0)
            self.pwm.set_pwm(self.params.motor_forward_channel, 0, pwm)

    def _msg_timeout_cb(self, event):
        """Called periodically by a timer to stop the car if no message arrived.

        Args:
            event: unused
        """
        if self.msg_timeout:
            self.set_throttle_pwm(0)
            time.sleep(0.1)
            self.set_steering_angle(0.0)
        self.msg_timeout = True

    def reconfigure_callback(self, config, level):
        self.params.servo_gain = config["servo_gain"]
        self.params.servo_offset = config["servo_offset"]
        return config

    def save_config(self, request):
        response = TriggerResponse()
        name = rospy.get_namespace().strip("/")
        path = os.path.join(
            rospkg.RosPack().get_path("picar"),
            "config",
            "motor",
            name + ".yaml"
        )
        try:
            self.params.write_params_to_file(path)
        except IOError as error:
            response.success = False
            response.message = "{}".format(error)
            return response
        response.success = True
        response.message = "Saved configuration in '{}'".format(path)
        return response


class MotorParams(object):
    def __init__(self, params_file_path):
        self.servo_channel = 0
        self.motor_forward_channel = 0
        self.motor_backward_channel = 0
        self.in_1_channel = 0  # BACKWARD MODE
        self.in_2_channel = 0  # FORWARD MODE

        self.angle_max = 0
        self.angle_min = 0
        self.throttle_min = 0
        self.throttle_max = 0

        self.servo_gain = 0.0
        self.servo_offset = 0
        self.read_params_from_file(params_file_path)

    def read_params_from_file(self, file_path):
        with open(file_path, "r") as file_handle:
            data = yaml.safe_load(file_handle)
            self.servo_channel = data["servo_channel"]
            self.motor_forward_channel = data["motor_forward_channel"]
            self.motor_backward_channel = data["motor_backward_channel"]
            self.in_1_channel = data["in_1_channel"]
            self.in_2_channel = data["in_2_channel"]
            self.angle_max = data["angle_max"]
            self.angle_min = data["angle_min"]
            self.throttle_max = data["throttle_max"]
            self.throttle_min = data["throttle_min"]
            self.servo_gain = data["servo_gain"]
            self.servo_offset = data["servo_offset"]
            self.write_params_to_server()

    def read_params_from_server(self):
        self.servo_channel = rospy.get_param(
            "~servo_channel",
            self.servo_channel)
        self.motor_forward_channel = rospy.get_param(
            "~motor_forward_channel",
            self.motor_forward_channel)
        self.motor_backward_channel = rospy.get_param(
            "~motor_backward_channel",
            self.motor_backward_channel)
        self.in_1_channel = rospy.get_param("~in_1_channel", self.in_1_channel)
        self.in_2_channel = rospy.get_param("~in_2_channel", self.in_2_channel)
        self.angle_max = rospy.get_param("~angle_max", self.angle_max)
        self.angle_min = rospy.get_param("~angle_min", self.angle_min)
        self.throttle_max = rospy.get_param("~throttle_max", self.throttle_max)
        self.throttle_min = rospy.get_param("~throttle_min", self.throttle_min)
        self.servo_gain = rospy.get_param("~servo_gain", self.servo_gain)
        self.servo_offset = rospy.get_param("~servo_offset", self.servo_offset)

    def write_params_to_server(self):
        rospy.set_param("~servo_channel", self.servo_channel)
        rospy.set_param("~motor_forward_channel", self.motor_forward_channel)
        rospy.set_param("~motor_backward_channel", self.motor_backward_channel)
        rospy.set_param("~in_1_channel", self.in_1_channel)
        rospy.set_param("~in_2_channel", self.in_2_channel)
        rospy.set_param("~angle_max", self.angle_max)
        rospy.set_param("~angle_min", self.angle_min)
        rospy.set_param("~throttle_max", self.throttle_max)
        rospy.set_param("~throttle_min", self.throttle_min)
        rospy.set_param("~servo_gain", self.servo_gain)
        rospy.set_param("~servo_offset", self.servo_offset)

    def write_params_to_file(self, file_path):
        with open(file_path, "w") as file_handle:
            data = dict(
                servo_channel=self.servo_channel,
                motor_forward_channel=self.motor_forward_channel,
                motor_backward_channel=self.motor_backward_channel,
                in_1_channel=self.in_1_channel,
                in_2_channel=self.in_2_channel,
                angle_max=self.angle_max,
                angle_min=self.angle_min,
                throttle_max=self.throttle_max,
                throttle_min=self.throttle_min,
                servo_gain=self.servo_gain,
                servo_offset=self.servo_offset,
            )
            yaml.dump(data, file_handle, default_flow_style=False)


if __name__ == "__main__":
    rospy.init_node("motor_node")
    rospy.loginfo("[{}] Starting...".format(rospy.get_name().strip("/")))
    node = MotorController()
    server = Server(motorConfig, node.reconfigure_callback)
    rospy.spin()
