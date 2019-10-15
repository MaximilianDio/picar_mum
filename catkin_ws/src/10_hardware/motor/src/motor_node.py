#!/usr/bin/env python
import Adafruit_PCA9685
import yaml
import rospy
import picar_common.picar_common as picar_common
from picar_msgs.msg import CarCmd
import time


class MotorController(object):
    def __init__(self, freq=200, address=0x40):
        # find config file
        config_file_name = picar_common.get_param("~config_file_name", "default")
        config_file_path = picar_common.get_config_file_path("motor", config_file_name)

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
        self.timer = rospy.Timer(rospy.Duration(1), self.msg_timeout_cb)

        # init pwm
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(freq)
        self.pwm.set_pwm(self.params.IN_1_CHANNEL, 0, 4095)
        self.pwm.set_pwm(self.params.IN_2_CHANNEL, 0, 4095)
        self.pwm.set_pwm(self.params.SERVO_CHANNEL, 0, 1228)
        self.pwm.set_pwm(self.params.MOTOR_FORWARD_CHANNEL, 0, 0)
        self.pwm.set_pwm(self.params.MOTOR_BACKWARD_CHANNEL, 0, 0)

        # handle shutdowns properly -> release motor
        rospy.on_shutdown(self.on_shutdown)

        # subscribe to motor commands
        self.sub_car_cmd = rospy.Subscriber("~car_cmd",
                                            CarCmd,
                                            self.car_cmd_callback,
                                            queue_size=1)

    def car_cmd_callback(self, msg):
        self.msg_timeout = False
        throttle = msg.velocity
        if throttle < 0:
            throttle = -int(abs(throttle*self.params.THROTTLE_MIN))
        else:
            throttle = int(throttle*self.params.THROTTLE_MAX)

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
        self.pwm.set_pwm(self.params.SERVO_CHANNEL, 0, int(pwm))

    def set_steering_angle(self, angle):
        # angle in radian
        # left steering is positive

        # limit angle to [ANGLE_MIN; ANGLE_MAX]
        angle = max(angle, self.params.ANGLE_MIN)
        angle = min(angle, self.params.ANGLE_MAX)

        pwm = int(float(self.params.SERVO_GAIN)*angle + self.params.SERVO_OFFSET)
        self.set_servo_pwm(pwm)

    def set_throttle_pwm(self, pwm):
        if pwm > self.params.THROTTLE_MAX:
            pwm = self.params.THROTTLE_MAX
        elif pwm < self.params.THROTTLE_MIN:
            pwm = self.params.THROTTLE_MIN

        if pwm < 0:
            self.pwm.set_pwm(self.params.MOTOR_FORWARD_CHANNEL, 0, 0)
            self.pwm.set_pwm(self.params.MOTOR_BACKWARD_CHANNEL, 0, -pwm)
        else:
            self.pwm.set_pwm(self.params.MOTOR_BACKWARD_CHANNEL, 0, 0)
            self.pwm.set_pwm(self.params.MOTOR_FORWARD_CHANNEL, 0, pwm)

    def msg_timeout_cb(self, event):
        if self.msg_timeout:
            self.set_throttle_pwm(0)
            time.sleep(0.1)
            self.set_steering_angle(0.0)
        self.msg_timeout = True


class MotorParams(object):
    def __init__(self, params_file_path):
        self.SERVO_CHANNEL = 0
        self.MOTOR_FORWARD_CHANNEL = 0
        self.MOTOR_BACKWARD_CHANNEL = 0
        self.IN_1_CHANNEL = 0  # BACKWARD MODE
        self.IN_2_CHANNEL = 0  # FORWARD MODE

        self.ANGLE_MAX = 0
        self.ANGLE_MIN = 0
        self.THROTTLE_MIN = 0
        self.THROTTLE_MAX = 0

        self.SERVO_GAIN = 0.0
        self.SERVO_OFFSET = 0
        self.read_params_from_file(params_file_path)

    def read_params_from_file(self, file_path):
        with open(file_path, "r") as f:
            data = yaml.safe_load(f)
            self.SERVO_CHANNEL = data["servo_channel"]
            self.MOTOR_FORWARD_CHANNEL = data["motor_forward_channel"]
            self.MOTOR_BACKWARD_CHANNEL = data["motor_backward_channel"]
            self.IN_1_CHANNEL = data["in_1_channel"]
            self.IN_2_CHANNEL = data["in_2_channel"]
            self.ANGLE_MAX = data["angle_max"]
            self.ANGLE_MIN = data["angle_min"]
            self.THROTTLE_MAX = data["throttle_max"]
            self.THROTTLE_MIN = data["throttle_min"]
            self.SERVO_GAIN = data["servo_gain"]
            self.SERVO_OFFSET = data["servo_offset"]
            self.write_params_to_server()

    def read_params_from_server(self):
        self.SERVO_CHANNEL = rospy.get_param("~servo_channel", self.SERVO_CHANNEL)
        self.MOTOR_FORWARD_CHANNEL = rospy.get_param("~motor_forward_channel", self.MOTOR_FORWARD_CHANNEL)
        self.MOTOR_BACKWARD_CHANNEL = rospy.get_param("~motor_backward_channel", self.MOTOR_BACKWARD_CHANNEL)
        self.IN_1_CHANNEL = rospy.get_param("~in_1_channel", self.IN_1_CHANNEL)
        self.IN_2_CHANNEL = rospy.get_param("~in_2_channel", self.IN_2_CHANNEL)
        self.ANGLE_MAX = rospy.get_param("~angle_max", self.ANGLE_MAX)
        self.ANGLE_MIN = rospy.get_param("~angle_min", self.ANGLE_MIN)
        self.THROTTLE_MAX = rospy.get_param("~throttle_max", self.THROTTLE_MAX)
        self.THROTTLE_MIN = rospy.get_param("~throttle_min", self.THROTTLE_MIN)
        self.SERVO_GAIN = rospy.get_param("~servo_gain", self.SERVO_GAIN)
        self.SERVO_OFFSET = rospy.get_param("~servo_offset", self.SERVO_OFFSET)

    def write_params_to_server(self):
        rospy.set_param("~servo_channel", self.SERVO_CHANNEL)
        rospy.set_param("~motor_forward_channel", self.MOTOR_FORWARD_CHANNEL)
        rospy.set_param("~motor_backward_channel", self.MOTOR_BACKWARD_CHANNEL)
        rospy.set_param("~in_1_channel", self.IN_1_CHANNEL)
        rospy.set_param("~in_2_channel", self.IN_2_CHANNEL)
        rospy.set_param("~angle_max", self.ANGLE_MAX)
        rospy.set_param("~angle_min", self.ANGLE_MIN)
        rospy.set_param("~throttle_max", self.THROTTLE_MAX)
        rospy.set_param("~throttle_min", self.THROTTLE_MIN)
        rospy.set_param("~servo_gain", self.SERVO_GAIN)
        rospy.set_param("~servo_offset", self.SERVO_OFFSET)

    def write_params_to_file(self, file_path):
        with open(file_path, "w") as f:
            data = dict(
                servo_channel=self.SERVO_CHANNEL,
                motor_forward_channel=self.MOTOR_FORWARD_CHANNEL,
                motor_backward_channel=self.MOTOR_BACKWARD_CHANNEL,
                in_1_channel=self.IN_1_CHANNEL,
                in_2_channel=self.IN_2_CHANNEL,
                angle_max=self.ANGLE_MAX,
                angle_min=self.ANGLE_MIN,
                throttle_max=self.THROTTLE_MAX,
                throttle_min=self.THROTTLE_MIN,
                servo_gain=self.SERVO_GAIN,
                servo_offset=self.SERVO_OFFSET,
            )
            yaml.dump(data, f, default_flow_style=False)


if __name__ == "__main__":
    rospy.init_node("motor_node")
    rospy.loginfo("[{}] Starting...".format(rospy.get_name().strip("/")))
    n = MotorController()
    rospy.spin()



