#!/usr/bin/python

from picar_msgs.msg import CarCmd
from picar_msgs.srv import SetValue
from velocity_controller import *
from std_msgs.msg import Float32
import rospy

class VelocityTrackingNode(object):

    def __init__(self):
        self.publishers = {}
        self.init_subscribers()
        self.controller = VelocityController(0.12, 0.025)

        self.velocity = 0.0
        self.angle = 100.0  # absolute value
        self.cur_vel = 0.0

        self.publisher = rospy.Publisher("~car_cmd",
                                         CarCmd,
                                         queue_size=1)

        self.service = rospy.Service("~set_p_gain",
                                     SetValue,
                                     self.set_p_gain)

        self.service = rospy.Service("~set_d_gain",
                                     SetValue,
                                     self.set_d_gain)

    def update_current_velocity(self, message):
        self.cur_vel = message.data

    def init_subscribers(self):
        rospy.Subscriber("~velocity_estimated",
                         Float32,
                         self.update_current_velocity,
                         queue_size=1)

        rospy.Subscriber("~trajectory_data",
                         CarCmd,
                         self.pub_message,
                         queue_size=1)

    def set_p_gain(self, message):
        self.controller.kp = message.value
        return 1

    def set_d_gain(self, message):
        self.controller.kd = message.value
        return 1

    def pub_message(self, trajectory_data):
        car_cmd = CarCmd()

        des_vel = trajectory_data.velocity
        car_cmd.angle = trajectory_data.angle

        car_cmd.velocity = self.controller.get_velocity_output(self.cur_vel, des_vel)

        self.publisher.publish(car_cmd)


def main():
    rospy.init_node("velocity_tracking_node")
    VelocityTrackingNode()
    rospy.spin()


if __name__ == "__main__":
    main()
