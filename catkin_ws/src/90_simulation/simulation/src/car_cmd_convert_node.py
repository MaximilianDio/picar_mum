#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from picar_msgs.msg import JointCmd


class CarCmdConvertNode(object):
    def __init__(self):
        self.pub = rospy.Publisher("motor_node/car_cmd", JointCmd, queue_size=1)
        self.vel_sub = rospy.Subscriber("~vel_cmd", Float32,
                                    self.vel_cmd_cb, queue_size=1)
        self.steering_sub = rospy.Subscriber("~steering_cmd", Float32,
                                             self.steering_cmd_cb, queue_size=1)

        self.vel = 0.0
        self.angle = 0.0

    def vel_cmd_cb(self, msg):
        """

        :type msg: Float32
        """
        self.vel = msg.data
        self.pub_msg()

    def steering_cmd_cb(self, msg):
        self.angle = msg.data
        self.pub_msg()

    def pub_msg(self):
        out_msg = JointCmd()
        out_msg.angle = self.angle
        out_msg.speed = self.vel
        self.pub.publish(out_msg)


if __name__ == "__main__":
    rospy.init_node("car_cmd_convert_node")
    n = CarCmdConvertNode()
    rospy.spin()
