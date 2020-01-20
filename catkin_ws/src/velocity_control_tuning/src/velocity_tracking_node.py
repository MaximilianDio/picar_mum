#!/usr/bin/python

from picar_msgs.msg import CarCmd
from velocity_controller import *
from std_msgs.msg import Float32
import rospy


class VelocityTrackingNode(object):

    def __init__(self):
        self.publishers = {}
        self.init_subscribers()
        self.controller = VelocityController(0.04, 0.013)

        self.velocity = 0.0
        self.angle = 0.0  # absolute value
        self.cur_vel = 0.0

        self.publisher = rospy.Publisher("~car_cmd",
                                         CarCmd,
                                         queue_size=1)

    def init_subscribers(self):
        rospy.Subscriber("~velocity_estimated",
                         Float32,
                         self.update_current_velocity,
                         queue_size=1)

        rospy.Subscriber("~trajectory_data",
                         CarCmd,
                         self.pub_message,
                         queue_size=1)

    def update_current_velocity(self, message):
        self.cur_vel = message.data

    def pub_message(self, trajectory_data):
        car_cmd = CarCmd()

        des_vel = trajectory_data.velocity
        car_cmd.angle = trajectory_data.angle
        car_cmd.velocity = 0.0
        time = trajectory_data.header.stamp
        time = time.secs + float(time.nsecs * 1e-9)
        if time > 15:
            car_cmd.velocity = des_vel
        else:
            car_cmd.velocity = self.controller.get_velocity_output(self.cur_vel, des_vel)

        self.publisher.publish(car_cmd)


def main():
    rospy.init_node("velocity_tracking_node")
    VelocityTrackingNode()
    rospy.spin()


if __name__ == "__main__":
    main()
