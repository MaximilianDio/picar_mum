#!/usr/bin/env python
import rospy
from picar_msgs.msg import WheelSpeedStamped


class VelocityEstimation(object):
    def __init__(self):
        self.pos_blue_ball = None
        self.pos_green_ball = None

        self._params = {}
        self.publishers = {}
        self.services = {}

        # register all publishers
        self.init_subscribers()
        # register all publishers
        self.init_publishers()

    def run(self):
        position = Point32()

        if self.pos_blue_ball is not None and self.pos_green_ball is not None:

            position.x = self.pos_blue_ball.x + 0.5 * (self.pos_green_ball.x - self.pos_blue_ball.x)
            position.y = self.pos_blue_ball.y + 0.5 * (self.pos_green_ball.y - self.pos_blue_ball.y)

        self.publishers["leader_relative_pos"].publish(position)

    def get_position_blue_ball_cb(self, data):
        self.pos_blue_ball = data
        self.run()

    def get_position_green_ball_cb(self, data):
        self.pos_green_ball = data

    def init_subscribers(self):
        """ initialize ROS subscribers and stores them in a dictionary"""
        # Subscription to encoder data - angular velocity
        rospy.Subscriber("~input_encoder_data", WheelSpeedStamped, self.get_position_blue_ball_cb)


    def init_publishers(self):
        """ initialize ROS publishers and stores them in a dictionary"""
        # COM velocity estimated on encoder data
        self.publishers["velocity_estimated"] = rospy.Publisher("~velocity_estimated",
                                                                              Point32,
                                                                              queue_size=1)


if __name__ == "__main__":
    rospy.init_node("velocity_estimation_node")
    LeaderPoseEstimation()
    rospy.spin()
