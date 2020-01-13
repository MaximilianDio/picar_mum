#!/usr/bin/env python
import rospy
from picar_msgs.msg import WheelSpeedStamped
from std_msgs.msg import Float32
from velocity_estimator import VelocityEstimator
from picar.parameters import Wheel


class VelocityEstimation(object):
    def __init__(self):
        self.rate = rospy.Rate(100)  # TODO publish hertz rate anpassen
        self.publishers = {}
        self.velocity_estimator = VelocityEstimator(radius=Wheel.wheel_diameter/2)

        # register all publishers
        self.init_subscribers()
        # register all publishers
        self.init_publishers()

    def encoder_callback(self, input_msg):
        velocity = Float32()

        velocity.data = self.velocity_estimator.getCOMvel([input_msg.rear_left, input_msg.rear_right])

        self.publishers["velocity_estimated"].publish(velocity)
        #self.rate.sleep()

    def init_subscribers(self):
        """ initialize ROS subscribers and stores them in a dictionary"""
        # Subscription to encoder data - angular velocity

        rospy.Subscriber("~input_encoder_data", WheelSpeedStamped, self.encoder_callback)


    def init_publishers(self):
        """ initialize ROS publishers and stores them in a dictionary"""
        # COM velocity estimated on encoder data
        self.publishers["velocity_estimated"] = rospy.Publisher("~velocity_estimated",
                                                                Float32,
                                                                queue_size=1)
if __name__ == "__main__":
    rospy.init_node("velocity_estimation_node")
    VelocityEstimation()
    rospy.spin()
