#!/usr/bin/env python
import rospy
from picar_msgs.msg import WheelSpeedStamped
from std_msgs.msg import Float32
from velocity_estimator import VelocityEstimator
from picar.parameters import Wheel
from geometry_msgs.msg import Quaternion


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
        try:
            velocity.data = self.velocity_estimator.getCOMvel([input_msg.rear_left, input_msg.rear_right])
        except AttributeError:
            velocity.data = self.velocity_estimator.getCOMvel([input_msg.w, input_msg.w]) # map omega float64 value in both list places

        # velocity.header.stamp = rospy.Time.now()
        self.publishers["velocity_estimated"].publish(velocity)
        self.rate.sleep()

    def init_subscribers(self):
        """ initialize ROS subscribers and stores them in a dictionary"""
        # Subscription to encoder data - angular velocity
        try:
            rospy.Subscriber("~input_encoder_data", WheelSpeedStamped, self.encoder_callback)
        except rospy.exceptions.TransportException as e:
            print("unable to create subscriber transport with msg:WheelSpeedStamped. Will try again -> msg:Quaternion", e)
            rospy.Subscriber("~input_encoder_data", Quaternion, self.encoder_callback) # input comes as float64 on .w

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
