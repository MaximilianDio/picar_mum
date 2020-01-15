#!/usr/bin/env python
import rospy
from picar_msgs.msg import WheelSpeedStamped
from std_msgs.msg import Float32
from velocity_estimator import VelocityEstimator
from picar.parameters import Wheel
from sensor_msgs.msg import Imu


class VelocityEstimation(object):
    def __init__(self):
        self.rate = rospy.Rate(100)  # TODO publish hertz rate anpassen
        self.publishers = {}
        self.velocity_estimator = VelocityEstimator(Wheel.wheel_diameter/2)
        self.encoder_data = [0.0, 0.0]

        self.velocity_est = Float32()

        # register all publishers
        self.init_subscribers()
        # register all publishers
        self.init_publishers()

    def correction_step(self, input_msg):
        self.encoder_data = [input_msg.rear_left, input_msg.rear_right]
        self.velocity_est.data = self.velocity_estimator.correction_step(self.encoder_data)
        self.publishers["velocity_estimated"].publish(self.velocity_est)

    def prediction_step(self, imudata):
        self.velocity_est.data = self.velocity_estimator.prediction_step(imudata)
        #velocity_est.header.stamp = rospy.Time.now()
        self.publishers["velocity_estimated"].publish(self.velocity_est)

    def init_subscribers(self):
        """ initialize ROS subscribers and stores them in a dictionary"""
        # Subscription to encoder data - angular velocity

        rospy.Subscriber("~input_encoder_data", WheelSpeedStamped, self.correction_step)
        rospy.Subscriber("~imu", Imu, self.prediction_step)


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
