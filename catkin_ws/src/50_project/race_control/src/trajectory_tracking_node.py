#!/usr/bin/env python
import rospy
from picar_msgs.msg import CarCmd
from velocity_controller import VelocityController


class TrajectoryTrackingNode(object):
    """Race controller ROS node."""

    def __init__(self):
        self.rate = rospy.Rate(100)
        self.publishers = {}
        self.subscribers = {}

        # register all publishers
        self.init_publishers()
        self.init_subscribers()
        self.tracking = VelocityController(10)
        self.cur_time = rospy.Time.now()
        self.start_time = self.cur_time

    def init_publishers(self):
        self.publishers["car_cmd"] = rospy.Publisher(
            "~car_cmd",
            CarCmd,
            queue_size=1)

    def init_subscribers(self):
        self.desired_vals = rospy.Subscriber("~des_motor_cmd",
                                        Point32,
                                        self.publish_car_cmd(self.cur_time),
                                        queue_size=1)

        self.meas_vals = rospy.Subscriber("~encoder_msg",
                                          Point32,
                                          self.publish_car_cmd(self.cur_time),
                                          queue_size=1)

    def publish_car_cmd(self):
        while not rospy.is_shutdown():
            cur_time = rospy.get_rostime() - self.start_time
            cur_time = cur_time.secs + float(cur_time.nsecs * 1e-9)
            message = CarCmd()
            message.header.stamp = rospy.get_rostime()
            des_vel = self.desired_vals.velocity
            cur_vel = self.meas_vals.encoder

            if cur_time > 10:
                message.velocity = self.tracking.get_velocity_output(cur_vel, des_vel)
            else:
                message.velocity = 0.0

            message.angle = self.desired_vals.angle
            self.publishers["car_cmd"].publish(message)
            self.rate.sleep()

def main():
    """Starts the lane control node"""
    rospy.init_node("trajectory_tracking_node")
    Tracking = TrajectoryTrackingNode()
    Tracking.publish_car_cmd()
    rospy.spin()


if __name__ == "__main__":
    main()
