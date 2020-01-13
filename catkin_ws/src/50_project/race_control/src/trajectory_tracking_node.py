#!/usr/bin/env python
import rospy
from picar_msgs.msg import CarCmd
from velocity_controller import VelocityController


class TrajectoryTrackingNode(object):
    """Race controller ROS node."""

    def __init__(self):
        self.publishers = {}
        self.subscribers = {}

        # register all publishers
        self.init_publishers()
        self.init_subscribers()
        self.tracking = VelocityController(10)
        self.cur_time = rospy.Time.now()
        self.start_time = self.cur_time
        self.desired_vel = None
        self.desired_angle = None

    def init_publishers(self):
        self.publishers["car_cmd"] = rospy.Publisher(
            "~car_cmd",
            CarCmd,
            queue_size=1)

    def init_subscribers(self):
        self.subscribers["desired_vals"] = rospy.Subscriber("~des_motor_cmd",
                                                            CarCmd,
                                                            self.update_des_values(),
                                                            queue_size=1)
        # TODO check message
        self.subscribers["vel_meas"] = rospy.Subscriber("~encoder_msg",
                                                        Point32,
                                                        self.update_current_velocity(),
                                                        queue_size=1)

    def update_des_values(self, message):
        self.desired_vel = message.velocity
        self.desired_angle = message.angle
        # frequency of controller is same as trajectoryplanner
        self.publish_car_cmd()

    def update_current_velocity(self, message):
        self.cur_vel = message  # TODO how does the message look like?

    def publish_car_cmd(self):
        cur_time = rospy.get_rostime() - self.start_time
        cur_time = cur_time.secs + float(cur_time.nsecs * 1e-9)
        message = CarCmd()
        message.header.stamp = rospy.get_rostime()
        if self.desired_vel is None or self.cur_vel is None:
            return

        des_vel = self.desired_vel
        cur_vel = self.cur_vel

        if cur_time > 10:
            message.velocity = self.tracking.get_velocity_output(cur_vel, des_vel) # call velocity controller
        else:
            message.velocity = 0.0

        message.angle = self.desired_angle
        self.publishers["car_cmd"].publish(message)
        self.rate.sleep()


def main():
    """Starts the lane control node"""
    rospy.init_node("trajectory_tracking_node")
    TrajectoryTrackingNode()
    rospy.spin()


if __name__ == "__main__":
    main()
