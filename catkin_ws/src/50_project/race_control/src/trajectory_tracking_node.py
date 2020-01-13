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
        self.tracking = VelocityController(10)
        self.start_time = rospy.Time.now()

        #self.desired_vals = rospy.Subscriber("~des_motor_cmd",
         #                                Point32,
          #                               self.pose_cb,
           #                              queue_size=1)



    def init_publishers(self):
        self.publishers["car_cmd"] = rospy.Publisher(
            "~car_cmd",
            CarCmd,
            queue_size=1)

    def publish_car_cmd(self, cur_vel, des_vel, cur_time, cur_angle):
        while not rospy.is_shutdown():
            message = CarCmd()
            message.header.stamp = rospy.get_rostime()

            if cur_time > 10:
                message.velocity = self.tracking.get_velocity_output(cur_vel, des_vel)
            else:
                message.velocity = 0.0

            message.angle = cur_angle
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
