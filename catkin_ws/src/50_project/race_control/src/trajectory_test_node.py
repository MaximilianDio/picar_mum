#!/usr/bin/env python
import rospy
from picar_msgs.msg import CarCmd
from trajectoryplanner import OvertakingTrajectory


class TrajectoryTestNode(object):
    """Race controller ROS node."""

    def __init__(self):
        self.rate = rospy.Rate(100)
        self.publishers = {}
        # register all publishers
        self.init_publishers()
        self.overtaker = OvertakingTrajectory()
        self.start_time = rospy.Time.now()

    def init_publishers(self):
        self.publishers["trajectory_cmd"] = rospy.Publisher(
            "~trajectory_data",
            CarCmd,
            queue_size=1)

    def publish_feedforward_values(self):
        while not rospy.is_shutdown():
            curTime = rospy.get_rostime() - self.start_time
            message = CarCmd()
            message.header.stamp = rospy.get_rostime()
            curTime = curTime.secs + float(curTime.nsecs * 1e-9)
            print "Time: " + str(curTime)
            if curTime > 12 and curTime <= 20:
                message.velocity = 0.5
                message.angle = 0.0
            elif curTime > 15:
                message.velocity, message.angle = self.overtaker.get_feedforward_control(curTime - 15)
            else:
                message.velocity = 0.0
                message.angle = 0.0
            self.publishers["trajectory_cmd"].publish(message)
            self.rate.sleep()


def main():
    """Starts the lane control node"""
    rospy.init_node("trajectory_test_node")
    t = TrajectoryTestNode()
    t.publish_feedforward_values()
    rospy.spin()


if __name__ == "__main__":
    main()
