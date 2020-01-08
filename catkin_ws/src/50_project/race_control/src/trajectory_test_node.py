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
        self.publishers["car_cmd"] = rospy.Publisher(
            "~car_cmd",
            CarCmd,
            queue_size=1)

    def publish_car_cmd(self):
        pass
        while not rospy.is_shutdown():
            curTime = rospy.get_rostime()
            message = CarCmd()
            message.header.stamp = rospy.get_rostime()
            curTime = curTime.secs + float(curTime.nsecs*1e-9)
            print curTime
            message.velocity, message.angle = self.overtaker.get_feedforward_control(curTime)
            message.angle = message.angle*180/3.1415
            print message.velocity
            print message.angle
            self.publishers["car_cmd"].publish(message)
            self.rate.sleep()

def main():
    """Starts the lane control node"""
    rospy.init_node("trajectory_test_node")
    t = TrajectoryTestNode()
    t.publish_car_cmd()
    rospy.spin()


if __name__ == "__main__":
    main()
