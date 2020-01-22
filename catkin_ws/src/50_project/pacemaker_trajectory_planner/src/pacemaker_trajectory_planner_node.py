#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

if __name__ == "__main__":
    rate = 50  # Hz

    rospy.init_node("pacemaker_trajectory_planner")

    pace_maker = rospy.Publisher("~clock", Float32, queue_size=1)

    timer = rospy.Rate(rate)
    while not rospy.is_shutdown():
        time = rospy.get_rostime()
        time_msg = Float32()
        time_msg.data = time.secs + time.nsecs * 1e-9
        pace_maker.publish(time_msg)
        timer.sleep()
