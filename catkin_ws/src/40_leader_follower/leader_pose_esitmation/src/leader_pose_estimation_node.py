#!/usr/bin/python
import rospy
import json
from geometry_msgs.msg import Point32
from picar_msgs.msg import LeaderPose
from lane_pose_estimation.leader_pose_estimation import PoseEstimator


class PoseEstimatorNode(object):
    def __init__(self):
        self.estimator = PoseEstimator()
        self.sub_lines = rospy.Subscriber("~track_position",
                                          Point32,
                                          self.rcv_line_cb,
                                          queue_size=1)

        self.pub_pose = rospy.Publisher("~pose",
                                        LeaderPose,
                                        queue_size=1)

    def rcv_line_cb(self, msg):
        track_position = msg
        pose = LeaderPose()
        pose.d, pose.phi = self.estimator.estimate_pose(track_position)
        self.pub_pose.publish(pose)


if __name__ == "__main__":
    rospy.init_node("leader_pose_estimation_node")
    n = PoseEstimatorNode()
    rospy.spin()