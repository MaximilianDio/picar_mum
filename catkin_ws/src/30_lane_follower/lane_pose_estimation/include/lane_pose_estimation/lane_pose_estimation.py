#!/usr/bin/python

class PoseEstimator(object):
    def __init__(self):
        pass

    def estimate_pose(self, track_position):
        distance = track_position.y

        # currently not calculated
        phi = 0.0

        return distance, phi


