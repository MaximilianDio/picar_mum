import apriltag
from Tag import Tag
import numpy as np


class AprilTagDetector:
    def __init__(self, tag_id, tag_size, tvec_tag2ref, rmat_tag2ref,
                 mtx, dist, tvec_cam, rmat_cam):
        # save camera matrix and distortion coeffitients
        self.mtx = mtx
        self.dist = dist
        self.tvec_cam = tvec_cam.reshape([3, 1])
        self.rmat_cam = rmat_cam.reshape([3, 3])

        # create Tag
        self.tag = Tag(tag_id, tag_size, tvec_tag2ref, rmat_tag2ref)

        # create apriltag detector object
        self.detector = apriltag.Detector()

    def __detect(self, image_gray):
        result = self.detector.detect(image_gray)

        if result:
            # go through all detected tags and check if used one was detected
            for apriltag in result:
                # detected tag ID
                detected_id = apriltag.tag_id

                if detected_id == self.tag.tag_id:
                    # extract information
                    quality = apriltag.decision_margin
                    corners = apriltag.corners
                    center = apriltag.center

                    # calculate pose of tags
                    self.tag.calc_pose(quality, corners, center, self.mtx, self.dist)

    def get_tag_pos(self, image_gray):
        # clear old pose of tag
        self.tag.clear_pose()

        # detect tag and calculate tag in camera system
        self.__detect(image_gray)

        # recalculate translation vector to world frame
        if self.tag.tvec is not None:
            return self.tvec_cam + np.dot(self.rmat_cam, self.tag.tvec)
        else:
            return None

    def draw_tag(self, image):
        self.tag.draw_tag(image)
