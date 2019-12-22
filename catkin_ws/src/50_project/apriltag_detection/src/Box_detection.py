import numpy as np
import apriltag
from Box import Box


class BoxDetector:
    tvec = None
    rvec = None

    def __init__(self, boxes):

        if not isinstance(boxes, list):
            raise Exception('boxes must be !LIST! of type Box')
        for box in boxes:
            if not isinstance(box, Box):
                raise Exception('boxes must be list of type !BOX!')
        self.boxes = boxes

        # create apriltag detector object
        self.detector = apriltag.Detector()

        # create dictionary of all used tags
        tag_dict_tmp = []
        for box in boxes:
            for tag in box.get_tags():
                # check if id is already taken
                tag_dict_tmp.append((tag.tag_id, tag))
        # FIXME check if Tag Id is used multiple times!
        self.tag_dict = dict(tag_dict_tmp)

    def detect_boxes(self, image_gray, mtx, dist):
        """ run tag detection, calculate tvec and rvec of tags and assign tvec and rvec to individual boxes"""

        result = self.detector.detect(image_gray)

        # clear pose of tag
        for tag_id, tag in self.tag_dict.items():
            tag.clear_pose()

        # determine pose of tags
        if result:
            for apriltag in result:
                detected_id = apriltag.tag_id
                # determine pose of detected AND used apriltags
                if detected_id in self.tag_dict:
                    tag = self.tag_dict[detected_id]

                    # get important information from detector
                    quality = apriltag.decision_margin
                    corners = apriltag.corners
                    center = apriltag.center

                    # calculate pose of tags
                    tag.calc_pose(quality, corners, center, mtx, dist)

        for box in self.boxes:
            pass
            # TODO determine pose of Box

    def draw_boxes(self, image, mtx, dist):
        """ draws edges around detected boxes """
        for box in self.boxes:
            box.draw_box(image, mtx, dist)
