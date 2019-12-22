import numpy as np
import cv2


def draw_axis(img, corners, imgpts):
    try:
        corner = tuple(corners.ravel())
        cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 2)
        cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 2)
        cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 2)
    except Exception as exc:
        pass


class Tag:

    def __init__(self, tag_id, tag_size, tvecref, rvecref):
        self.tag_id = int(tag_id)
        self.tvecref = tvecref
        self.rvecref = rvecref

        tag_size = float(tag_size)
        self.__calc_obj_corners(tag_size)

        self.clear_pose()

        self.parent = None

        self.axis = np.float32([[0.03825, 0, 0], [0, 0.03825, 0], [0, 0, -0.03825]]).reshape(-1, 3)

    def __calc_obj_corners(self, tag_size):
        """ determine corners in 3d of tag"""
        self.corners3D = np.array([[-tag_size / 2, tag_size / 2, 0],
                                   [tag_size / 2, tag_size / 2, 0],
                                   [tag_size / 2, -tag_size / 2, 0],
                                   [-tag_size / 2, -tag_size / 2, 0]], np.float32)

    def clear_pose(self):
        """ clear pose of tag"""
        self.corners_image = None
        self.center_image = None
        self.quality = 0
        self.tvec = None
        self.rvec = None

    def calc_pose(self, quality, corners_image, center_image, mtx, dist):
        """ calculates the pose (tvec,rvec) of tag """

        # apply image coordinates and reshape for drawing
        self.center_image = np.array(center_image, np.int32)
        self.corners_image = np.array(corners_image, np.int32).reshape((-1, 1, 2))

        self.quality = quality

        # reshape corners_image for PnP solver
        corners_image = np.array(corners_image, dtype='float32')
        corners_image = np.expand_dims(corners_image, axis=1)
        _, rvecs, tvecs, inliers = cv2.solvePnPRansac(self.corners3D, corners_image, mtx, dist)

        self.rvec = rvecs
        self.tvec = tvecs

    def draw_tag(self, image, mtx, dist):
        """ draws a box around the tag with a coordinate sytsem and shows tag id """
        if self.corners_image is not None:
            # draw corners of apriltags
            cv2.polylines(image, [self.corners_image], True, (0, 0, 255))

            # draw centers of apriltags with ID
            cv2.circle(image, (self.center_image[0], self.center_image[1]), 3, (0, 255, 0), -1)

            ID = "ID#" + str(self.tag_id)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(image, ID, (self.center_image[0] + 15, self.center_image[1]), font, 1, (0, 255, 0), 2,
                        cv2.LINE_AA)

            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(self.axis, self.rvec, self.tvec, mtx, dist)

            draw_axis(image, self.center_image, imgpts)

    def set_parent(self, parent):
        self.parent = parent
