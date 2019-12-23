import numpy as np
import cv2


def draw_edges(image, corners):
    corners = np.array(corners, dtype='int32').squeeze()
    cv2.polylines(image, [corners[0:4]], True, (0, 255, 255), 2)
    cv2.polylines(image, [corners[4:8]], True, (0, 255, 255), 2)
    for corner in corners:
        cv2.circle(image, (corner[0], corner[1]), 5, (0, 255, 255), -1)


def draw_axis(img, origin, imgpts):
    try:
        imgpts.astype('int32')
        origin = tuple(origin.astype('int32').ravel())
        cv2.line(img, origin, tuple(imgpts[0].ravel()), (255, 0, 0), 2)  # x
        cv2.line(img, origin, tuple(imgpts[1].ravel()), (0, 255, 0), 2)  # y
        cv2.line(img, origin, tuple(imgpts[2].ravel()), (0, 0, 255), 2)  # z
    except Exception as exc:
        pass


class Box:
    __tvec = None
    __rvec = None
    __corners3D = None

    def __init__(self, id, tags, dimension):
        self.__id = id
        self.__tags = tags

        assert len(dimension) == 3, "dimension must have 3 entries"

        self.__dimension = dimension

        # add parent to tag
        for tag in tags:
            tag.set_parent(self)

        self.__calc_corners()

        self.axis = np.float32([[dimension[0], 0, 0], [0, dimension[1], 0], [0, 0, dimension[2]]]).reshape(-1, 3)

    def get_tags(self):
        return self.__tags

    def get_pose(self):
        pass

    def draw_box(self, image, mtx, dist):
        # calculate a mean value for tvec and rvec
        i = 0
        rvec = np.zeros(shape=(3, 1))
        tvec = np.zeros(shape=(3, 1))
        for tag in self.__tags:
            if tag.rvec is not None and tag.tvec is not None:
                rvec = rvec + tag.quality * tag.rvec
                tvec = tvec + tag.quality * tag.tvec
                i += tag.quality

        # project 3D points to image plane
        try:
            rvec /= i
            tvec /= i

            # project box corners to image
            corners2D, jac = cv2.projectPoints(self.__corners3D, rvec, tvec, mtx, dist)
            corners2D = corners2D.astype('int32').reshape((-1, 1, 2))
            draw_edges(image, corners2D)

            # project coordinate axis to image
            imgpts, jac = cv2.projectPoints(self.axis, rvec, tvec, mtx, dist)
            draw_axis(image, corners2D[0], imgpts)

            ID = "OBSTACLE ID#" + str(self.__id)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(image, ID, (corners2D[0].ravel()[0] + 15, corners2D[0].ravel()[1]), font, 0.5, (255, 255, 0), 1,
                        cv2.LINE_AA)
        except ZeroDivisionError:
            # no tag was detected
            pass

        # draw tags above other stuff
        for tag in self.__tags:
            tag.draw_tag(image, mtx, dist)

    def __calc_corners(self):

        # length of box edges
        l_x = self.__dimension[0]
        l_y = self.__dimension[1]
        l_z = self.__dimension[2]
        self.__corners3D = np.array([[0, 0, 0], [0, 0, l_z], [0, l_y, l_z], [0, l_y, 0],
                                     [l_x, 0, 0], [l_x, 0, l_z], [l_x, l_y, l_z], [l_x, l_y, 0], ]).reshape(-1, 3, 1)
