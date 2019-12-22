import numpy as np


class Box:
    __tvec = None
    __rvec = None
    __corners = None

    def __init__(self, tags, dimension):
        self.__tags = tags

        assert len(dimension) == 3, "dimension must have 3 entries"

        self.__dimension = dimension

        # add parent to tag
        for tag in tags:
            tag.set_parent(self)

        self.__calc_corners()

    def get_tags(self):
        return self.__tags

    def get_pose(self):
        pass

    def draw_box(self, image, mtx, dist):
        for tag in self.__tags:
            tag.draw_tag(image, mtx, dist)

        # TODO draw the edges of the corners
        pass

    def __calc_corners(self):
        # length of box edges
        l_x = self.__dimension[0]
        l_y = self.__dimension[1]
        l_z = self.__dimension[2]
        # TODO calculate the positions of the corners relative to the origin of the coordinate system
        self.__corners = np.array([[0, 0, 0], [0, l_y, 0], [0, 0, l_z], [0, l_y, l_z],
                                   [l_x, 0, 0], [l_x, l_y, 0], [l_x, 0, l_z], [l_x, l_y, l_z]])
