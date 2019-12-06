#!/usr/bin/env python
import numpy as np
from image_geometry import PinholeCameraModel
from picar_common.picar_common import FisheyeCameraModel
import warnings


def create_p(r_matrix, t_vector, intrinsic_matrix):
    extrinsic_matrix = np.concatenate((r_matrix, t_vector), axis=1)
    extrinsic_matrix = np.concatenate((extrinsic_matrix, np.array([[0, 0, 0, 1]])), axis=0)

    return np.matmul(intrinsic_matrix, extrinsic_matrix)


def create_h(p_matrix):
    p1 = p_matrix[:, 0]
    p2 = p_matrix[:, 1]
    p4 = p_matrix[:, 3]

    # define homography
    return np.column_stack((p1, p2, p4))


def euclidean2homogeneous(poinR2):
    return np.array([poinR2[0], poinR2[1], 1], dtype=float)


class WorldProjector(object):
    def __init__(self, camera_info, r_matrix, t_matrix, k_matrix, distorted_input=True):

        # TODO import projection_z as parameter
        # defines height of the projection plane
        self.projection_z = 0.05

        # camera data
        self.r_matrix = r_matrix
        self.t_matrix = t_matrix
        self.k_matrix = k_matrix

        # due to offset in projection space the z-part of the translatory vector t_vector has to be adjusted
        self.t_matrix[2] -= self.projection_z

        # calculate projection matrix and invertible homography (Z_world == 0)
        self.p_matrix = create_p(self.r_matrix, self.t_matrix, self.k_matrix)
        self.h_matrix = create_h(self.p_matrix)

        if camera_info is not None:
            # distortion related stuff
            self.distorted_input = distorted_input
            self.camera_info = camera_info
            self.fisheye = False
            if camera_info.distortion_model == "fisheye":
                self.camera = FisheyeCameraModel()
                self.camera.from_camera_info(camera_info)
                self.fisheye = True
            else:
                self.camera = PinholeCameraModel()
                self.camera.fromCameraInfo(camera_info)


    def pixel2world(self, pixel_coord, distorted_input=None):

        try:
            # use default value if 'distorted_input' is not set
            if distorted_input is None:
                distorted_input = self.distorted_input
            if distorted_input:
                pixel_coord = self.rectify_pixel(pixel_coord)
        except Exception as err:
            warnings.warn("rectification went wrong")

        pixel_coord = np.array(pixel_coord)
        pixel_coord = euclidean2homogeneous(pixel_coord)

        # transform pixel coordinates in homogeneous world coordinates
        point_world = np.matmul(np.linalg.inv(self.h_matrix), pixel_coord)
        # transform to euclidean space
        try:
            point_world = point_world / point_world[2]
        except ValueError:
            warnings.warn("dividing by zero error, point lies in infinity!")

        point_world[2] = self.projection_z

        return point_world

    def rectify_pixel(self, pixel_coord):
        """Calculates the rectified undistorted image coordinate of a pixel.

        Args:
            pixel_coord: Distorted pixel coordinates

        Returns: Undistorted pixel coordinates
        """
        if self.fisheye:
            return self.camera.undistort_point(pixel_coord)
        else:
            return self.camera.rectifyPoint(pixel_coord)

    def rectify_image(self, img):
        if self.fisheye:
            return self.camera.undistort_image(img)
        else:
            output = np.empty_like(img)
            self.camera.rectifyImage(img, output)
            return output
