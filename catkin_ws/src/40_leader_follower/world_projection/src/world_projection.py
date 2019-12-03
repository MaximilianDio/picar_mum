#!/usr/bin/env python
import numpy as np
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point32
from picar_common.picar_common import FisheyeCameraModel


def create_homogeneous_coordinate(pixel_coord, image_width, image_height):
    """Computes normalized pixel coordinates.

        Args:
            pixel_coord (list/tuple): Coordinates of the pixel in uv-coordinate
                frame.
            image_width (int): Width of the image in pixels.
            image_height (int): Height of the image in pixels.

        Returns (numpy.ndarray): Numpy array of normalized pixel coordinates.

        """
    x = float(pixel_coord[0]) / (image_width - 1)
    y = float(pixel_coord[1]) / (image_height - 1)
    z = 1.0
    return np.array([x, y, z], dtype=float)


class WorldProjector(object):
    def __init__(self, camera_info, h_matrix, p_matrix, distorted_input=True):

        """
        The WorldProjector can rectify pixels or the whole image and the
        World coordinates of a pixel by using the homography matrix H

        Args:
            camera_info:
            h_matrix: Homography matrix, that maps normalized undistorted pixel
                coordinates into the world plane.
            distorted_input: Should only be False if simulation without
                simulated camera distortion is used.
        """
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
        self.h_matrix = h_matrix
        self.p_matrix = p_matrix

        self.z_projection = 0.1

    def pixel2world(self, pixel_coord, distorted_input=None):
        """Returns the world projection of a pixel.

        For distorted input the undistorted coordinates of the pixel will be
        calculcated. Afterwards the pixel will get noramlized and projected to
        the world plane by applying the homography matrix.

        Args:
            pixel_coord (tuple/list/numpy.ndarray): Pixel coordinates of the
                point that will be projected into the world plane.
            distorted_input (bool): Can be set to override the default value
                defined when creating the instance.

        Returns: Coordinates of the pixel projected into the world plane
            expressed in the vehicles coordinate system.

        """
        # use default value if 'distorted_input' is not set
        if distorted_input is None:
            distorted_input = self.distorted_input
        if distorted_input:
            pixel_coord = self.rectify_pixel(pixel_coord)
        pixel_coord = np.array(pixel_coord)

        # create homogeneous image coordinate
        pixel_coord = create_homogeneous_coordinate(pixel_coord,
                                                    self.camera.width,
                                                    self.camera.height)

        # separate projection matrix
        p1 = self.p_matrix[:, 0]
        p2 = self.p_matrix[:, 1]
        p3 = self.p_matrix[:, 2]
        p4 = self.p_matrix[:, 3]

        # define homography
        H = np.column_stack((p1, p2, p4))

        # transform pixel coordinates in homogeneous world coordinates
        point_world = np.matmul(np.linalg.inv(H), (pixel_coord - p3 * self.z_projection))

        point = Point32()

        # YOUR CALCULATION NEEDS TO BE DONE HERE. REPLACE THE ASSIGNMENT
        # OF THE POINT'S COORDINATES BY REASONABLE ASSIGNMENTS OF YOUR GROUND
        # PROJECTION.
        point.x = point_world[0] / point_world[2]
        point.y =point_world[1] / point_world[2]
        point.z = self.z_projection

        # transform homogeneous coordinates to euclidean world coordinates
        return point

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
