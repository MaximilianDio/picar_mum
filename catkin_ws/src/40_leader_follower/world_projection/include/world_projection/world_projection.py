#!/usr/bin/env python
import numpy as np
from geometry_msgs.msg import Point32
from image_geometry import PinholeCameraModel
from picar_common.picar_common import FisheyeCameraModel


def normalize_image_coordinates(pixel_coord, image_width, image_height):
    """Computes normalized pixel coordinates.

    Args:
        pixel_coord (list/tuple): Coordinates of the pixel in uv-coordinate
            frame.
        image_width (int): Width of the image in pixels.
        image_height (int): Height of the image in pixels.

    Returns (numpy.ndarray): Numpy array of normalized pixel coordinates.

    """
    x_normalized = float(pixel_coord[0]) / (image_width - 1)
    y_normalized = float(pixel_coord[1]) / (image_height - 1)
    return np.array([x_normalized, y_normalized])


class WorldProjector(object):
    def __init__(self, camera_info, h_matrix, distorted_input=True):
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

        # normalize coordinates
        pixel_normalized = normalize_image_coordinates(
            pixel_coord,
            self.camera.width,
            self.camera.height)

        point = self._fake_project(pixel_normalized)

        return point

    def _project_normalized_pixel(self, pixel_normalized):
        """ TODO: WRITE CODE TO COMPUTE THE PIXEL'S CORRESPONDING World PLANE POINT

        Args:
            pixel_normalized: Normalized pixel coordinate

        Returns: 3D-coordinates of the projection expressed in the vehicle's
            frame

        """
        point = Point32()

        # YOUR CALCULATION NEEDS TO BE DONE HERE. REPLACE THE ASSIGNMENT
        # OF THE POINT'S COORDINATES BY REASONABLE ASSIGNMENTS OF YOUR World
        # PROJECTION.
        point.x = 0.0
        point.y = 0.0
        point.z = 0.0

        return point

    def _fake_project(self, pixel_normalized):
        """
        THIS METHOD IS JUST A DUMMY. REPLACE ALL CALLS OF THIS METHOD BY
        '_project_normalized_pixel()'

        Args:
            pixel_normalized:

        Returns:

        """
        point = Point32()
        point.x = 1.0 - pixel_normalized[1]
        point.y = 0.5 - pixel_normalized[0]
        point.z = 0.0
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
