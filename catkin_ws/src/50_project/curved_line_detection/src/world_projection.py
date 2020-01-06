#!/usr/bin/env python
import numpy as np
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


    def pixel2ground(self, pixel_coord, distorted_input=None):
        """Returns the ground projection of a pixel."""
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

        point = self._project_normalized_pixel(pixel_normalized)
        return point

    def _project_normalized_pixel(self, pixel_normalized):
        # create homogeneous coordinate
        pixel_coord = np.append(pixel_normalized, 1.0)
        # project to world
        ground_coord = np.dot(self.h_matrix, pixel_coord)
        # scale the coordinates appropriately -> transform to eucledian space
        ground_coord = ground_coord / ground_coord[2]

        return ground_coord


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
