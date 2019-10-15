import math
import numpy as np
import cv2
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
import ground_projection
from picar_common import FisheyeCameraModel


class Chessboard:
    def __init__(self,
                 columns,
                 rows,
                 square_size,
                 (x, y, z)=(0.46, -0.12, 0.0),
                 S=np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])):
        self.columns = columns
        self.rows = rows
        self.square_size = square_size
        self.size = (columns, rows)
        self.position = np.array([x, y, z]).reshape(3, 1)
        self.S = S


class Calibrator:

    t_vehicle_chessboard = np.array([[0.46, -0.12, 0.0]]).reshape(3, 1)
    S_vehicle_chessboard = np.array([[0, 1, 0],
                                     [1, 0, 0],
                                     [0, 0, -1]])

    refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                       3000,
                       0.00001)

    upscale_size = (2560, 1920)

    def __init__(self, size, square, camera_info):
        self.chessboard = Chessboard(size[0], size[1], square)

        self.camera_info = camera_info
        if camera_info.distortion_model == "fisheye":
            self.camera = FisheyeCameraModel()
            self.camera.from_camera_info(camera_info)
        else:
            self.camera = PinholeCameraModel()
            self.camera.fromCameraInfo(camera_info)

        dummy_H = np.empty((3, 3))
        self.ground_projector = ground_projection.GroundProjector(camera_info,
                                                                  dummy_H)

    def calibrate(self, image):
        success, corners = self.get_corners(image)
        if not success:
            return [np.zeros([3, 3])]*3

        height, width = image.shape[:2]

        source_points = np.copy(corners.reshape(-1, 2))
        for i in range(source_points.shape[0]):
            source_points[i] = self.ground_projector.rectify_pixel(
                source_points[i])

            source_points[i] = ground_projection.normalize_image_coordinates(
                source_points[i],
                width,
                height
            )

        source_points = np.array(source_points).reshape(-1, 2)
        # only x and y coordinate are needed for homography
        destination_points = self.get_object_points_vehicle_frame(
            self.chessboard)[:, :2]

        H, _ = cv2.findHomography(source_points,
                                  destination_points,
                                  cv2.RANSAC)

        source_points = np.copy(corners.reshape(-1, 2))
        for i in range(source_points.shape[0]):
            source_points[i] = self.ground_projector.rectify_pixel(
                source_points[i])
        destination_points = self.get_object_points_chessboard_frame(
            self.chessboard)
        # dont use distortion here, since the corners have been rectified
        _, rvecs, tvecs = cv2.solvePnP(
            destination_points,
            source_points,
            self.camera.P[:3, :3],
            None
        )

        t, S = self.get_camera_loc_rot(rvecs, tvecs, self.chessboard)

        roll_pitch_yaw = self.rotation_matrix_to_roll_pitch_yaw_degree(S)

        return H, S, t, roll_pitch_yaw

    def rotation_matrix_to_roll_pitch_yaw_degree(self, S):
        pitch_rad = np.arcsin(S[0, 2])
        roll_rad = np.arccos(S[2, 2]/np.cos(pitch_rad))
        yaw_rad = np.arccos(S[0, 0]/np.cos(roll_rad))
        rpy_rad = np.array([roll_rad, pitch_rad, yaw_rad])
        rpy_deg = rpy_rad * 180.0 / np.pi
        return rpy_deg.reshape([3, 1])

    def get_camera_loc_rot(self, rvecs, tvecs, chessboard):
        S_camera_chessboard = cv2.Rodrigues(rvecs)[0]
        t_camera_chessboard = tvecs
        S_chessboard_camera = S_camera_chessboard.transpose()
        t_chessboard_camera = -t_camera_chessboard

        camera_position_chessboard = S_chessboard_camera.dot(
            t_chessboard_camera
        )

        camera_position_vehicle = (chessboard.position
                                   + chessboard.S.dot(
                    camera_position_chessboard))

        S_vehicle_camera = np.matmul(chessboard.S, S_chessboard_camera)

        return camera_position_vehicle, S_vehicle_camera

    def get_corners(self, image, refine=True):
        chessboard = self.chessboard
        height, width = image.shape[0:2]
        scale = float(width)/self.upscale_size[0]
        image = cv2.resize(image, self.upscale_size)
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
                 + cv2.CALIB_CB_NORMALIZE_IMAGE
                 + cv2.CALIB_CB_FAST_CHECK)

        success, corners = cv2.findChessboardCorners(image_gray,
                                                     chessboard.size,
                                                     flags=flags)

        border = int(8/scale)
        if success:
            if not all([(border < x < (self.upscale_size[0] - border)) and
                        (border < y < (self.upscale_size[1] - border)) for
                        (x, y) in corners[:,0]]):
                print("not in border")
                success = False

        if success and refine:
            n_points = corners.shape[0]
            minimal_distance = self.minimal_distance(corners)
            window_size = max(1, int(minimal_distance*0.5))
            corners = cv2.cornerSubPix(image_gray, corners,
                                       (window_size, window_size),
                                       (-1, -1),
                                       self.refine_criteria)

            if not (corners.shape[0] == n_points):
                success = False
                print("Error during refinement!!!")
        if success:
            corners = corners*scale
        return success, corners

    def draw_corners(self, image, success, corners):
        image = cv2.drawChessboardCorners(
            image,
            self.chessboard.size,
            corners,
            success)
        return image

    def minimal_distance(self, corners):
        min_distance_squared = float("inf")
        n_points, _, _ = corners.shape
        # TODO: This can be done smarter. You dont have to compute distance
        #  between each and every point
        for i in range(n_points):
            for j in range(n_points):
                if not (i == j):
                    min_distance_squared = min(
                        min_distance_squared,
                        self.points_distance_squared(corners[i][0],
                                                     corners[j][0]
                                                     ))

        min_distance = math.sqrt(min_distance_squared)
        return min_distance

    def points_distance(self, p1, p2):
        return math.sqrt(self.points_distance_squared(p1, p2))

    def points_distance_squared(self, p1, p2):
        return (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2

    def get_object_points_vehicle_frame(self, chessboard):
        """Returns 3d chessboard points in vehicle frame"""
        rows = chessboard.rows
        columns = chessboard.columns
        square = chessboard.square_size
        offset = chessboard.position[:2].ravel()

        object_points = np.zeros([rows*columns, 2], dtype=np.float32)
        for row in range(rows):
            for col in range(columns):
                object_points[row*columns + col] = np.array(
                    [row*square, col*square]) + offset

        world_points = np.zeros([object_points.shape[0],
                                 object_points.shape[1]+1],
                                dtype=np.float32)
        world_points[:, :-1] = object_points
        return world_points

    def get_object_points_chessboard_frame(self, chessboard):
        rows = chessboard.rows
        columns = chessboard.columns
        object_points = np.zeros([rows*columns, 3],
                                 np.float32)

        object_points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)

        object_points *= chessboard.square_size

        return object_points
