# adaption of https://github.com/strawlab/image_pipeline/blob/master/camera_calibration/src/camera_calibration/calibrator.py
import math
import cv2
import numpy as np

def pairwise_minimum(sequence_1, sequence_2):
    return [min(a,b) for (a, b) in zip(sequence_1, sequence_2)]


def pairwise_maximum(sequence_1, sequence_2):
    return [max(a, b) for (a, b) in zip(sequence_1, sequence_2)]


class ReturnObject:
    def __init__(self, image):
        self.image = image
        self.parameters = tuple()
        self.good_sample = False


class Chessboard:
    def __init__(self, columns, rows, square_size):
        self.columns = columns
        self.rows = rows
        self.square_size = square_size
        self.size = (columns, rows)


class Camera:
    def __init__(self, K, D, P, (width, height), distortion_model):
        self.K = K
        self.D = D
        self.P = P
        self.width = width
        self.height = height
        self.size = (width, height)
        self.distortion_model = distortion_model


class Calibrator:
    def __init__(self, size, square, fisheye):
        self.chessboard = Chessboard(size[0], size[1], square)
        self.fisheye = fisheye
        self.database = []
        self.all_corners = []
        self.refine_criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            100,
            0.001,
        )
        self.calibrated = False
        self.parameter_names = ["x", "y", "size", "skew"]
        self.parameter_ranges = [0.7, 0.7, 0.4, 0.5]
        self.image_size = (0, 0)
        self.max_images = 40
        self.good_enough = False
        self.camera = None
        self.map1 = None
        self.map2 = None

    def get_corners(self, image, refine=True):
        chessboard = self.chessboard
        height, width = image.shape[0:2]
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
                 + cv2.CALIB_CB_NORMALIZE_IMAGE
                 + cv2.CALIB_CB_FAST_CHECK)

        success, corners = cv2.findChessboardCorners(image_gray,
                                                     chessboard.size,
                                                     flags=flags)

        border = 8
        if success:
            if not all([(border < x < (width - border)) and
                        (border < y < (height - border)) for
                        (x, y) in corners[:,0]]):
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

        return success, corners


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

    def get_outer_corners(self, corners):
        n_corners = corners.shape[0]
        x_dimension = self.chessboard.columns
        y_dimension = self.chessboard.rows
        if not (n_corners == x_dimension*y_dimension):
            raise Exception("Invalid number of corners!\n"
                            "rows: {} cols: {} corners: {}".format(
                y_dimension, x_dimension, n_corners)
            )

        bottom_right = np.array(corners[0, 0])
        bottom_left = np.array(corners[x_dimension-1, 0])
        top_left = np.array(corners[-1, 0])
        top_right = np.array(corners[-x_dimension, 0])
        return bottom_right, bottom_left, top_left, top_right

    def get_skew(self, corners):
        bottom_right, bottom_left, top_left, _ = self.get_outer_corners(corners)

        def angle(a, b, c):
            ab = b-a
            ac = c-a

            return (math.acos(np.dot(ab, ac)
                    / (np.linalg.norm(ab)*np.linalg.norm(ac))))

        skew_angle = angle(bottom_left, bottom_right, top_left)
        skew = min(1.0,
                   2.0*abs(math.pi/2 - skew_angle)
                   )
        return skew

    def get_area(self, corners):
        bottom_right, bottom_left, top_left, top_right = self.get_outer_corners(
            corners
        )
        a = top_right - top_left
        b = bottom_right - bottom_left
        c = bottom_left - top_left

        p = b + c
        q = a + b
        return abs(p[0]*q[1] - p[1]*q[0]) / 2.0

    def get_parameters(self, corners, image_size):
        width = image_size[0]
        height = image_size[1]

        x_list = [x for (x, y) in corners[:, 0]]
        y_list = [y for (x, y) in corners[:, 0]]

        area = self.get_area(corners)

        border_length = math.sqrt(area)

        x = min(1.0,
                max(0.0,
                    (np.mean(x_list) - border_length/2) / (width-border_length)
                    )
                )

        y = min(1.0,
                max(0.0,
                    (np.mean(y_list) - border_length/2) / (height-border_length)
                    )
                )

        size = math.sqrt(area / (width*height))
        skew = self.get_skew(corners)

        parameters = (x, y, size, skew)
        return parameters

    def is_good_sample(self, parameters):
        if not self.database:
            return True

        def parameter_distance(p1, p2):
            return sum([abs(a-b) for (a, b) in zip(p1, p2)])

        database_parameters = [sample[0] for sample in self.database]
        minimal_distance = min(
            [parameter_distance(parameters, p) for p in database_parameters]
        )

        res = minimal_distance > 0.2

        return res

    def compute_good_enough(self):
        if not self.database:
            return None

        parameters = [sample[0] for sample in self.database]
        min_parameters = reduce(pairwise_minimum, parameters)
        max_parameters = reduce(pairwise_maximum, parameters)
        min_parameters = [min_parameters[0], min_parameters[1], 0.0, 0.0]

        progress = [min((high-low) / r, 1.0) for (low, high, r) in
                    zip(min_parameters, max_parameters, self.parameter_ranges)]

        self.good_enough = ((len(self.database) >= self.max_images) or
                            all([p == 1.0 for p in progress]))

        return zip(self.parameter_names,
                   min_parameters,
                   max_parameters,
                   progress)

    def undistort(self, image):
        if self.map1 is not None and self.map2 is not None:
            output = cv2.remap(image, self.map1, self.map2, cv2.INTER_LINEAR)
            return output
        else:
            raise Exception("undistort called before undistort map was "
                            "initialized.")

    def get_object_points(self):
        rows = self.chessboard.rows
        columns = self.chessboard.columns
        square = self.chessboard.square_size
        object_points = np.zeros((1, rows*columns, 3), np.float32)
        # object_points[0, :, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
        for row in range(rows):
            for col in range(columns):
                object_points[0, row*columns+col, :2] = np.array([col, row],
                                                                dtype=np.float32)
        object_points = object_points*np.float32(self.chessboard.square_size)

        return object_points

    def calibrate(self):
        if self.fisheye:
            self._calibrate_fisheye()
        else:
            self._calibrate_pinhole()

    def change_projection_matrix(self, scale, use_new_camera_matrix):
        if not self.calibrated:
            return
        if not use_new_camera_matrix:
            self.camera.P = self.camera.K
        else:
            if self.fisheye:
                P = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                    self.camera.K,
                    self.camera.D,
                    self.image_size,
                    np.eye(3),
                    balance=min(1.0, max(0.0, scale)),
                )
                self.camera.P = P

            else:
                P, _ = cv2.getOptimalNewCameraMatrix(
                    self.camera.K,
                    self.camera.D,
                    self.image_size,
                    min(1.0, max(0.0, scale))
                )
                self.camera.P = P

        if self.fisheye:
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                self.camera.K,
                self.camera.D,
                np.eye(3),
                self.camera.P,
                self.image_size,
                cv2.CV_32FC1
            )
        else:
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                self.camera.K,
                self.camera.D,
                np.eye(3),
                self.camera.P,
                self.image_size,
                cv2.CV_32FC1
            )

    def _calibrate_fisheye(self):
        # https://bitbucket.org/amitibo/pyfisheye/src/default/fisheye/core.py
        self._remove_bad_images()
        object_points = self.get_object_points()
        object_points = [object_points] * len(self.all_corners)

        flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
                 + cv2.fisheye.CALIB_FIX_SKEW
                 + cv2.fisheye.CALIB_CHECK_COND)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30,
                    1e-6)


        ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(object_points,
                                                        self.all_corners,
                                                        self.image_size,
                                                        None,
                                                        None,
                                                        criteria=criteria,
                                                        flags=flags)

        P = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K,
            D,
            self.image_size,
            np.eye(3)
        )
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            K,
            D,
            np.eye(3),
            K,
            self.image_size,
            cv2.CV_32FC1)
        self.calibrated = True
        self.camera = Camera(K, D, K, self.image_size, "fisheye")

    def _calibrate_pinhole(self):
        object_points = self.get_object_points()
        object_points = [object_points] * len(self.all_corners)

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30,
                    1e-6)

        ret, K, D, rvecs, tvecs = cv2.calibrateCamera(object_points,
                                                      self.all_corners,
                                                      self.image_size,
                                                      None,
                                                      None,
                                                      criteria=criteria)

        P = cv2.getOptimalNewCameraMatrix(K, D, self.image_size, 0)

        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            K,
            D,
            np.eye(3),
            K,
            self.image_size,
            cv2.CV_32FC1)
        self.calibrated = True
        self.camera = Camera(K, D, K, self.image_size, "plumb_bob")

    def _remove_bad_images(self):
        object_points = self.get_object_points()
        object_points = [object_points] * len(self.all_corners)
        flags = cv2.fisheye.CALIB_CHECK_COND
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    1,
                    1e-6)
        for i in xrange(len(self.all_corners)-1, -1, -1):
            try:
                ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                    [object_points[i]],
                    [self.all_corners[i]],
                    self.image_size,
                    None,
                    None,
                    criteria=criteria,
                    flags=flags)
            except cv2.error:
                print("bad image")
                del self.all_corners[i]
                del object_points[i]

    def get_images(self):
        """Returns list of images used for calibration"""
        return [image for (parameter, image) in self.database]

    def get_calibration_data(self):
        """Returns calibration data K, D, P, image_size stored in camera object"""
        return self.camera

    def process_image(self, image):
        height, width = image.shape[0:2]
        self.image_size = (width, height)

        success, corners = self.get_corners(image)
        good_sample = False

        if self.calibrated:
            image = self.undistort(image)

        elif success:
            image = cv2.drawChessboardCorners(image, self.chessboard.size,
                                              corners, success)
            parameters = self.get_parameters(corners, (width, height))
            if self.is_good_sample(parameters):
                good_sample = True
                self.database.append((parameters, image.copy()))
                self.all_corners.append(corners.reshape(1, -1, 2))

        result = ReturnObject(image)
        result.parameters = self.compute_good_enough()
        result.good_sample = good_sample

        return result















