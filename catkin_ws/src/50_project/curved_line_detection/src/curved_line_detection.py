import cv2
import numpy as np
from world_projection import WorldProjector
import math

# Constants for track bar
MIN_HUE = MIN_SATURATION = MIN_VALUE = 0
MAX_HUE = 180
MAX_SATURATION = MAX_VALUE = 255

MIN_NUM_STRIPES = 1
MAX_NUM_STRIPES = 30


def nothing(x):
    """ dummy function does nothing, used as dummy callback for trackbar"""
    pass


class Trackbar:
    """ window with trackbars based on parameter dictionary which has a list value of 3 entries"""

    def __init__(self, param_dict):

        # check dimension of dictionary values:
        for value in param_dict.values():
            if len(value) != 3:
                raise TypeError("dimension of values in parameter dictionary has to be 3")
        self.param_dict = param_dict

        self.__create_trackbar(param_dict)
        self.__update_trackbars()

    def __create_trackbar(self, param_dict):
        # dimensions of track bar window
        self.WIDTH = 300  # px
        self.HEIGHT = 600  # px

        # create black window
        self.trackbar_window = np.zeros((self.WIDTH, self.HEIGHT, 3), np.uint8)
        self.window_name = "trackbars"
        cv2.namedWindow(self.window_name)

        # create trackbars based on parameter dictionary with keys as trackbar name
        for name, param in self.param_dict.items():
            cv2.createTrackbar(name, self.window_name, param[0], param[2], nothing)
            cv2.setTrackbarMin(name, self.window_name, param[1])

    def __update_trackbars(self):
        """ show trackbar window """
        cv2.imshow(self.window_name, self.trackbar_window)
        cv2.waitKey(1)

    def update_trackbar_pos(self):
        """ update the dictionary"""
        for parameter, value in self.param_dict.items():
            self.param_dict[parameter][0] = cv2.getTrackbarPos(parameter, self.window_name)


class CurvePointExtractor:

    def __init__(self, hsv_mask_interval, num_stripes, roi_interval, use_trackbar=True):
        """
        :param hsv_mask_interval: 2 by 3 numpy array which contains low HSV values and high HSV values for masking the
        region of interest
        :param num_stripes: amount of stripes in which the roi will be disected
        :param roi_interval: list/ tuple of cropping parameters with length 3
         with values between 0 an 1:
         (normalized width, normalized cropping border bottom, normalized cropping border top)
        :param use_trackbar: flag to show trackbar and mask
        """

        self.EDGE_THRESHOLD = 200

        if not isinstance(num_stripes, int) or num_stripes < MIN_NUM_STRIPES:
            raise ValueError("num_stripes must be integer > 0")

        self.__num_stripes = num_stripes

        try:
            if hsv_mask_interval.shape != (2, 3):
                raise ValueError("hsv_mask interval must be numpy array of dimension 2 by 3")
        except AttributeError:
            raise ValueError("hsv_mask interval must be numpy array of dimension 2 by 3")

        self.__hsv_mask_interval = hsv_mask_interval

        self.use_trackbars = use_trackbar
        if self.use_trackbars:
            param_dict = {'Hue_low': [hsv_mask_interval[0, 0], MIN_HUE, MAX_HUE],
                          'Saturation_low': [hsv_mask_interval[0, 1], MIN_SATURATION, MAX_SATURATION],
                          'Value_low': [hsv_mask_interval[0, 2], MIN_VALUE, MAX_VALUE],
                          'Hue_high': [hsv_mask_interval[1, 0], MIN_HUE, MAX_HUE],
                          'Saturation_high': [hsv_mask_interval[1, 1], MIN_SATURATION, MAX_SATURATION],
                          'Value_high': [hsv_mask_interval[1, 2], MIN_VALUE, MAX_VALUE],
                          'Num_stripes': [self.__num_stripes, MIN_NUM_STRIPES, MAX_NUM_STRIPES]}

            self.trackbar = Trackbar(param_dict)

        # ROI
        if len(roi_interval) != 3:
            raise ValueError("roi has to be tuple or list of 3 elements between 0 and 1")
        if all(x <= 1 for x in roi_interval) and all(x >= 0 for x in roi_interval) and roi_interval[0] > 0 and \
                roi_interval[1] + roi_interval[2] < 1:

            self.__roi_interval = roi_interval
        else:
            raise ValueError(
                "ROI has to be tuple or list of 3 elements between 0 and 1 while normalized cropping borders have to be less than 0")

    def __get_roi(self, image):
        """
        crop image to region of interest parameters
        :param image:
        :return: image of region of interest
        """

        height_image, width_image, _ = image.shape

        y1 = int(height_image * self.__roi_interval[2])
        y2 = int(height_image - height_image * self.__roi_interval[1])
        x1 = int(width_image * (1 - self.__roi_interval[0]) / 2.0)
        x2 = int(x1 + width_image * self.__roi_interval[0])
        if x1 == x2:
            x2 = x1 + 1
        if y1 == y2:
            y2 = y1 + 1

        # save roi offset for later recalculation to original image coordinates
        self.__roi_offset = [x1, y1]

        # crop image to roi
        roi = image[y1:y2, x1:x2]
        return roi
        pass

    def __extract_image_curve_points(self, roi):
        """
        extracts points of curve from region of interest
        :param roi: region of interest
        :return: points of curve
        """

        # segment roi in stripes
        height_roi, width_roi, _ = roi.shape
        # convert color to hsv, so it is easier to mask certain colors
        roi_hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)

        # blur image
        try:
            roi_hsv = cv2.GaussianBlur(roi_hsv, (3, 3), cv2.BORDER_DEFAULT)
        except:
            # do nothing
            pass

        # mask image
        mask_roi = self.__mask(roi_hsv)

        curve_points = []
        # number of stripes -> number of points in x direction of car
        for i in range(0, self.__num_stripes, 1):
            # calculate cropping borders for stripes
            y1 = height_roi - (i + 1) * height_roi / self.__num_stripes
            y2 = height_roi - i * height_roi / self.__num_stripes

            y = int((y1 + y2) / 2)  # alternative 1
            # y = int(y2 - 1)  # alternative 2

            # crop image to line
            line = mask_roi[y, :]

            # offset to original image for later recalculation to original image coordinates
            line_offset = [self.__roi_offset[0], y + self.__roi_offset[1]]

            # get edges - return -255 to 255
            edge = cv2.Sobel(line, cv2.CV_64F, 0, 1, ksize=3)

            start_points = np.argwhere(edge > self.EDGE_THRESHOLD)
            end_points = np.argwhere(edge < -self.EDGE_THRESHOLD)

            # start and end points will always have two high values exactly next to each other
            # -> only use the even values
            try:
                points = zip(start_points, end_points)[1::2]
                curve_points.append([])

                for start_point, end_point in points:
                    curve_point = [(start_point[0] + end_point[0]) / 2 + line_offset[0], line_offset[1]]
                    curve_points[i].append(curve_point)
            except IndexError:
                print "error with point extraction"

        curve_points = [x for x in curve_points if x != []]
        return curve_points

    def __mask(self, img_hsv):
        """ mask image by hsv interval """
        if self.__hsv_mask_interval[0, 0] > self.__hsv_mask_interval[1, 0]:
            # because hue value is degree 0 to 180 it should be possible to have a range from e.g. 170 to 20
            # therefore, if Hue_low is larger than hue_high 2 masks are created. one for 170 to 180 and one for 0 to
            # 20. combining both with an or operation gives desired mask

            hsv_interval_1_low = np.copy(self.__hsv_mask_interval[0])
            hsv_interval_1_high = np.copy(self.__hsv_mask_interval[1])

            hsv_interval_2_low = np.copy(self.__hsv_mask_interval[0])
            hsv_interval_2_high = np.copy(self.__hsv_mask_interval[1])

            hsv_interval_1_high[0, 0] = MAX_HUE
            hsv_interval_2_low[0, 0] = MIN_HUE

            mask1 = cv2.inRange(img_hsv, hsv_interval_1_low, hsv_interval_1_high)
            mask2 = cv2.inRange(img_hsv, hsv_interval_2_low, hsv_interval_2_high)

            # add two masks logically
            mask = mask1 | mask2
        else:
            # normal mask
            mask = cv2.inRange(img_hsv, self.__hsv_mask_interval[0], self.__hsv_mask_interval[1])

        # create black edge so that sobel edge detection recognizes it as an edge
        mask[:, 0:2] = 0
        mask[:, -1:-3] = 0

        # show mask result in trackbar window -> will resize window to width of mask!!!
        if self.use_trackbars:
            cv2.imshow(self.trackbar.window_name, mask)
            cv2.waitKey(1)

        return mask

    def detect_curve(self, image):
        # update parameters from trackbar
        self.__update_param()

        roi = self.__get_roi(image)
        curve_points = self.__extract_image_curve_points(roi)

        return curve_points

    def __update_param(self):
        """
            update parameter dictionary
        :return:
        """
        try:
            self.trackbar.update_trackbar_pos()
            param_dict = self.trackbar.param_dict

            self.__hsv_mask_interval[0] = [param_dict["Hue_low"][0], param_dict["Saturation_low"][0],
                                           param_dict["Value_low"][0]]
            self.__hsv_mask_interval[1] = [param_dict["Hue_high"][0], param_dict["Saturation_high"][0],
                                           param_dict["Value_high"][0]]

            self.__num_stripes = param_dict["Num_stripes"][0]

        except AttributeError:
            pass


class CurveEstimator:
    def __init__(self, camera_info, h_matrix, distortion_input=True):
        self.world_projector = WorldProjector(camera_info, h_matrix, distortion_input)

    def estimate_curve(self, image_curve_points):

        if len(image_curve_points) != 0:
            num_curves = max(len(x) for x in image_curve_points)
        else:
            return []

        # separate curve into multiple possible paths -> multiple y values for one x value
        #  try to define roi so that a curve with angles greater that 90 deg are not visible
        # FIXME not the best implementation find better one!!!
        curve_points = [[] for i in range(num_curves)]
        # project points to world coordinates
        for image_curve_point_same_x_value in image_curve_points:
            for curve_num, image_curve_point in enumerate(image_curve_point_same_x_value):
                world_point = self.world_projector.pixel2ground(image_curve_point)
                curve_points[curve_num].append(Point2D([world_point[0], world_point[1]]))

        for curve_id, curve in enumerate(curve_points):
            num_points = len(curve)
            # only proceed if at least 3 points in curve
            if len(curve) < 3:
                continue
            for i in range(0, num_points, 1):

                # define points for derivative
                if i == 0:
                    p1 = curve[i]
                    p2 = curve[i + 1]
                elif i == num_points - 1:
                    p1 = curve[i - 1]
                    p2 = curve[i]
                else:
                    p1 = curve[i - 1]
                    p2 = curve[i + 1]
                # calc derivative
                slope = diff(p1, p2)
                curve_points[curve_id][i].slope = slope

                # radius requires 3 points - cant use end points
                if i > 0 and i < num_points - 1:
                    circle = Circle2D(curve[i - 1], curve[i], curve[i + 1])
                    curve_points[curve_id][i].circle = circle

            curve_points[curve_id][0].circle = curve_points[curve_id][1].circle
            curve_points[curve_id][num_points - 1].circle = curve_points[curve_id][num_points - 2].circle

        return curve_points


def diff(p1, p2):
    """

    :param p1: poin2D object with first value x and second value y
    :param p2: same as p1
    :return: float - slope f'(x) ~= (y2-y1)/(x2-x1)
    """
    try:
        if isinstance(p1, Point2D) and isinstance(p2, Point2D):
            return (p2.y - p1.y) / (p2.x - p1.x)
        else:
            raise ValueError("input must be 2 point2D objects")
    except ZeroDivisionError:
        raise ValueError("points must have different x-values")


def midpoint(p1, p2):
    """

    :param p1: list of shape [x,y]
    :param p2: same as p1
    :return: point2D - midpoint between both points
    """
    if isinstance(p1, Point2D) and isinstance(p2, Point2D):
        return Point2D([(p1.x + p2.x) / 2, (p1.y + p2.y) / 2])
    else:
        raise ValueError("input must be 2 point2D objects")


def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


class Point2D(object):

    def __init__(self, point):
        if not isinstance(point[0], list) and not isinstance(point[1], list):
            self.x = float(point[0])
            self.y = float(point[1])
        else:
            raise ValueError("points must be 1D list or tuple of length 2")


class CurvePoint2D(Point2D):
    def __init__(self, point):
        self.slope = None
        self.circle = None
        super(CurvePoint2D, self).__init__(point)


class Circle2D:

    def __init__(self, p1, p2, p3):
        if isinstance(p1, Point2D) and isinstance(p2, Point2D) and isinstance(p3, Point2D):
            self.calc_circle(p1, p2, p3)
        else:
            raise ValueError("input must be 3 point2D objects")

    def calc_circle(self, p1, p2, p3):

        """

        :param p1: list of shape [x,y] with first value the variable (e.g. x) and second function value (e.g. y)
        :param p2: same as p1
        :param p3: same as p1
        :return: center of circle and radius
        """

        # calculate midpoints of points
        mp1 = midpoint(p1, p2)
        mp2 = midpoint(p2, p3)

        # calculate orthogonal gradient of midpoint lines
        try:
            grad_L1 = -1 / diff(p1, p2)
            grad_L2 = -1 / diff(p2, p3)

            if isclose(grad_L1, grad_L2):
                self.center = Point2D([float("inf"), float("inf")])
                self.radius = float("inf")
                return

            # no edge case
            x = 1 / (grad_L1 - grad_L2) * (-mp1.y + mp2.y + grad_L1 * mp1.x - grad_L2 * mp2.x)
            y = grad_L1 * (x - mp1.x) + mp1.y
        except ZeroDivisionError:
            # go through edge cases:
            gradp1p2 = diff(p1, p2)
            gradp2p3 = diff(p2, p3)
            if isclose(gradp1p2, 0.0) and isclose(gradp2p3, 0.0):
                self.center = Point2D([float("inf"), float("inf")])
                self.radius = float("inf")
                return
            elif isclose(gradp1p2, 0.0):
                grad_L2 = -1 / gradp2p3
                x = mp1.x
                y = grad_L2 * (x - mp2.x) + mp2.y
            elif isclose(gradp2p3, 0.0):
                grad_L1 = -1 / gradp1p2
                x = mp2.x
                y = grad_L1 * (x - mp1.x) + mp1.y


        # calculate radius
        radius = math.sqrt(math.pow(p1.x - x, 2) + math.pow(p1.y - y, 2))

        self.center = Point2D([x, y])
        self.radius = radius
