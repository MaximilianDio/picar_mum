import cv2
import numpy as np
from world_projection import WorldProjector


class CurvePointExtractor:

    def __init__(self, hsv_mask_interval, num_stripes, roi_interval):
        """
        :param num_stripes: amount of stripes in which the roi will be disected
        :param roi_interval: list/ tuple of cropping parameters with length 3
         with values between 0 an 1:
         (normalized width, normalized cropping border bottom, normalized cropping border top)
        """
        self.EDGE_THRESHOLD = 200

        self.__num_stripes = num_stripes

        self.__hsv_mask_interval = hsv_mask_interval

        # ROI
        if len(roi_interval) != 3:
            raise TypeError("roi has to be tuple or list of 3 elements between 0 and 1")
        if all(x <= 1 for x in roi_interval) and all(x >= 0 for x in roi_interval) and roi_interval[0] > 0 and \
                roi_interval[1] + roi_interval[2] < 1:

            self.__roi_interval = roi_interval
        else:
            raise TypeError(
                "ROI has to be tuple or list of 3 elements between 0 and 1 while normalized cropping borders have to be less than 0")

    def __get_roi(self, image):
        """

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

        # segment roi in stripes
        height_roi, width_roi = mask_roi.shape

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
            for start_point, end_point in zip(start_points, end_points)[1::2]:
                try:
                    curve_point = [(start_point[0] + end_point[0]) / 2 + line_offset[0], line_offset[1]]
                except ReferenceError:
                    print "error with point extraction"

            curve_points.append(curve_point)

        return curve_points

    def __mask(self, img_hsv):
        # TODO implement a better masking function
        return cv2.inRange(img_hsv, self.__hsv_mask_interval[0], self.__hsv_mask_interval[1])

    def detect_curve(self, image):
        roi = self.__get_roi(image)
        curve_points = self.__extract_image_curve_points(roi)

        return curve_points


class CurveEstimator:
    def __init__(self, camera_info, h_matrix):
        self.world_projector = WorldProjector(camera_info, h_matrix)

    def estimate_curve(self, image_curve_points):
        # TODO: project points to world coordinates

        # TODO: curve fitting

        pass