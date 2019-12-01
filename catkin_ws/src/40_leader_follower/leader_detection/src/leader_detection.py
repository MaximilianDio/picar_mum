#!/usr/bin/env python
import numpy as np
import cv2


class LeaderGetter(object):
    def __init__(self, params):
        self.update_params(params)

        # TODO move following parameters in update params
        self.lower_blue = np.array([0, 100, 0])
        self.upper_blue = np.array([200, 255, 255])
        self.angle_threshold = np.pi / 6  # rad

    def update_params(self, params):
        # TODO make it more modular!
        try:
            self.crop_ratio_middle_x = params["crop_ratio_middle_x"]
            self.crop_ratio_top_y = params["crop_ratio_top_y"]
        except Exception as exc:
            print exc
            self.crop_ratio_middle_x = 1.0
            self.crop_ratio_top_y = 1.0

    '''
    :param image matrix of piCar
    :returns position (px) and orientation (rad) of leader in image coordinates
    '''

    def process_image(self, img_rgb):
        # TODO

        # crop image
        img_rgb = self.__crop_image(img_rgb)

        # convert color to hsv, so it is easier to mask certain colors
        img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2HSV)

        # # convert to grayscale
        # img_gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        #
        # # blur image so edge detection is better
        mask = cv2.GaussianBlur(img_hsv, (3, 3), cv2.BORDER_DEFAULT)

        mask = self._mask(mask)

        # edge detection using canny edge detector
        low_threshold = 50
        high_threshold = 150
        edges = cv2.Canny(mask, low_threshold, high_threshold)

        # transform
        rho = 1  # distance resolution in pixels of the Hough grid
        theta = np.pi / 180  # angular resolution in radians of the Hough grid
        threshold = 20 # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 150  # minimum number of pixels making up a line
        max_line_gap = 10 # maximum gap in pixels between connectable line segments
        line_image = np.copy(img_rgb) * 0  # creating a blank to draw lines on

        # Run Hough on edge detected image
        # Output "lines" is an array containing endpoints of detected line segments
        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                                min_line_length, max_line_gap)

        if not (lines is None):
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            filtered_line, line_center = self.filter_lines(lines)
            # for line in filtered_lines:


            for x1, y1, x2, y2 in filtered_line:
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 3)
                orientation = np.arctan(float(y2-y1)/float(x2-x1))

            cv2.circle(line_image, (line_center[0], line_center[1]), 10, (0, 0, 255), 3)

            text = np.array2string(line_center)
            line_image = cv2.putText(line_image, text, (line_center[0] + 10, line_center[1] + 50),
                                     cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

            # Draw the lines on the  image
            lines_edges = cv2.addWeighted(img_rgb, 0.5, line_image, 1, 0)

            # shows image but use only for debugging purposes!
            cv2.imshow('test', lines_edges)
            cv2.waitKey(1)

            #  r:  position,   orientation
            return (line_center[0], line_center[1]), orientation





    def __crop_image(self, img):
        # get dimension of image
        (height, width) = img.shape[:2]

        # calculate Dx and Dy (see leader_detection.yaml)
        Dx = width * self.crop_ratio_middle_x
        Dy = height * self.crop_ratio_top_y

        # determine slices (has to be int)
        slice_x1 = int((width - Dx) / 2)
        slice_x2 = int((width + Dx) / 2)
        slice_y1 = int(0)
        slice_y2 = int(Dy)

        # crop image
        cropped_img = img[slice_y1:slice_y2, slice_x1:slice_x2]

        return cropped_img

    def filter_lines(self, lines):
        filtered_lines = []
        line_center = [[], []]

        for line in lines:
            for x1, y1, x2, y2 in line:
                if x2 - x1 == 0:
                    continue
                if np.abs(float(y2 - y1) / float(x2 - x1)) <= np.tan(float(self.angle_threshold)):
                    filtered_lines.append(line)
                    x_center = np.abs((x2 + x1) / 2)
                    y_center = np.abs((y2 + y1) / 2)

                    line_center[0].append(x_center)
                    line_center[1].append(y_center)

        line_center = np.array(line_center)

        # choose lowest line (max y-center)
        idx = np.argmax(line_center[1, :])
        return filtered_lines[idx], line_center[:, idx]

    def _mask(self, img_hsv):

        mask = cv2.inRange(img_hsv, self.lower_blue, self.upper_blue)

        return mask
