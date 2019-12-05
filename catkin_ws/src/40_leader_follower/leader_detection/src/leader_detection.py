#!/usr/bin/env python
import numpy as np
import cv2


def nothing(x):
    pass


class LeaderGetter(object):
    def __init__(self, params):

        self.update_params(params)

        self.use_trackbars = True

        if self.use_trackbars == True:
            self.__create_trackbar()
            self.__update_trackbars()

    def __create_trackbar(self):
        self.TRACKBAR_NAMES = ['H_blue_l', 'S_blue_l', 'V_blue_l',
                               'H_blue_h', 'S_blue_h', 'V_blue_h',
                               'H_green_l', 'S_green_l', 'V_green_l',
                               'H_green_h', 'S_green_h', 'V_green_h',
                               'minDist', 'param1', 'param2', 'min_Radius', 'max_Radius']

        self.HSV_values = [self.mask_param_blue_low[0], self.mask_param_blue_low[1], self.mask_param_blue_low[2],
                           self.mask_param_blue_high[0], self.mask_param_blue_high[1], self.mask_param_blue_high[2],
                           self.mask_param_green_low[0], self.mask_param_green_low[1], self.mask_param_green_low[2],
                           self.mask_param_green_high[0], self.mask_param_green_high[1], self.mask_param_green_high[2],
                           self.minDist, self.param1, self.param2, self.min_circle_radius,
                           self.max_circle_radius]

        self.WIDTH = 300  # px
        self.HEIGHT = 600  # px
        self.trackbar_window = np.zeros((self.WIDTH, self.HEIGHT, 3), np.uint8)
        self.window_name = "trackbars"
        cv2.namedWindow(self.window_name)
        for name, value in zip(self.TRACKBAR_NAMES, self.HSV_values):
            print name
            cv2.createTrackbar(name, self.window_name, value, 255, nothing)

    def __update_trackbars(self):
        cv2.imshow(self.window_name, self.trackbar_window)
        cv2.waitKey(1)

    def __update_trackbar_pos(self):

        # TODO make it more modular

        # get current position of slider
        # HSV blue 1
        self.mask_param_blue_low[0] = cv2.getTrackbarPos("H_blue_l", self.window_name)
        self.mask_param_blue_low[1] = cv2.getTrackbarPos("S_blue_l", self.window_name)
        self.mask_param_blue_low[2] = cv2.getTrackbarPos("V_blue_l", self.window_name)
        # HSV blue 2
        self.mask_param_blue_high[0] = cv2.getTrackbarPos("H_blue_h", self.window_name)
        self.mask_param_blue_high[1] = cv2.getTrackbarPos("S_blue_h", self.window_name)
        self.mask_param_blue_high[2] = cv2.getTrackbarPos("V_blue_h", self.window_name)
        # HSV green 1
        self.mask_param_green_low[0] = cv2.getTrackbarPos("H_green_l", self.window_name)
        self.mask_param_green_low[1] = cv2.getTrackbarPos("S_green_l", self.window_name)
        self.mask_param_green_low[2] = cv2.getTrackbarPos("V_green_l", self.window_name)
        # HSV green 2
        self.mask_param_green_high[0] = cv2.getTrackbarPos("H_green_h", self.window_name)
        self.mask_param_green_high[1] = cv2.getTrackbarPos("S_green_h", self.window_name)
        self.mask_param_green_high[2] = cv2.getTrackbarPos("V_green_h", self.window_name)

        # update circle detection parameters
        # self.dp = cv2.getTrackbarPos("dp", self.window_name)
        self.minDist = cv2.getTrackbarPos("minDist", self.window_name)
        self.param1 = cv2.getTrackbarPos("param1", self.window_name)
        self.param2 = cv2.getTrackbarPos("param2", self.window_name)
        self.min_circle_radius = cv2.getTrackbarPos("min_circle_radius", self.window_name)
        self.max_circle_radius = cv2.getTrackbarPos("max_circle_radius", self.window_name)


    def update_params(self, params):
        # TODO make it more modular!
        try:
            self.crop_ratio_middle_x = params["crop_ratio_middle_x"]
            self.crop_ratio_top_y = params["crop_ratio_top_y"]
            # circle detection parameters
            self.dp = params["dp"]
            self.minDist = params["minDist"]
            self.param1 = params["param1"]
            self.param2 = params["param2"]
            self.min_circle_radius = params["min_circle_radius"]
            self.max_circle_radius = params["max_circle_radius"]
            # masking parameters blue_ball
            self.mask_param_blue_low = np.array(params["mask_param_blue_low_H"],params["mask_param_blue_low_S"] ,params["mask_param_blue_low_V"])
            self.mask_param_blue_high = np.array(params["mask_param_blue_high_H"],params["mask_param_high_low_S"] ,params["mask_param_blue_high_V"])
            # masking parameters green ball
            self.mask_param_green_low = np.array(params["mask_param_green_low_H"],params["mask_param_green_low_S"] ,params["mask_param_green_low_V"])
            self.mask_param_green_high = np.array(params["mask_param_green_high_H"],params["mask_param_green_low_S"] ,params["mask_param_green_high_V"])
        except Exception as exc:
            print exc
            self.crop_ratio_middle_x = 1.0
            self.crop_ratio_top_y = 1.0
            self.dp = 2
            self.minDist = 10
            self.param1 = 50
            self.param2 = 20
            self.min_circle_radius = 1
            self.max_circle_radius = 30
            # masking parameters blue_ball
            self.mask_param_blue_low = np.array([100, 215, 0])
            self.mask_param_blue_high = np.array([130, 255, 255])

            # masking parameters green ball
            self.mask_param_green_low = np.array([35, 230, 20])
            self.mask_param_green_high = np.array([70, 255, 255])

    def process_image(self, img_rgb):
        """
            :param img_rgb: image of picar
            :returns position (px) of both tracking balls [x_blue y_blue x_green y_green]
            """
        if self.use_trackbars == True:
            # update trackbar postions
            self.__update_trackbar_pos()

        # crop image
        img_rgb, x_offset = self.__crop_image(img_rgb)

        # convert color to hsv, so it is easier to mask certain colors
        img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2HSV)

        img_hsv = cv2.GaussianBlur(img_hsv, (5, 5), cv2.BORDER_DEFAULT)

        # mask images
        mask_blue = self._mask(img_hsv, "blue")
        mask_green = self._mask(img_hsv, "green")

        mask_blue = cv2.GaussianBlur(mask_blue, (3, 3), cv2.BORDER_DEFAULT)
        mask_green = cv2.GaussianBlur(mask_green, (3, 3), cv2.BORDER_DEFAULT)

        if self.use_trackbars == True:
            cv2.imshow("mask_blue", mask_blue)
            cv2.imshow("mask_green", mask_green)


        return self.__get_circle_pos(img_rgb, mask_blue, mask_green, x_offset)


    # TODO implement a better masking function
    def _mask(self, img_hsv, color):
        if color == "blue":
            mask = cv2.inRange(img_hsv, self.mask_param_blue_low, self.mask_param_blue_high)
        elif color == "green":
            mask = cv2.inRange(img_hsv, self.mask_param_green_low, self.mask_param_green_high)
        else:
            print "color " + color + "currently not able to process"
            return img_hsv

        return mask

    # FIXME balls at which are farther away from the center will appear as ellipses -will not be detected correctly
    def __get_circle_pos(self, img_rgb, mask_blue, mask_green, x_offset):
        '''
        :param img_rgb:
        :param mask_blue:
        :param mask_green:
        :return: x_blue, y_blue, x_green, y_green, image_with_detection
        '''
        output = img_rgb.copy()

        circles_blue = cv2.HoughCircles(mask_blue, cv2.HOUGH_GRADIENT, self.dp, self.minDist,
                                        param1=self.param1, param2=self.param2, minRadius=self.min_circle_radius,
                                        maxRadius=self.max_circle_radius)
        circles_green = cv2.HoughCircles(mask_green, cv2.HOUGH_GRADIENT, self.dp, self.minDist,
                                         param1=self.param1, param2=self.param2, minRadius=self.min_circle_radius,
                                         maxRadius=self.max_circle_radius)

        # ensure at least some circles were found
        if not (circles_blue is None or circles_green is None):

            # convert the (x, y) coordinates and radius of the circles to integers
            circles_blue = np.round(circles_blue[0, :]).astype("int")
            circles_green = np.round(circles_green[0, :]).astype("int")

            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles_blue:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(output, (x, y), r, (0, 0, 150), 2)
                cv2.rectangle(output, (x - 1, y - 1), (x + 1, y + 1), (0, 0, 150), -1)

            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles_green:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(output, (x, y), r, (0, 0, 255), 2)
                cv2.rectangle(output, (x - 1, y - 1), (x + 1, y + 1), (0, 0, 255), -1)

            for (x, y, r) in circles_blue:
                x_blue = x
                y_blue = y
            for (x, y, r) in circles_green:
                x_green = x
                y_green = y

            return (x_blue + x_offset, y_blue, x_green + x_offset, y_green), output
        else:
            return None, output

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

        return cropped_img, slice_x1
