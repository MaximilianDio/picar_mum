#!/usr/bin/env python
import numpy as np
import cv2


class LeaderGetter(object):
    def __init__(self, params):

        self.update_params(params)

        # TODO move following parameters in update params
        self.dp = 2
        self.minDist = 10
        self.param1 = 50
        self.param2 = 30
        self.MIN_CIRCLE_RADIUS = 1
        self.MAX_CIRCLE_RADIUS = 100

        #  TODO find proper values
        self.lower_blue1 = np.array([110, 130, 20])
        self.upper_blue1 = np.array([130, 255, 255])
        self.lower_blue2 = np.array([0, 20, 0])
        self.upper_blue2 = np.array([20, 255, 20])

        self.lower_green1 = np.array([50, 130, 20])
        self.upper_green1 = np.array([70, 255, 255])
        self.lower_green2 = np.array([0, 0, 20])
        self.upper_green2 = np.array([20, 20, 255])

    def update_params(self, params):
        # TODO make it more modular!
        try:
            self.crop_ratio_middle_x = params["crop_ratio_middle_x"]
            self.crop_ratio_top_y = params["crop_ratio_top_y"]
        except Exception as exc:
            print exc
            self.crop_ratio_middle_x = 1.0
            self.crop_ratio_top_y = 1.0

    def process_image(self, img_rgb):
        """
            :param image matrix of piCar
            :returns position (px) of both tracking balls [x_blue y_blue x_green y_green]
            """

        # crop image
        img_rgb = self.__crop_image(img_rgb)

        # convert color to hsv, so it is easier to mask certain colors
        img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2HSV)

        img_hsv = cv2.GaussianBlur(img_hsv, (5, 5), cv2.BORDER_DEFAULT)

        cv2.imshow('rgb', img_rgb)

        # mask images
        mask_blue = self._mask(img_hsv, "blue")
        mask_green = self._mask(img_hsv, "green")

        cv2.imshow('green', mask_green)
        cv2.imshow('blue', mask_blue)

        cv2.imshow('green_blur', mask_green)
        cv2.imshow('blue_blur', mask_blue)

        # TODO find good values
        circles_blue = cv2.HoughCircles(mask_blue, cv2.HOUGH_GRADIENT, self.dp, self.minDist,
                                        param1=self.param1, param2=self.param2, minRadius=self.MIN_CIRCLE_RADIUS,
                                        maxRadius=self.MAX_CIRCLE_RADIUS)
        circles_green = cv2.HoughCircles(mask_green, cv2.HOUGH_GRADIENT, self.dp, self.minDist,
                                         param1=self.param1, param2=self.param2, minRadius=self.MIN_CIRCLE_RADIUS,
                                         maxRadius=self.MAX_CIRCLE_RADIUS)

        cv2.imshow('green', mask_green)
        cv2.imshow('blue', mask_blue)
        # ensure at least some circles were found
        if not (circles_blue is None or circles_green is None):
            output = img_rgb.copy()
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

            # show the output image
            cv2.imshow("output", output)
        cv2.waitKey(1)

        if not (circles_blue is None or circles_green is None):
            for (x, y, r) in circles_blue:
                x_blue = x
                y_blue = y
            for (x, y, r) in circles_green:
                x_green = x
                y_green = y

            return x_blue, y_blue, x_green, y_green
        else:
            return -1, -1, -1, -1

    # TODO implement a better masking function
    def _mask(self, img_hsv, color):
        if color == "blue":
            mask = cv2.inRange(img_hsv, self.lower_blue1, self.upper_blue1)
        elif color == "green":
            mask = cv2.inRange(img_hsv, self.lower_green1, self.upper_green1)
        else:
            print "color " + color + "currently not able to process"
            return img_hsv

        # TODO image has to be BW
        return mask

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
