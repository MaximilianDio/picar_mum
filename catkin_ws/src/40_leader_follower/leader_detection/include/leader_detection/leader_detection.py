#!/usr/bin/env python
import numpy as np
import cv2
import yaml


class LeaderGetter(object):
    def __init__(self, config_path):
        pass
    #
    #     self._config_path = config_path
    #
    #     self.horizon_rel_height = 0.0
    #     self.bottom_cut_rel_height = 0.0
    #     self.hsv_lower_red_1 = 0
    #     self.hsv_upper_red_1 = 0
    #     self.hsv_lower_red_2 = 0
    #     self.hsv_upper_red_2 = 0
    #
    #     self.contour_area_thresh_red = 1.0
    #     self.load_parameters()
    #
    # def load_parameters(self):
    #     with open(self._config_path, "r") as f:
    #         data = yaml.safe_load(f)
    #         self.horizon_rel_height = data["horizon_rel_height"]
    #         self.bottom_cut_rel_height = data["bottom_cut_rel_height"]
    #         self.hsv_lower_red_1 = np.array(data["hsv_lower_red_1"])
    #         self.hsv_upper_red_1 = np.array(data["hsv_upper_red_1"])
    #         self.hsv_lower_red_2 = np.array(data["hsv_lower_red_2"])
    #         self.hsv_upper_red_2 = np.array(data["hsv_upper_red_2"])
    #         self.contour_area_thresh_red = data["contour_area_thresh_red"]
    #
    # def process_image(self, img_rgb):
    #     (height, width) = img_rgb.shape[:2]
    #
    #     slice_y1 = int(height * self.horizon_rel_height)
    #     slice_y2 = int(height * self.bottom_cut_rel_height)
    #
    #     img_rgb = img_rgb[slice_y1:slice_y2]
    #
    #     # convert color to hsv, so it is easier to mask certain colors
    #     img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2HSV)
    #
    #     mask_red = self._mask_red(img_hsv)
    #
    #     contour_list_red = cv2.findContours(mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]
    #
    #     if len(contour_list_red) == 0:
    #         return None
    #
    #     contour_list_red_sorted = sorted(contour_list_red, key=cv2.contourArea, reverse=True)
    #
    #     contour_red_biggest = contour_list_red_sorted[0]
    #
    #     area_red = cv2.contourArea(contour_red_biggest)
    #
    #     if area_red >= width * height * self.contour_area_thresh_red:
    #         momenents_red = cv2.moments(contour_red_biggest)
    #         center_x = momenents_red["m10"] / momenents_red["m00"]
    #         return (center_x, slice_y2)
    #
    #     return None
    #
    # def _mask_red(self, img_hsv):
    #     mask_1 = cv2.inRange(img_hsv, self.hsv_lower_red_1, self.hsv_upper_red_1)
    #     mask_2 = cv2.inRange(img_hsv, self.hsv_lower_red_2, self.hsv_upper_red_2)
    #
    #     mask = np.bitwise_or(mask_1, mask_2)
    #
    #     return mask
