import numpy as np
import cv2
import yaml


class Normalizer(object):
    def __init__(self, config_path):
        pass

    def process_image(self, img):
        img_normalized = self._normalize_color(img)
        return img_normalized

    def _normalize_color(self, img):
        # TODO
        return img


