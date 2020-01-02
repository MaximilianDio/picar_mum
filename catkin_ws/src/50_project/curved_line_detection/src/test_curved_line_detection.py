import unittest
from curved_line_detection import CurvePointExtractor, Trackbar
import cv2
import numpy as np
import time

if __name__ == "__main__":
    image_file = "Bild3.png"
    image = cv2.imread(image_file)

    hsv_interval = np.matrix([[90, 124, 124], [180, 255, 255]])
    curve_detector = CurvePointExtractor(hsv_interval, 10, [1, 0.35, 0.35])

    while True:
        t0 = time.time()
        curve_points = curve_detector.detect_curve(image)
        print time.time() - t0

        # DEBUG
        for curve_point in curve_points:
            cv2.circle(image, (curve_point[0], curve_point[1]), 5, (255, 0, 0))
        cv2.imshow("line", image)
        cv2.waitKey(0)
