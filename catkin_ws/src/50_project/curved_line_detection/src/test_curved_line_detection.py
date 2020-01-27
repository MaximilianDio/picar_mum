import unittest
from curved_line_detection import CurvePointExtractor, diff, midpoint, CurvePoint2D, Circle2D
import cv2
import numpy as np
import time

if __name__ == "__main__":
    a = CurvePoint2D([-1, -1])
    b = CurvePoint2D([0, 0])
    c = CurvePoint2D([1, 0])
    print diff(a, b)
    print diff(b, c)

    # print midpoint(a,b)

    circle = Circle2D(a, b, c)

    print circle.center.x
    print circle.center.y
    print circle.radius

    curve_points = [[CurvePoint2D([1.0, 0.0]), CurvePoint2D([2, 0.1]), CurvePoint2D([3.0, 0.3])],
                    [CurvePoint2D([1.0, 0.0]), CurvePoint2D([2, 0.1]), CurvePoint2D([3.0, 0.1]), CurvePoint2D([4.0, -0.1])],
                    [CurvePoint2D([1.0, 0.0]), CurvePoint2D([2, 0.1])]]
    num_curves = len(curve_points)

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

    print curve_points
    # image_file = "Bild3.png"
    # image = cv2.imread(image_file)
    #
    # hsv_interval = np.matrix([[90, 124, 124], [180, 255, 255]])
    # curve_detector = CurvePointExtractor(hsv_interval, 10, [1, 0.35, 0.35],0)
    #
    # while True:
    #     t0 = time.time()
    #     curve_points = curve_detector.detect_curve(image)
    #     print time.time() - t0
    #
    #     # DEBUG
    #     for curve_points_same_x_value in curve_points:
    #         for i,curve_point in enumerate(curve_points_same_x_value):
    #             cv2.circle(image, (curve_point[0], curve_point[1]), 5, (124*i, 0, 0))
    #     cv2.imshow("line", image)
    #     cv2.waitKey(0)
